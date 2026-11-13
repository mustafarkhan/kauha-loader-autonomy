"""Frame authentication and the segment guard (kauha/can/auth.py, guard.py)."""
from __future__ import annotations

import random

import pytest

from kauha.can.auth import (
    RFC4493_KEY,
    TEST_KEYS,
    AuthSender,
    aes128_encrypt,
    auth_frame_id,
    cmac,
    expand_key,
    load_keys,
    session_key,
)
from kauha.can.codec import Codec
from kauha.can.guard import PAIR_WINDOW_S, SegmentGuard

MSG = bytes.fromhex("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51"
                    "30c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710")
CH = bytes(range(1, 9))


@pytest.mark.req("AR-06")
def test_aes_and_cmac_match_the_published_vectors():
    # FIPS-197 appendix B and RFC 4493 section 4
    assert aes128_encrypt(expand_key(RFC4493_KEY), bytes.fromhex("3243f6a8885a308d313198a2e0370734")).hex() \
        == "3925841d02dc09fbdc118597196a0b32"
    for n, tag in ((0, "bb1d6929e95937287fa37d129b756746"), (16, "070a16b46b4d4144f79bdd9dd04a287c"),
                   (40, "dfa66747de9ae63030ca32611497c827"), (64, "51f0bebf7e3b9d92fc49741779363cfe")):
        assert cmac(RFC4493_KEY, MSG[:n]).hex() == tag


def _setup():
    guard = SegmentGuard(dict(TEST_KEYS), CH)
    sender = AuthSender(0x80, TEST_KEYS[0x80])
    sender.on_challenge(CH)
    return guard, sender, Codec()


@pytest.mark.req("AR-06")
def test_genuine_command_is_forwarded_and_everything_else_is_not():
    guard, sender, codec = _setup()
    fid, d = codec.encode("DriveCmd", dict(SpeedReq=1.5, Mode=1))
    assert guard.from_open(0.0, fid, d) is None                      # nothing until the companion arrives
    assert guard.from_open(0.0005, *sender.sign(fid, d)) == (fid, d)
    # the same pair again: stale freshness
    auth = sender.sign(fid, d)
    guard.from_open(0.1, fid, d)
    assert guard.from_open(0.1005, *auth) == (fid, d)
    guard.from_open(0.2, fid, d)
    assert guard.from_open(0.2005, *auth) is None and guard.stats.dropped_stale == 1
    # payload changed after signing
    fid2, d2 = codec.encode("DriveCmd", dict(SpeedReq=3.0, Mode=1))
    guard.from_open(0.3, fid2, d2)
    assert guard.from_open(0.3005, *sender.sign(fid, d)) is None and guard.stats.dropped_bad_mac == 1
    # companion too late
    auth = sender.sign(fid, d)
    guard.from_open(0.4, fid, d)
    assert guard.from_open(0.4 + PAIR_WINDOW_S + 0.001, *auth) is None
    # a safety-segment identifier on the open segment
    sid, sd = codec.encode("SafetyStatus", dict(State=0, SpeedLimit=3.0))
    assert guard.from_open(0.5, sid, sd) is None and guard.stats.intrusions == 1
    # another sender's key does not sign DriveCmd
    other = AuthSender(0x80, TEST_KEYS[0x82])
    other.on_challenge(CH)
    guard.from_open(0.6, fid, d)
    assert guard.from_open(0.6005, *other.sign(fid, d)) is None


@pytest.mark.req("AR-06")
def test_a_new_challenge_invalidates_recorded_frames():
    guard, sender, codec = _setup()
    fid, d = codec.encode("ImplementCmd", dict(BoomRate=0.5))
    auth = sender.sign(fid, d)
    restarted = SegmentGuard(dict(TEST_KEYS), bytes(range(9, 17)))
    restarted.from_open(0.0, fid, d)
    assert restarted.from_open(0.0005, *auth) is None and restarted.stats.dropped_bad_mac == 1
    assert session_key(TEST_KEYS[0x80], CH) != session_key(TEST_KEYS[0x80], bytes(range(9, 17)))
    sender.on_challenge(bytes(range(9, 17)))
    restarted.from_open(0.1, fid, d)
    assert restarted.from_open(0.1005, *sender.sign(fid, d)) == (fid, d)


@pytest.mark.req("AR-06")
def test_random_companions_are_never_accepted():
    guard, _, codec = _setup()
    rnd = random.Random(7)
    fid, d = codec.encode("DriveCmd", dict(SpeedReq=1.0, Mode=1))
    for k in range(20000):
        guard.from_open(k * 0.01, fid, d)
        fake = bytes([0x10]) + bytes(rnd.randrange(256) for _ in range(7))
        assert guard.from_open(k * 0.01 + 0.0005, auth_frame_id(0x80), fake) is None
    assert guard.stats.forwarded == 0


def test_sender_without_a_challenge_does_not_sign_and_keys_load(tmp_path):
    s = AuthSender(0x80, TEST_KEYS[0x80])
    assert s.sign(0x0CFF1080, bytes(8)) is None
    f = tmp_path / "k.json"
    f.write_text('{"0x80": "%s"}' % RFC4493_KEY.hex())
    assert load_keys(f) == {0x80: RFC4493_KEY}
    f.write_text('{"0x80": "abcd"}')
    with pytest.raises(ValueError):
        load_keys(f)
