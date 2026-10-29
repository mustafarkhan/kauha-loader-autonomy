"""Train the bucket-filling policy with PPO and export it for the vehicle.

Method follows the RL bucket-filling / excavation literature (ETH Zurich,
Tampere University): PPO on a fast bucket-pile model with heavy domain
randomisation over material, pile geometry, entry speed and sensor noise.

Model selection uses a validation seed set; the final comparison against the
tuned rule-based operator uses held-out test seeds and fixed materials.

The exported artefact is a plain NumPy file (weights + observation
normalisation), so the autonomy computer runs the policy without PyTorch.

    python -m kauha.train.train_bucket --steps 1500000 --out models/bucket_policy.npz
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

from kauha.autonomy.bucket_policy import BucketPolicy
from kauha.train.bucket_env import BucketFillEnv, ScriptedOperator, evaluate, scripted_operator

VAL_SEED, TEST_SEED = 9000, 20000
TEST_MATERIALS = {
    "sand": dict(material="sand", density_t_m3=1.6, k_resist=1.0, repose_deg=33, height_m=3.0),
    "gravel": dict(material="gravel", density_t_m3=1.75, k_resist=1.35, repose_deg=36, height_m=3.0),
    "crushed_rock": dict(material="crushed_rock", density_t_m3=1.6, k_resist=2.1, repose_deg=42, height_m=3.0),
    "wood_chips": dict(material="wood_chips", density_t_m3=0.35, k_resist=0.45, repose_deg=40, height_m=3.0),
}


def export(model: PPO, venv: VecNormalize, path: Path, residual_scale: float = 0.0) -> None:
    sd = model.policy.state_dict()
    arrays = {
        "residual_scale": np.float32(residual_scale),
        "base_params": np.array(ScriptedOperator.DEFAULT, dtype=np.float32),
        "obs_mean": venv.obs_rms.mean.astype(np.float32),
        "obs_var": venv.obs_rms.var.astype(np.float32),
        "clip_obs": np.float32(venv.clip_obs),
        "eps": np.float32(venv.epsilon),
    }
    layer = 0
    for key in sorted(k for k in sd if k.startswith("mlp_extractor.policy_net") and k.endswith("weight")):
        arrays[f"w{layer}"] = sd[key].cpu().numpy()
        arrays[f"b{layer}"] = sd[key.replace("weight", "bias")].cpu().numpy()
        layer += 1
    arrays[f"w{layer}"] = sd["action_net.weight"].cpu().numpy()
    arrays[f"b{layer}"] = sd["action_net.bias"].cpu().numpy()
    arrays["n_layers"] = np.int32(layer + 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, **arrays)


def compare(policy, n: int) -> dict:
    out = {"randomised": {"learned": evaluate(policy, n=n, seed=TEST_SEED),
                          "baseline": evaluate(scripted_operator, n=n, seed=TEST_SEED)}}
    for name, m in TEST_MATERIALS.items():
        kw = {"fixed_material": m}
        out[name] = {"learned": evaluate(policy, n=n // 4, seed=TEST_SEED, env_kwargs=kw),
                     "baseline": evaluate(scripted_operator, n=n // 4, seed=TEST_SEED, env_kwargs=kw)}
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=1_500_000)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", type=Path, default=Path("models/bucket_policy.npz"))
    ap.add_argument("--residual", type=float, default=0.0,
                    help="residual scale on top of the tuned rule-based operator (0 = learn from scratch)")
    ap.add_argument("--test-episodes", type=int, default=200)
    args = ap.parse_args()

    torch.set_num_threads(2)
    venv = make_vec_env(BucketFillEnv, n_envs=8, seed=args.seed, env_kwargs={"residual_scale": args.residual})
    # Rewards are already O(1-10); normalising them (and clipping at 10) would
    # flatten the terminal reward that carries the objective.
    venv = VecNormalize(venv, norm_obs=True, norm_reward=False, clip_obs=10.0)
    model = PPO("MlpPolicy", venv, seed=args.seed, n_steps=2048, batch_size=2048, n_epochs=10,
                gamma=1.0, gae_lambda=0.95, learning_rate=3e-4, ent_coef=0.0, clip_range=0.2,
                policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[128, 128]),
                                   activation_fn=torch.nn.Tanh, log_std_init=-0.5),
                verbose=0)
    t0 = time.time()
    log, best = [], (-1.0, 0)
    chunk = 131_072
    ckpt = args.out.with_suffix(".ckpt.npz")
    done = 0
    while done < args.steps:
        model.learn(total_timesteps=chunk, reset_num_timesteps=False)
        done += chunk
        export(model, venv, ckpt, args.residual)
        r = evaluate(BucketPolicy.load(ckpt).act, n=80, seed=VAL_SEED)
        r.update(step=done, minutes=round((time.time() - t0) / 60, 1),
                 train_ep_return=float(np.mean([e["r"] for e in model.ep_info_buffer])))
        log.append(r)
        if r["score_mean"] > best[0]:
            best = (r["score_mean"], done)
            shutil.copy(ckpt, args.out)
        print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}), flush=True)
    ckpt.unlink(missing_ok=True)
    policy = BucketPolicy.load(args.out)
    result = {"selected_step": best[1], "validation_score": best[0], "log": log,
              "test": compare(policy.act, args.test_episodes),
              "baseline_params": dict(zip(("throttle", "pen_lift", "boom1", "f_thr", "load_end", "t_end"),
                                          ScriptedOperator.DEFAULT))}
    (args.out.parent / "bucket_training.json").write_text(json.dumps(result, indent=1))
    t = result["test"]
    print(json.dumps({k: {a: round(v[a]["score_mean"], 3) for a in v} for k, v in t.items()}))


if __name__ == "__main__":
    main()
