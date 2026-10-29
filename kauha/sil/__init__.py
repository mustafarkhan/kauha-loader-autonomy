"""Multi-process software-in-the-loop: the machine, the safety PLC, the autonomy
computer and the remote station as separate operating-system processes.

They talk the way the real units do: J1939 frames on a shared CAN bus
(python-can's UDP-multicast interface stands in for the cable) and, between
the machine and the safety PLC, a separate I/O link that stands in for the
hardwired OSSD inputs and relay outputs. Killing a process is then a real
fault, not a flag in a loop.
"""
