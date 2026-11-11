#!/usr/bin/env python3
"""Kestrel flight software as a separate SITL process.

Launched by the sim through ``world.recipe(...)``. Receives navigation
messages over UDP, runs guidance, and replies with throttle and attitude
commands. The seeded fault (for ``fsw``-layer faults) is read from the
campaign run context, because the Monte Carlo runner plans the recipe once
and only patches each run's environment.
"""

from __future__ import annotations

import json
import os
import signal
import socket
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from faults import from_meta  # noqa: E402
from fsw import STATE_SIZE, Guidance, pack_reply, unpack_state  # noqa: E402

STATE_PORT = int(os.environ.get("ELODIN_MC_PORT_STATE", "9103"))
COMMAND_PORT = int(os.environ.get("ELODIN_MC_PORT_COMMAND", "9102"))
STOP = threading.Event()


def run_fault():
    path = os.environ.get("ELODIN_MONTE_CARLO_CONTEXT")
    if not path:
        return from_meta({})
    return from_meta(json.loads(Path(path).read_text()).get("meta", {}))


def main() -> None:
    signal.signal(signal.SIGTERM, lambda *_: STOP.set())
    signal.signal(signal.SIGINT, lambda *_: STOP.set())
    guidance = Guidance(run_fault())
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("127.0.0.1", STATE_PORT))
    recv_sock.settimeout(0.05)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_seq, last_reply = -1, b""
    while not STOP.is_set():
        try:
            raw, _ = recv_sock.recvfrom(1024)
        except TimeoutError:
            continue
        if len(raw) < STATE_SIZE:
            continue
        seq, nav = unpack_state(raw)
        if seq < last_seq:
            continue
        if seq > last_seq:  # a retransmit of the last state gets the same answer
            last_seq, last_reply = seq, pack_reply(seq, guidance.step(nav))
        send_sock.sendto(last_reply, ("127.0.0.1", COMMAND_PORT))


if __name__ == "__main__":
    main()
