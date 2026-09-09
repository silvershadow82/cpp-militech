#!/usr/bin/env python3
"""UDP fan-out для MAVLink: один дрон -> кілька споживачів.

simulation11 шле телеметрію рівно на одну адресу (--mavlink-remote), а показати
її треба одночасно і чекеру, і QGroundControl. Цей релей слухає один порт,
роздає кожну датаграму дрона всім призначенням і повертає дрону все, що
надходить від них (зокрема COMMAND_ACK на команду скиду).

    ./tools/mavlink_fanout.py <listen_port> <host:port> [<host:port> ...]

Приклад:

    ./tools/mavlink_fanout.py 14540 127.0.0.1:14550 127.0.0.1:14560
    #                         ^слухає  ^QGroundControl  ^чекер
"""
import socket
import sys


def main():
    if len(sys.argv) < 3:
        print(__doc__.strip(), file=sys.stderr)
        return 2

    listen_port = int(sys.argv[1])
    targets = []
    for spec in sys.argv[2:]:
        host, _, port = spec.rpartition(":")
        targets.append((host, int(port)))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", listen_port))
    print(f"fan-out :{listen_port} -> " + ", ".join(f"{h}:{p}" for h, p in targets), flush=True)

    drone = None  # адреса дрона вивчається з першої датаграми
    target_set = set(targets)

    while True:
        data, src = sock.recvfrom(4096)
        if src in target_set:
            # відповідь від споживача (наприклад COMMAND_ACK) - назад дрону
            if drone is not None:
                sock.sendto(data, drone)
            continue

        if drone != src:
            drone = src
            print(f"drone: {src[0]}:{src[1]}", flush=True)
        for dst in targets:
            sock.sendto(data, dst)


if __name__ == "__main__":
    sys.exit(main())
