#!/usr/bin/env python3
"""Relay a fixed public RTCM3 station into the existing local NTRIP caster.

No GGA is sent upstream; use a fixed station, not a VRS/NEAR mountpoint.
"""
import argparse
import base64
from collections import Counter
import json
import logging
import signal
import socket
import threading
import time

LOG = logging.getLogger('ntrip_relay')


def read_response(sock):
    """Consume only the response header, including single-line NTRIP v1 ICY."""
    header = bytearray()
    while len(header) < 16384:
        byte = sock.recv(1)
        if not byte:
            raise ConnectionError('connection closed during response header')
        header.extend(byte)
        if header.endswith(b'\r\n') and header.startswith(b'ICY '):
            break
        if header.endswith(b'\r\n\r\n'):
            break
    else:
        raise ConnectionError('response header exceeds 16 KiB')
    first = bytes(header).split(b'\r\n')[0]
    if first != b'ICY 200 OK' and not (
        first.startswith(b'HTTP/1.') and first.split()[1:2] == [b'200']
    ):
        raise ConnectionError('caster rejected stream: '+first.decode(errors='replace'))
    lower = bytes(header).lower()
    if b'transfer-encoding:' in lower or b'gnss/sourcetable' in lower or b'text/plain' in lower:
        raise ConnectionError('expected unencoded RTCM stream, received HTTP content')


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc


def extract_frames(buffer):
    """Return CRC-valid complete RTCM3 frames; retain incomplete trailing data."""
    frames = []
    while buffer:
        if buffer[0] != 0xD3:
            del buffer[0]
            continue
        if len(buffer) < 3:
            break
        if buffer[1] & 0xFC:
            del buffer[0]
            continue
        size = ((buffer[1] & 3) << 8 | buffer[2]) + 6
        if len(buffer) < size:
            break
        frame = bytes(buffer[:size])
        if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], 'big'):
            del buffer[0]
            continue
        frames.append(frame)
        del buffer[:size]
    return frames


def relay(args, stop):
    with open(args.caster_config) as config_file:
        config = json.load(config_file)
    tokens = [token for token, mount in config['tokens_source'].items()
              if mount in (args.local_mountpoint, '*')]
    if not tokens:
        raise ValueError('local mountpoint has no source token in caster config')
    password = tokens[0]
    for value in (args.upstream_host, args.upstream_mountpoint, args.local_mountpoint, password):
        if not value or any(c.isspace() for c in value):
            raise ValueError('host, mountpoints and source token must be nonempty without whitespace')
    port = config['listen_port']
    while not stop.is_set():
        try:
            with socket.create_connection((args.upstream_host, args.upstream_port), timeout=10) as upstream:
                upstream.settimeout(15)
                request = (f'GET /{args.upstream_mountpoint} HTTP/1.0\r\n'
                           f'Host: {args.upstream_host}:{args.upstream_port}\r\n'
                           'User-Agent: NTRIP njord-relay/1.0\r\n')
                if args.upstream_user:
                    auth = base64.b64encode(f'{args.upstream_user}:{args.upstream_password}'.encode()).decode()
                    request += f'Authorization: Basic {auth}\r\n'
                upstream.sendall((request+'\r\n').encode())
                read_response(upstream)
                with socket.create_connection((args.local_host, port), timeout=5) as downstream:
                    downstream.settimeout(5)
                    downstream.sendall((f'SOURCE {password} /{args.local_mountpoint}\r\n'
                                        'Source-Agent: NTRIP njord-relay/1.0\r\n\r\n').encode())
                    read_response(downstream)
                    LOG.info('Relaying %s:%s/%s -> %s:%s/%s', args.upstream_host,
                             args.upstream_port, args.upstream_mountpoint,
                             args.local_host, port, args.local_mountpoint)
                    buffer = bytearray()
                    counts = Counter()
                    last_report = time.monotonic()
                    last_frame = last_report
                    upstream.settimeout(1)
                    while not stop.is_set():
                        try:
                            chunk = upstream.recv(8192)
                        except socket.timeout:
                            if time.monotonic()-last_frame > 15:
                                raise ConnectionError('no CRC-valid RTCM frame for 15 seconds')
                            continue
                        if not chunk:
                            raise ConnectionError('upstream closed stream')
                        buffer.extend(chunk)
                        for frame in extract_frames(buffer):
                            downstream.sendall(frame)
                            if len(frame) >= 8:
                                counts[(frame[3] << 4) | (frame[4] >> 4)] += 1
                            last_frame = time.monotonic()
                        if time.monotonic()-last_frame > 15:
                            raise ConnectionError('no CRC-valid RTCM frame for 15 seconds')
                        if time.monotonic()-last_report >= 30:
                            LOG.info('Forwarded CRC-valid RTCM message counts: %s', dict(counts))
                            last_report = time.monotonic()
        except (OSError, ConnectionError) as exc:
            LOG.warning('%s; reconnecting in 5 seconds', exc)
            stop.wait(5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--caster-config', required=True)
    parser.add_argument('--local-host', default='127.0.0.1')
    parser.add_argument('--local-mountpoint', default='RTCM3')
    parser.add_argument('--upstream-host', default='geortk.jp')
    parser.add_argument('--upstream-port', type=int, default=2101)
    parser.add_argument('--upstream-mountpoint', default='tamtam')
    parser.add_argument('--upstream-user', default='')
    parser.add_argument('--upstream-password', default='')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(levelname)s %(message)s')
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    relay(args, stop)


if __name__ == '__main__':
    main()
