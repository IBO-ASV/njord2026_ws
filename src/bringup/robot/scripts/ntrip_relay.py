#!/usr/bin/env python3
"""Relay an NTRIP station into the local caster; support auto selection and GGA."""
import argparse
import base64
from collections import Counter
import json
import logging
import math
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
    if b'gnss/sourcetable' in lower or b'text/plain' in lower:
        raise ConnectionError('expected unencoded RTCM stream, received HTTP content')
    encodings = [line.split(b':', 1)[1].strip() for line in lower.split(b'\r\n')
                 if line.startswith(b'transfer-encoding:')]
    if encodings and encodings != [b'chunked']:
        raise ConnectionError('unsupported HTTP transfer encoding')
    return bool(encodings)


class ChunkDecoder:
    """Decode HTTP chunk framing incrementally without forwarding framing bytes."""
    def __init__(self):
        self.buffer = bytearray()
        self.remaining = None
        self.done = False
        self.trailers = False

    def feed(self, data):
        if self.done:
            raise ConnectionError('data after final HTTP chunk')
        self.buffer.extend(data)
        output = bytearray()
        while True:
            if self.trailers:
                end = self.buffer.find(b'\r\n')
                if end < 0:
                    if len(self.buffer) > 16384:
                        raise ConnectionError('HTTP chunk trailer too long')
                    break
                line = bytes(self.buffer[:end])
                del self.buffer[:end+2]
                if not line:
                    self.done = True
                    if self.buffer:
                        raise ConnectionError('data after final HTTP chunk')
                    break
                continue
            if self.remaining is None:
                end = self.buffer.find(b'\r\n')
                if end < 0:
                    if len(self.buffer) > 128:
                        raise ConnectionError('HTTP chunk header too long')
                    break
                if end > 128:
                    raise ConnectionError('HTTP chunk header too long')
                size = bytes(self.buffer[:end]).split(b';', 1)[0]
                if not size or any(c not in b'0123456789abcdefABCDEF' for c in size):
                    raise ConnectionError('invalid HTTP chunk size')
                self.remaining = int(size, 16)
                if self.remaining > 16 * 1024 * 1024:
                    raise ConnectionError('HTTP chunk too large')
                del self.buffer[:end+2]
                if self.remaining == 0:
                    self.remaining = None
                    self.trailers = True
                    continue
            if len(self.buffer) < self.remaining + 2:
                break
            if self.buffer[self.remaining:self.remaining+2] != b'\r\n':
                raise ConnectionError('invalid HTTP chunk terminator')
            output.extend(self.buffer[:self.remaining])
            del self.buffer[:self.remaining+2]
            self.remaining = None
        return output


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


def parse_gga(sentence):
    """Return (lat, lon) only for a fresh, checksummed, valid position GGA."""
    try:
        text = sentence.strip()
        body, checksum = text[1:].rsplit('*', 1)
        if not text.startswith('$') or len(checksum) != 2:
            return None
        check = 0
        for char in body:
            check ^= ord(char)
        if check != int(checksum, 16):
            return None
        fields = body.split(',')
        if fields[0] not in ('GNGGA', 'GPGGA') or int(fields[6]) not in (1, 2, 4, 5, 8, 9):
            return None
        def coord(value, direction, limit):
            v = float(value)
            degrees = int(v // 100)
            minutes = v - degrees * 100
            result = degrees + minutes / 60
            if not 0 <= minutes < 60 or result > limit or direction not in ('N', 'S', 'E', 'W'):
                raise ValueError('invalid coordinate')
            return -result if direction in ('S', 'W') else result
        if fields[3] not in ('N', 'S') or fields[5] not in ('E', 'W'):
            return None
        lat = coord(fields[2], fields[3], 90)
        lon = coord(fields[4], fields[5], 180)
        return lat, lon
    except (ValueError, IndexError, OverflowError):
        return None


def parse_sourcetable(data, allow_auth=False):
    """Use only fixed RTCM3 streams with valid advertised coordinates."""
    stations = []
    for line in data.splitlines():
        fields = line.strip().split(';')
        if len(fields) < 17 or fields[0] != 'STR':
            continue
        try:
            lat, lon = float(fields[9]), float(fields[10])
            if (not fields[1] or any(c.isspace() for c in fields[1]) or
                not fields[3].upper().startswith('RTCM 3') or fields[11] != '0' or
                (fields[15] != 'N' and not allow_auth) or
                not math.isfinite(lat) or not math.isfinite(lon) or
                not -90 <= lat <= 90 or not -180 <= lon <= 180 or (lat == 0 and lon == 0)):
                continue
            stations.append((fields[1], lat, lon))
        except ValueError:
            continue
    return stations


def distance_km(a, b):
    lat1, lon1, lat2, lon2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    dlat, dlon = lat2-lat1, lon2-lon1
    h = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 12742.0 * math.asin(math.sqrt(min(1.0, max(0.0, h))))


def fetch_sourcetable(host, port, user='', password=''):
    with socket.create_connection((host, port), timeout=10) as sock:
        sock.settimeout(10)
        request = f'GET / HTTP/1.0\r\nHost: {host}:{port}\r\nUser-Agent: NTRIP njord-relay/1.0\r\n'
        if user:
            auth = base64.b64encode(f'{user}:{password}'.encode()).decode()
            request += f'Authorization: Basic {auth}\r\n'
        sock.sendall((request+'\r\n').encode())
        data = bytearray()
        while len(data) < 1024 * 1024:
            chunk = sock.recv(8192)
            if not chunk:
                break
            data.extend(chunk)
            if b'ENDSOURCETABLE' in data:
                break
        if len(data) >= 1024 * 1024 or b'ENDSOURCETABLE' not in data:
            raise ConnectionError('incomplete or oversized sourcetable')
        first = bytes(data).split(b'\r\n', 1)[0]
        if first != b'SOURCETABLE 200 OK' and not (first.startswith(b'HTTP/1.') and first.split()[1:2] == [b'200']):
            raise ConnectionError('caster rejected sourcetable request')
        return bytes(data).decode('utf-8', errors='replace')


class GgaSource:
    def __init__(self):
        self.lock = threading.Lock()
        self.sample = None

    def update(self, sentence):
        coords = parse_gga(sentence)
        if coords is not None:
            with self.lock:
                self.sample = (sentence.strip(), coords, time.monotonic())
        else:
            with self.lock:
                self.sample = None

    def get(self):
        with self.lock:
            sample = self.sample
        return sample if sample and time.monotonic() - sample[2] <= 10 else None


def start_gga_listener(source):
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('ntrip_gga_listener')
    node.create_subscription(String, '/sensor/vehicle_gnss/gga/raw', lambda m: source.update(m.data), 10)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return node, thread


def relay(args, stop, gga_source=None):
    if not args.caster_config:
        raise ValueError('--caster-config is required when relaying')
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
    failures = set()
    while not stop.is_set():
        try:
            mountpoint = args.upstream_mountpoint
            if mountpoint.lower() == 'auto':
                sample = gga_source.get() if gga_source else None
                if sample is None:
                    raise ConnectionError('waiting for a fresh valid UM982 GGA for station selection')
                table = fetch_sourcetable(args.upstream_host, args.upstream_port, args.upstream_user, args.upstream_password)
                stations = parse_sourcetable(table, bool(args.upstream_user))
                candidates = sorted((distance_km(sample[1], (lat, lon)), name) for name, lat, lon in stations)
                candidates = [(distance, name) for distance, name in candidates if distance <= args.max_distance_km]
                available = [(distance, name) for distance, name in candidates if name not in failures]
                if not available and candidates:
                    failures.clear()
                    available = candidates
                if not available:
                    raise ConnectionError('no suitable RTCM3 station within distance limit')
                distance, mountpoint = available[0]
                LOG.info('Selected upstream station %s (%.1f km, approximate sourcetable coordinates)', mountpoint, distance)
            if args.require_gga and (not gga_source or gga_source.get() is None):
                raise ConnectionError('waiting for fresh GGA required by upstream')
            with socket.create_connection((args.upstream_host, args.upstream_port), timeout=10) as upstream:
                upstream.settimeout(15)
                request = (f'GET /{mountpoint} HTTP/1.0\r\n'
                           f'Host: {args.upstream_host}:{args.upstream_port}\r\n'
                           'User-Agent: NTRIP njord-relay/1.0\r\n')
                if args.upstream_user:
                    auth = base64.b64encode(f'{args.upstream_user}:{args.upstream_password}'.encode()).decode()
                    request += f'Authorization: Basic {auth}\r\n'
                upstream.sendall((request+'\r\n').encode())
                sample = gga_source.get() if gga_source and args.send_gga else None
                if sample:
                    upstream.sendall((sample[0]+'\r\n').encode('ascii'))
                last_gga = time.monotonic()
                chunked = read_response(upstream)
                decoder = ChunkDecoder() if chunked else None
                with socket.create_connection((args.local_host, port), timeout=5) as downstream:
                    downstream.settimeout(5)
                    downstream.sendall((f'SOURCE {password} /{args.local_mountpoint}\r\n'
                                        'Source-Agent: NTRIP njord-relay/1.0\r\n\r\n').encode())
                    read_response(downstream)
                    LOG.info('Relaying %s:%s/%s -> %s:%s/%s', args.upstream_host,
                             args.upstream_port, mountpoint,
                             args.local_host, port, args.local_mountpoint)
                    buffer = bytearray()
                    counts = Counter()
                    last_report = time.monotonic()
                    last_frame = last_report
                    upstream.settimeout(1)
                    while not stop.is_set():
                        if args.send_gga and time.monotonic() - last_gga >= 5:
                            sample = gga_source.get() if gga_source else None
                            if sample:
                                upstream.sendall((sample[0]+'\r\n').encode('ascii'))
                            elif args.require_gga:
                                raise ConnectionError('required upstream GGA became invalid or stale')
                            last_gga = time.monotonic()
                        try:
                            chunk = upstream.recv(8192)
                        except socket.timeout:
                            if time.monotonic()-last_frame > 15:
                                raise ConnectionError('no CRC-valid RTCM frame for 15 seconds')
                            continue
                        if not chunk:
                            raise ConnectionError('upstream closed stream')
                        if decoder:
                            chunk = decoder.feed(chunk)
                        buffer.extend(chunk)
                        for frame in extract_frames(buffer):
                            downstream.sendall(frame)
                            if len(frame) >= 8:
                                counts[(frame[3] << 4) | (frame[4] >> 4)] += 1
                            last_frame = time.monotonic()
                        if decoder and decoder.done:
                            raise ConnectionError('upstream finished chunked stream')
                        if time.monotonic()-last_frame > 15:
                            raise ConnectionError('no CRC-valid RTCM frame for 15 seconds')
                        if time.monotonic()-last_report >= 30:
                            LOG.info('Forwarded CRC-valid RTCM message counts: %s', dict(counts))
                            last_report = time.monotonic()
        except (OSError, ConnectionError) as exc:
            if args.upstream_mountpoint.lower() == 'auto' and mountpoint != 'auto':
                failures.add(mountpoint)
            LOG.warning('%s; reconnecting in 5 seconds', exc)
            stop.wait(5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--caster-config', default='')
    parser.add_argument('--local-host', default='127.0.0.1')
    parser.add_argument('--local-mountpoint', default='RTCM3')
    parser.add_argument('--upstream-host', default='geortk.jp')
    parser.add_argument('--upstream-port', type=int, default=2101)
    parser.add_argument('--upstream-mountpoint', default='tamtam')
    parser.add_argument('--upstream-user', default='')
    parser.add_argument('--upstream-password', default='')
    parser.add_argument('--max-distance-km', type=float, default=50.0)
    parser.add_argument('--send-gga', choices=['true', 'false'], default='true')
    parser.add_argument('--require-gga', choices=['true', 'false'], default='false')
    parser.add_argument('--list-stations', action='store_true')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(levelname)s %(message)s')
    if args.list_stations:
        for name, lat, lon in parse_sourcetable(fetch_sourcetable(args.upstream_host, args.upstream_port, args.upstream_user, args.upstream_password), bool(args.upstream_user)):
            print(f"{name}\t{lat:.5f}\t{lon:.5f}")
        return
    args.send_gga = args.send_gga == "true"
    args.require_gga = args.require_gga == "true"
    if args.require_gga and not args.send_gga:
        parser.error("--require-gga requires --send-gga true")
    source = GgaSource()
    node, thread = start_gga_listener(source)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        relay(args, stop, source)
    finally:
        import rclpy
        rclpy.shutdown()
        thread.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
