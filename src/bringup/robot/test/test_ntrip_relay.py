"""Protocol boundary regressions: HTTP bytes must never reach the GNSS stream."""
import importlib.util
from pathlib import Path
import socket

import pytest

spec = importlib.util.spec_from_file_location('ntrip_relay', Path(__file__).parents[1]/'scripts/ntrip_relay.py')
relay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay)


@pytest.mark.parametrize('header', [b'ICY 200 OK\r\n', b'HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n'])
def test_header_preserves_first_binary_byte(header):
    a, b = socket.socketpair()
    with a, b:
        b.sendall(header+b'\xd3\x00')
        relay.read_response(a)
        assert a.recv(2) == b'\xd3\x00'


@pytest.mark.parametrize('header', [b'SOURCETABLE 200 OK\r\n\r\n', b'HTTP/1.1 401 Unauthorized\r\n\r\n', b'HTTP/1.1 200 OK\r\nTransfer-Encoding: gzip, chunked\r\n\r\n'])
def test_rejects_non_rtcm_response(header):
    a, b = socket.socketpair()
    with a, b:
        b.sendall(header)
        with pytest.raises(ConnectionError):
            relay.read_response(a)


def test_chunked_response_and_fragmented_payload():
    a, b = socket.socketpair()
    with a, b:
        b.sendall(b'HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n')
        assert relay.read_response(a)
    decoder = relay.ChunkDecoder()
    payload = b'\xd3\x00\x02\x3e\xd0'
    frame = payload + relay.crc24q(payload).to_bytes(3, 'big')
    wire = b'3;foo=bar\r\n' + frame[:3] + b'\r\n5\r\n' + frame[3:] + b'\r\n0\r\nX-Test: ok\r\n\r\n'
    decoded = bytearray()
    for byte in wire:
        decoded.extend(decoder.feed(bytes([byte])))
    assert decoder.done
    assert relay.extract_frames(decoded) == [frame]


@pytest.mark.parametrize('wire', [b'z\r\n', b'1\r\naXX', b'1000001\r\n'])
def test_rejects_invalid_chunks(wire):
    with pytest.raises(ConnectionError):
        relay.ChunkDecoder().feed(wire)


def test_rtcm_crc_and_split_frames():
    assert relay.crc24q(b'123456789') == 0xCDE703
    body = b'\xd3\x00\x02\x3e\xd0'
    frame = body+relay.crc24q(body).to_bytes(3, 'big')
    corrupt = bytearray(frame)
    corrupt[-1] ^= 1
    buffer = bytearray(b'junk')+corrupt+frame[:4]
    assert relay.extract_frames(buffer) == []
    buffer.extend(frame[4:]+frame)
    assert relay.extract_frames(buffer) == [frame, frame]
    assert not buffer


def gga(quality=1):
    body = f'GNGGA,123519,3533.0000,N,13939.0000,E,{quality},12,1.0,10.0,M,39.0,M,,'
    checksum = 0
    for c in body:
        checksum ^= ord(c)
    return f'${body}*{checksum:02X}'


def test_gga_validation_freshness_and_loss():
    source = relay.GgaSource()
    source.update(gga(5))
    assert source.get()[1] == pytest.approx((35.55, 139.65))
    source.update(gga(0))
    assert source.get() is None
    source.update(gga(6))
    assert source.get() is None
    source.update(gga(1)[:-2] + '00')
    assert source.get() is None


def test_station_selection_filters_virtual_invalid_and_authorized():
    rows = [
        'STR;near;near;RTCM 3.3;x;2;GPS;;JPN;35.55;139.58;0;0;;none;N;N;;',
        'STR;far;far;RTCM 3.2;x;2;GPS;;JPN;36.0;140.0;0;0;;none;N;N;;',
        'STR;virtual;vrs;RTCM 3.2;x;2;GPS;;JPN;35.55;139.65;1;1;;none;N;N;;',
        'STR;locked;x;RTCM 3.2;x;2;GPS;;JPN;35.55;139.65;0;0;;none;B;N;;',
        'STR;missing;x;RTCM 3.2;x;2;GPS;;JPN;0;0;0;0;;none;N;N;;',
        'STR;legacy;x;RTCM 2.3;x;2;GPS;;JPN;35.55;139.65;0;0;;none;N;N;;',
    ]
    stations = relay.parse_sourcetable('\n'.join(rows))
    assert {x[0] for x in stations} == {'near', 'far'}
    assert min(stations, key=lambda x: relay.distance_km((35.55,139.65), x[1:]))[0] == 'near'
    assert any(x[0] == 'locked' for x in relay.parse_sourcetable('\n'.join(rows), True))


def test_gga_stale_is_not_reused(monkeypatch):
    source = relay.GgaSource()
    monkeypatch.setattr(relay.time, 'monotonic', lambda: 1.0)
    source.update(gga())
    monkeypatch.setattr(relay.time, 'monotonic', lambda: 12.0)
    assert source.get() is None


def test_vrs_receives_gga_and_rtcm_reaches_local_caster(tmp_path):
    import json
    import threading
    from types import SimpleNamespace
    def server():
        sock = socket.socket()
        sock.bind(('127.0.0.1', 0)); sock.listen(1); sock.settimeout(5)
        return sock
    upstream = server(); caster = server(); stop = threading.Event()
    config = tmp_path/'caster.json'
    config.write_text(json.dumps({'listen_port': caster.getsockname()[1], 'tokens_source': {'secret': 'RTCM3'}}))
    body = b'\xd3\x00\x02\x3e\xd0'
    frame = body + relay.crc24q(body).to_bytes(3, 'big')
    received = {}
    def upstream_worker():
        with upstream, upstream.accept()[0] as conn:
            conn.settimeout(5); data = b''
            while gga().encode() not in data:
                data += conn.recv(4096)
            received['upstream'] = data
            conn.sendall(b'ICY 200 OK\r\n' + frame)
            stop.wait(3)
    def caster_worker():
        with caster, caster.accept()[0] as conn:
            conn.settimeout(5); data = b''
            while b'\r\n\r\n' not in data: data += conn.recv(4096)
            conn.sendall(b'ICY 200 OK\r\n')
            received['rtcm'] = conn.recv(4096)
            stop.set()
    threads = [threading.Thread(target=fn) for fn in (upstream_worker, caster_worker)]
    for thread in threads: thread.start()
    source = relay.GgaSource(); source.update(gga())
    args = SimpleNamespace(caster_config=str(config), local_mountpoint='RTCM3', local_host='127.0.0.1',
                           upstream_host='127.0.0.1', upstream_port=upstream.getsockname()[1],
                           upstream_mountpoint='VRS', upstream_user='', upstream_password='',
                           max_distance_km=50, send_gga=True, require_gga=True)
    relay.relay(args, stop, source)
    for thread in threads: thread.join(5)
    assert gga().encode() in received['upstream']
    assert received['rtcm'] == frame
