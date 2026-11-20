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
