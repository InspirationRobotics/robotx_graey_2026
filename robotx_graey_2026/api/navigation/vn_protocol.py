"""Checked VN ASCII transactions; callers must exclusively own the serial port."""
import time


def packet(body):
    checksum = 0
    for byte in body.encode('ascii'):
        checksum ^= byte
    return f'${body}*{checksum:02X}\r\n'.encode('ascii')


def checked(line):
    try:
        if not line.startswith(b'$'):
            raise ValueError('Missing VN packet marker')
        body, checksum = line.decode('ascii').strip().removeprefix('$').split('*')
        if packet(body).decode().strip().split('*')[1] != checksum.upper():
            raise ValueError('VN checksum mismatch')
        return body.split(',')
    except (UnicodeError, ValueError) as error:
        raise ValueError('Invalid VN reply') from error


def transact(port, body, timeout=2.):
    prefix = body.split(',')[:2]
    port.write(packet(body))
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        try:
            fields = checked(raw)
        except ValueError:
            continue
        if fields[0] == 'VNERR':
            raise OSError('VN rejected command: '+','.join(fields))
        if fields[:2] == prefix:
            if body.startswith('VNWRG,') and fields != body.split(','):
                raise OSError('VN write acknowledgement differs from request')
            return fields[2:]
    raise OSError('VN command timed out: '+','.join(prefix))


def relative_heading(port):
    before = transact(port, 'VNRRG,35')
    if len(before) != 4 or before[0] != '1':
        raise OSError('Unexpected VN heading control register')
    expected = [before[0], '1', *before[2:]]
    if before != expected:
        transact(port, 'VNWRG,35,'+','.join(expected))
    if transact(port, 'VNRRG,35') != expected:
        raise OSError('Relative heading mode readback failed')
    return expected
