"""The presence row (palimpsest api/SPEC-v1.md 4.14, 6.4) takes no cell any
screen uses. test_tui.cpp is built with the Control API and run twice: as
it is, and with a guarded request waiting ("presence": the row's text at its
longest, 37 characters). Every screen is rendered with pyte both ways and
compared cell by cell:

  - the only cells that may differ are on row 13, columns 43 to 79;
  - on a screen that differs, those cells are blank (a space on the frame's
    blue) without the request, so the row covers nothing that was there;
  - a screen that differs shows the whole text there, never a part of it
    (the part right of a box drawn over the main menu), and at least one
    screen (the main menu) shows it.

    python tests/host/presence_rows.py <source dir> [extra g++ defines...]

Prints "N checks, M failed"; COULD NOT RUN (exit 98) without pyte or g++.
"""
import os, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TEXT = 'JP2? setup.ctrlg #2147483647 0123abcd'
ROW, COL0, COL1 = 12, 42, 78          # 0-based: row 13, columns 43 to 79 (1-based)


def main():
    try:
        import pyte
    except ImportError:
        print('COULD NOT RUN: no pyte'); return 98
    src = sys.argv[1]
    defs = sys.argv[2:] or ['-DATABOY_CTL=1']
    tmp = tempfile.mkdtemp(prefix='ataboy-presence-')
    exe = os.path.join(tmp, 'test_tui.exe')
    r = subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O1', '-w', *defs, '-I', os.path.join(HERE, 'mock'),
                        '-I', HERE, '-I', src, '-o', exe, os.path.join(HERE, 'test_tui.cpp')])
    if r.returncode:
        print('COULD NOT RUN: test_tui did not build'); return 98
    shots = {}
    for mode in ('plain', 'presence'):
        out = os.path.join(tmp, mode)
        os.makedirs(out)
        subprocess.run([exe, out] + (['presence'] if mode == 'presence' else []), check=True)
        shots[mode] = {f[:-4]: open(os.path.join(out, f), 'rb').read() for f in os.listdir(out)}

    def render(data):
        scr = pyte.Screen(80, 24)
        pyte.ByteStream(scr).feed(data)
        return scr

    checks = failures = 0
    shown = 0
    for name in sorted(shots['plain']):
        checks += 1
        if name not in shots['presence']:
            failures += 1; print(f'FAIL: {name}: not drawn with a request waiting'); continue
        a, b = render(shots['plain'][name]), render(shots['presence'][name])
        bad, under = [], []
        for y in range(24):
            for x in range(80):
                ca, cb = a.buffer[y][x], b.buffer[y][x]
                if (ca.data, ca.fg, ca.bg, ca.bold) == (cb.data, cb.fg, cb.bg, cb.bold):
                    continue
                if y == ROW and COL0 <= x <= COL1:
                    if ca.data != ' ' or ca.bg != 'blue':
                        under.append((x + 1, ca.data, ca.bg))
                else:
                    bad.append((y + 1, x + 1))
        if bad:
            failures += 1; print(f'FAIL: {name}: {len(bad)} cells outside row 13, 43..79 differ, first {bad[:4]}')
        checks += 1
        if under:
            failures += 1; print(f'FAIL: {name}: the row covers {ascii(under[:6])}')
        row = ''.join(b.buffer[ROW][x].data for x in range(COL0, COL1 + 1))
        differs = any((a.buffer[ROW][x].data, a.buffer[ROW][x].fg, a.buffer[ROW][x].bg) !=
                      (b.buffer[ROW][x].data, b.buffer[ROW][x].fg, b.buffer[ROW][x].bg) for x in range(COL0, COL1 + 1))
        if row.rstrip() == TEXT:
            shown += 1
        checks += 1
        if differs and row.rstrip() != TEXT:
            failures += 1; print(f'FAIL: {name}: a part of the row shows: {ascii(row.strip())}')
    checks += 1
    if shown == 0:
        failures += 1; print('FAIL: no screen shows the presence row')
    print(f'presence row: {len(shots["plain"])} screens, {shown} show the row, the rest unchanged')
    print(f'{checks} checks, {failures} failed')
    return min(failures, 255)


if __name__ == '__main__':
    sys.exit(main())
