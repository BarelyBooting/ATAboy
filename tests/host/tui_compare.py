"""Compare what two builds of the TUI put on an 80x24 terminal.

    python tests/host/tui_compare.py <old src> <new src> [--expect screen:row,row ...]

--expect names rows (1-based) of a screen where the new build is meant to
show something different; differences there are counted but do not fail.
A screen name of * means every screen (e.g. *:1 for a new version banner).

Builds test_tui.cpp against each source tree, renders every captured screen
with pyte (the emulator the console harness uses), and reports:

  1. cells that differ between old and new as the console harness sees them
     (pyte). A blank cell that was in the default colours and is now a blank
     painted with the frame's blue is the intended change and is counted
     separately, as is a blank whose (invisible) foreground colour changed.
     ANY other difference (a character, a colour on a non-blank
     cell, the red selection, the yellow picker) is a failure;
  2. cells left in the terminal's DEFAULT background with background-colour
     erase turned off (GNU screen with bce off, issue #9), for old and new.
     Any such cell in the new build is a failure.

A screen only the new build captures (a new feature's screen, which the old
source cannot draw) is listed as NEW and checked for (2) only. A screen the
old build has and the new one lacks is a failure.

Note: pyte's ESC[2J only repaints cells that were written before, so on a
fresh screen pyte shows the frame's interior in the default colours even
with background-colour erase. That is why (1) sees the fill at all.
"""
import os, subprocess, sys, tempfile
import pyte

HERE = os.path.dirname(os.path.abspath(__file__))


class NoBceScreen(pyte.Screen):
    """Erases fill with the default colours, like screen with bce off."""
    def _plain(self, fn, *a, **k):
        saved = self.cursor.attrs
        self.cursor.attrs = self.default_char
        try:
            fn(*a, **k)
        finally:
            self.cursor.attrs = saved

    def erase_in_display(self, *a, **k):
        self._plain(super().erase_in_display, *a, **k)

    def erase_in_line(self, *a, **k):
        self._plain(super().erase_in_line, *a, **k)

    def erase_characters(self, *a, **k):
        self._plain(super().erase_characters, *a, **k)


def build_and_capture(src, work):
    os.makedirs(work, exist_ok=True)
    exe = os.path.join(work, 'test_tui.exe')
    cxx = os.environ.get('CXX', 'g++')
    subprocess.run([cxx, '-std=c++17', '-O1', '-w', '-I', os.path.join(HERE, 'mock'),
                    '-I', HERE, '-I', src, '-o', exe, os.path.join(HERE, 'test_tui.cpp')],
                   check=True)
    out = os.path.join(work, 'screens')
    os.makedirs(out, exist_ok=True)
    subprocess.run([exe, out], check=True)
    return {f[:-4]: open(os.path.join(out, f), 'rb').read() for f in os.listdir(out)}


def render(data, cls=pyte.Screen):
    scr = cls(80, 24)
    pyte.ByteStream(scr).feed(data)
    return scr


def cells(scr):
    return [[scr.buffer[y][x] for x in range(80)] for y in range(24)]


def main():
    old_src, new_src = sys.argv[1], sys.argv[2]
    expect = {}
    args = sys.argv[3:]
    while args:
        if args[0] != '--expect' or len(args) < 2:
            raise SystemExit('usage: tui_compare.py OLD NEW [--expect screen:row,row ...]')
        scr, rows = args[1].split(':')
        expect[scr] = {int(r) - 1 for r in rows.split(',')}
        args = args[2:]
    tmp = tempfile.mkdtemp(prefix='ataboy-tui-')
    old = build_and_capture(old_src, os.path.join(tmp, 'old'))
    new = build_and_capture(new_src, os.path.join(tmp, 'new'))
    assert old and set(old) <= set(new), 'screens missing from the new build, or none at all'
    bad = 0
    print(f'{"screen":18} {"bytes old":>9} {"new":>6} {"blank->filled":>13} {"invisible":>9} {"expected":>8} {"other diffs":>11}  {"default-bg cells, bce off":>26}')
    for name in sorted(old):
        a, b = cells(render(old[name])), cells(render(new[name]))
        diff, filled, invisible, expected = [], 0, 0, 0
        for y in range(24):
            for x in range(80):
                ca, cb = a[y][x], b[y][x]
                if ca == cb:
                    continue
                if (ca.data == ' ' and cb.data == ' ' and ca.bg == 'default'
                        and cb.bg in ('blue', 'black')):
                    filled += 1
                elif (ca.data == ' ' and cb.data == ' ' and ca.bg == cb.bg
                        and not (ca.reverse or cb.reverse or ca.underscore or cb.underscore
                                 or ca.strikethrough or cb.strikethrough)):
                    invisible += 1      # a blank's foreground colour cannot be seen
                elif y in expect.get(name, ()) or y in expect.get('*', ()):
                    expected += 1
                else:
                    diff.append((y, x))
        na = sum(c.bg == 'default' for row in cells(render(old[name], NoBceScreen)) for c in row)
        nb = sum(c.bg == 'default' for row in cells(render(new[name], NoBceScreen)) for c in row)
        print(f'{name:18} {len(old[name]):9} {len(new[name]):6} {filled:13} {invisible:9} {expected:8} {len(diff):11}  {"old " + str(na):>13} {"new " + str(nb):>12}')
        for y, x in diff[:5]:
            print(f'    row {y+1} col {x+1}: old {a[y][x]} new {b[y][x]}')
        if diff:
            bad += 1
        if nb:
            bad += 1
    for name in sorted(set(new) - set(old)):
        nb = sum(c.bg == 'default' for row in cells(render(new[name], NoBceScreen)) for c in row)
        print(f'{name:18} {"NEW":>9} {len(new[name]):6} {"":13} {"":9} {"":8} {"":11}  {"new " + str(nb):>26}')
        if nb:
            bad += 1
    print('OK' if not bad else f'{bad} problem(s)')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
