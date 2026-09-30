"""End to end: the real host client against the firmware's Control port.

    python tests/host/ctl_e2e.py <test_ctl executable> [--client-dir DIR] [--write-corpus DIR]

test_ctl's "serve" mode runs the firmware's own ctl*.c, menus.c, ide.c and
obs.c (ctl_harness.hpp) with the Control port on stdin and stdout. This
script puts palimpsest's host client, api/ataboy_ctl.py, on the other end,
unchanged: its discovery (fed one USB record for the unit), its hello
binding, its strict response parser and closed schemas, its four-state
results and its hash-chained transcript. So what the client accepts is
what the firmware sends, field by field, and the lines the firmware reads
are the ones the client writes.

Stage 2: the ratchet through the client's own request(), each op: wp.on,
pipelined.off, identify.forbid, and unmount and detach naming the mount
read from status (serve starts with a drive mounted through the menus'
own path). The client refuses unmount and detach without expect_mount_seq
before sending anything; a stale one is sent and refused E_STALE by the
firmware, which the client reports as could-not-run.

Then every request the client's own encoder makes for each op in its
tables (Tier S, session and Tier M included, with a stand-in token) goes to
the firmware as raw lines, and each answer is put through the client's
parse_response() and envelope_errors(): the ops this stage carries answer
DONE, and the ones it does not answer E_UNKNOWN_OP, which the client's
hello already told it (ABSENT).

Prints "N checks, M failed" like the other host tests; exit status is the
number failed, or 98 when it could not run (no client, no executable).
--write-corpus writes every line the client sent into the fuzzer's corpus
(client.hex), so the fuzzer mutates the client's real lines too.
"""
import argparse, json, os, queue, subprocess, sys, tempfile, threading, time

checks = failures = 0


def check(cond, what):
    global checks, failures
    checks += 1
    if not cond:
        failures += 1
        print(f'FAIL {what}')


class ServePort:
    """The port the client opens: bytes to test_ctl serve's stdin, and its
    stdout back, read by a thread, handed out like pyserial with a 50 ms
    timeout."""

    def __init__(self, exe, mode='serve'):
        self.p = subprocess.Popen([exe, mode], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.q = queue.Queue()
        self.buf = bytearray()
        self.lines_out = []
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        while True:
            b = self.p.stdout.read1(65536) if hasattr(self.p.stdout, 'read1') else self.p.stdout.read(1)
            if not b:
                return
            self.q.put(b)

    @property
    def in_waiting(self):
        while not self.q.empty():
            self.buf += self.q.get_nowait()
        return len(self.buf)

    def read(self, n=1):
        if not self.in_waiting:
            try:
                self.buf += self.q.get(timeout=0.05)
            except queue.Empty:
                return b''
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def write(self, b):
        self.lines_out.append(bytes(b))
        self.p.stdin.write(b)
        self.p.stdin.flush()
        return len(b)

    def flush(self):
        pass

    def act(self):
        """The human's JP2 act (stage 4): a line serve takes for itself."""
        self.p.stdin.write(b'#ACT\n')
        self.p.stdin.flush()

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=10)
        except Exception:
            self.p.kill()


def raw_request(ctl, exe, mode, op, args, rid=90):
    """One request the client encodes, on a fresh serve port, answered and
    parsed by the client's own parser; for ops outside its tool allowlist
    (selftest.status). The response, or None."""
    port = ServePort(exe, mode)
    line, errs = ctl.encode_request(rid, op, args, None)
    if errs:
        port.close()
        return None
    port.write(line)
    deadline = time.time() + 10
    raw = b''
    while not raw.endswith(b'\n') and time.time() < deadline:
        raw += port.read(65536)
    port.close()
    try:
        resp = ctl.parse_response_line(raw)
        return resp if ctl.envelope_errors(resp) is None else None
    except ctl.ProtocolViolation:
        return None


# Review delta-B LOW-11: the scratch directory is removed when the run ends
# (ATABOY_KEEP_TMP=1 keeps it to look at); before, every run left one in %TEMP%.
_TMPS = []


def _mkdtemp(prefix):
    d = tempfile.mkdtemp(prefix=prefix)
    _TMPS.append(d)
    return d


def _run():
    try:
        return main()
    finally:
        if not os.environ.get('ATABOY_KEEP_TMP'):
            import shutil
            for d in _TMPS:
                shutil.rmtree(d, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('exe')
    ap.add_argument('--client-dir', default=os.environ.get('ATABOY_CTL_CLIENT', 'C:/Development/palimpsest/api'))
    ap.add_argument('--write-corpus', default=None)
    a = ap.parse_args()
    if not os.path.exists(a.exe) and os.path.exists(a.exe + '.exe'):
        a.exe += '.exe'
    if not os.path.exists(os.path.join(a.client_dir, 'ataboy_ctl.py')) or not os.path.exists(a.exe):
        print(f'COULD NOT RUN: no client at {a.client_dir} or no {a.exe}')
        return 98
    sys.path.insert(0, a.client_dir)
    import ataboy_ctl as ctl

    serial = 'E6614103E7A1B22C'                     # test_ctl's board id (ctl_harness.hpp)
    records = [
        ctl.PortRecord('SERVE-TUI', ctl.ATABOY_VID, ctl.ATABOY_PID, serial, '1-1:1.0', 'ATAboy CDC'),
        ctl.PortRecord('SERVE-CTL', ctl.ATABOY_VID, ctl.ATABOY_PID, serial, '1-1:1.3', 'ATAboy Control'),
    ]
    ports = []

    def opener(binding):
        check(binding.device == 'SERVE-CTL' and binding.interface_number == 3, f'bound to {binding}')
        p = ServePort(a.exe)
        ports.append(p)
        return p

    tmp = _mkdtemp('ataboy-e2e-')
    tpath = os.path.join(tmp, 'transcript.jsonl')
    t0 = time.time()
    c, res = ctl.connect(serial, tpath, records_fn=lambda: records, opener=opener)
    check(c is not None and res.state == 'Pass', f'hello: {res!r}')
    if c is None:
        print(f'{checks} checks, {failures} failed')
        return failures or 1
    r = res.result
    check(r['fw']['version'] == '0.6f4-palimpsest' and r['fw']['banner'] == 'v0.6f4 (fork+api)',
          f"version {r['fw']['version']} banner {r['fw']['banner']}")
    check(r['limits']['max_mutating_id'] == 0 and c.ledger.seed_boot == c.boot, 'ids seeded from max_mutating_id')
    jumpers = r['fw']['build_flags'].get('board_jumpers') is True
    tier_r = [op for op, v in ctl.OP_TABLE.items() if v[0] == 'R']
    tier_s = [op for op, v in ctl.OP_TABLE.items() if v[0] == 'S']
    check(sorted(tier_s) == ['detach', 'identify.forbid', 'pipelined.off', 'unmount', 'wp.on'], f'Tier S {tier_s}')
    tier_m = [op for op, v in ctl.OP_TABLE.items() if v[0] == 'M']
    for op in ctl.OP_TABLE:
        if op in tier_m:        # stage 4: no lease yet; without the jumpers, never
            want = {'state': 'LOCKED', 'why': 'NO_LEASE' if jumpers else 'JUMPERS_UNSUPPORTED'}
            check(r['caps'][op] == want, f"hello caps {op}: {r['caps'][op]}")
        else:
            check(r['caps'][op]['state'] == 'OPERATIONAL', f"hello caps {op}: {r['caps'][op]}")
    for op in ctl.HUMAN_ONLY_OPS | ctl.NOT_IN_V1_OPS:
        check(r['caps'][op]['state'] == 'ABSENT', f'{op} not ABSENT')

    # The reads the client's CLI has, through its own request() and schemas.
    for op, args in [('status', {}), ('config.get', {}), ('log.read', {'cursor': 0, 'max_entries': 128}),
                     ('log.read', {'cursor': 3, 'max_entries': 2}), ('hashes.read', {'cursor': 0, 'max_entries': 96}),
                     ('status', {})]:
        x = c.request(op, args)
        check(x.state == 'Pass', f'{op} {args}: {x!r}')
    st = c.request('status').result
    check(st['tui']['state'] == 'MAIN' and st['presence']['waiting'] is False and st['ids']['max_mutating'] == 0 and
          st['mounted'] is True and st['mount_seq'] == 1 and st['setup']['source'] == 'TUI_DETECT_NORMAL',
          f"status values: mounted {st['mounted']} mount_seq {st['mount_seq']} source {st['setup']['source']}")
    lr = c.request('log.read', {'cursor': 0, 'max_entries': 128}).result
    page = bytes.fromhex(lr['page_hex'])
    check(page[:8] == b'ATBLOG01' and lr['entries'] >= 5 and len(page) == 32 + 32 * lr['entries'],
          f"log page: {page[:8]!r}, {lr['entries']} entries")
    # Stage 2, the ratchet, through the client's own request(), strict
    # schemas and semantic checks (a DONE that leaves the wrong state fails).
    e0 = st['epoch']
    x = c.request('wp.on', {'expect_epoch': e0})
    check(x.state == 'Pass' and x.result['wp']['effective'] is True and x.result['wp']['last_change'] == 'API',
          f'wp.on: {x!r}')
    x = c.request('wp.on', {})
    check(x.state == 'Pass' and x.result['changed'] is False, f'wp.on again: {x!r}')
    x = c.request('pipelined.off', {})
    check(x.state == 'Pass' and x.result['pipelined_reads'] is False, f'pipelined.off: {x!r}')
    x = c.request('identify.forbid', {})
    check(x.state == 'Pass' and x.result['identify_forbidden'] is True and x.result['changed'] is True,
          f'identify.forbid: {x!r}')
    st = c.request('status').result
    check(st['rules']['identify_forbidden'] is True and st['rules']['identify_barred'] is True and
          st['wp']['effective'] is True and st['features']['pipelined_reads'] is False, 'status after the ratchet')
    # unmount and detach: refused by the client itself without expect_mount_seq,
    # before anything is written (owner decision on R-API-1 finding 11).
    sent = len(ports[0].lines_out)
    for op in ('unmount', 'detach'):
        x = c.request(op, {})
        check(x.state == 'CouldNotRun' and 'expect_mount_seq' in x.reason and len(ports[0].lines_out) == sent,
              f'{op} without expect_mount_seq: {x!r}, {len(ports[0].lines_out) - sent} lines sent')
    # A stale name: sent, refused by the firmware, nothing changed.
    x = c.request('unmount', {'expect_mount_seq': st['mount_seq'] + 1})
    check(x.state == 'CouldNotRun' and x.response is not None and x.response['err']['code'] == 'E_STALE' and
          len(ports[0].lines_out) == sent + 1,
          f'a stale unmount: {x!r}')
    check(c.request('status').result['mounted'] is True, 'a stale unmount unmounted')
    x = c.request('unmount', {'expect_mount_seq': st['mount_seq']})
    check(x.state == 'Pass' and x.result == {'mounted': False, 'changed': True, 'native_mode_ended': False},
          f'unmount: {x!r}')
    x = c.request('detach', {'expect_mount_seq': st['mount_seq'], 'expect_epoch': c.request('status').result['epoch']})
    check(x.state == 'Pass' and x.result == {'detached': True}, f'detach: {x!r}')
    st2 = c.request('status').result
    check(st2['mounted'] is False and st2['setup']['source'] == 'NONE' and st2['geometry']['total_sectors'] == 0 and
          st2['tui']['state'] == 'MAIN', 'status after detach')
    x = c.request('setup.detect', {})
    check(x.state == 'CouldNotRun', 'setup.detect')
    sj = raw_request(ctl, a.exe, 'serve', 'selftest.status', {})
    check(sj is not None and sj['result']['jumpers'] == ('OK' if jumpers else 'NOT_BUILT') and
          st2['jumpers'] == ({'jp1': 'OPEN', 'jp2': 'OPEN', 'mount_credit_ms_left': 0} if jumpers else
                             {'jp1': 'NOT_BUILT', 'jp2': 'NOT_BUILT', 'mount_credit_ms_left': 0}),
          f"jumpers: selftest {sj and sj['result'].get('jumpers')}, status {st2['jumpers']}")
    c.close()
    v, recs = ctl.verify_transcript(tpath)
    check(v.state == 'Pass', f'transcript: {v.reason}')
    client_lines = list(ports[0].lines_out)

    # Stage 3: the jumpers' real values through the client's closed schemas.
    # JP1 fitted (serve-jp1): LOCK, write protect forced and shown forced by
    # JP1; a pad set up wrong (serve-pads): both UNREADABLE, the lock too.
    if jumpers:
        for mode, want in (('serve-jp1', ('FITTED', 'OPEN', 'OK')), ('serve-pads', ('UNREADABLE', 'UNREADABLE', 'PAD_CONFIG_MISMATCH'))):
            jports = []

            def jopener(binding, mode=mode, jports=jports):
                p = ServePort(a.exe, mode)
                jports.append(p)
                return p
            jc, jres = ctl.connect(serial, os.path.join(tmp, mode + '.jsonl'), records_fn=lambda: records, opener=jopener)
            check(jc is not None and jres.state == 'Pass', f'{mode}: hello {jres!r}')
            if jc is None:
                continue
            js = jc.request('status')
            check(js.state == 'Pass' and js.result['jumpers']['jp1'] == want[0] and js.result['jumpers']['jp2'] == want[1] and
                  js.result['wp']['effective'] is True and js.result['wp']['config'] is True and
                  js.result['wp']['forced_by'] == ['JP1'], f'{mode}: status {js!r}')
            jw = jc.request('wp.on', {})
            check(jw.state == 'Pass' and jw.result['wp']['forced_by'] == ['JP1'] and jw.result['changed'] is False,
                  f'{mode}: wp.on {jw!r}')
            jt = raw_request(ctl, a.exe, mode, 'selftest.status', {})
            check(jt is not None and jt['outcome'] == 'DONE' and jt['result']['jumpers'] == want[2], f'{mode}: selftest {jt!r}')
            for op in ('setup.ctrlg', 'mount'):
                wwhy = 'JP1_FITTED' if mode == 'serve-jp1' else 'JUMPERS_UNREADABLE'
                check(jres.result['caps'][op] == {'state': 'LOCKED', 'why': wwhy}, f"{mode}: caps {op} {jres.result['caps'][op]}")
            jc.close()
            jv, _ = ctl.verify_transcript(os.path.join(tmp, mode + '.jsonl'))
            check(jv.state == 'Pass', f'{mode}: transcript {jv.reason}')
            client_lines += jports[0].lines_out

    # Stage 4: Tier M through the client's own guarded() flow, request first
    # (spec 4.14, 12.2): the client sends the request, reads status.presence,
    # and only when it shows this request (id and digest) prompts the human,
    # whose act (JP2) serve then takes. The declared pair (Q3): one act, the
    # mount on the credit with no second prompt. And a request withdrawn by
    # the client (confirm None, and a declined act) runs nothing.
    if jumpers:
        m_ports = []

        def m_opener(binding):
            p = ServePort(a.exe, 'serve-idle')
            m_ports.append(p)
            return p
        mpath = os.path.join(tmp, 'tier-m.jsonl')
        mc, mres = ctl.connect(serial, mpath, tool='ataboy_manual_chs', records_fn=lambda: records, opener=m_opener)
        check(mc is not None and mres.state == 'Pass', f'Tier M: hello {mres!r}')
        if mc is not None:
            o = mc.open_session()
            check(o.state == 'Pass' and mc.token is not None, f'Tier M: session.open {o!r}')
            prompts = []

            def confirm_act(prompt):
                prompts.append(prompt)
                m_ports[0].act()
                return True
            args = {'cyls': 522, 'heads': 4, 'spt': 40, 'mode': 'idp', 'device': 'master', 'drive_power_cycled': True}
            r1, r2 = mc.setup_ctrlg_then_mount(args, confirm_act)
            check(r1.state == 'Pass' and r1.result['mount_credit'] is True and r1.result['source'] == 'API_CTRLG_IDP',
                  f'Tier M: the pair\'s set-up {r1!r}')
            check(r2 is not None and r2.state == 'Pass' and r2.result['used_credit'] is True and r2.result['mounted'] is True,
                  f'Tier M: the pair\'s mount {r2!r}')
            check(len(prompts) == 1 and prompts[0]['op'] == 'setup.ctrlg' and 'then_mount_expect' in prompts[0] and
                  len(prompts[0]['request_sha256']) == 64, f'Tier M: one prompt for the pair ({len(prompts)})')
            st = mc.request('status')
            check(st.state == 'Pass' and st.result['mounted'] is True and st.result['presence']['waiting'] is False and
                  st.result['setup']['source'] == 'API_CTRLG_IDP', f'Tier M: status after the pair {st!r}')
            # A request the flow never prompts for (confirm None): withdrawn
            # by session.close, nothing runs. (A set-up refused while mounted
            # first: E_STATE at arrival, no prompt.)
            ref = mc.guarded('setup.ctrlg', dict(args, cyls=500), confirm_act)
            check(ref.state == 'CouldNotRun' and ref.response is not None and ref.response['err']['code'] == 'E_STATE' and
                  len(prompts) == 1, f'Tier M: a set-up while mounted {ref!r}')
            mc.close()
        # A fresh unit: a request withdrawn unprompted, and one declined.
        for label, conf in (('never prompted', None), ('declined', lambda prompt: False)):
            w_ports = []

            def w_opener(binding, w_ports=w_ports):
                p = ServePort(a.exe, 'serve-idle')
                w_ports.append(p)
                return p
            wc, wres = ctl.connect(serial, os.path.join(tmp, f'withdrawn-{len(label)}.jsonl'), tool='ataboy_manual_chs',
                                   records_fn=lambda: records, opener=w_opener)
            check(wc is not None and wres.state == 'Pass', f'Tier M {label}: hello')
            if wc is None:
                continue
            wc.open_session()
            wr = wc.guarded('setup.ctrlg', {'cyls': 522, 'heads': 4, 'spt': 40, 'mode': 'idp', 'device': 'master',
                                            'drive_power_cycled': True}, conf)
            check(wr.state == 'CouldNotRun' and wr.response is not None and wr.response['err']['code'] == 'E_LEASE_REVOKED' and
                  wr.response['err'].get('by') == 'CLOSED' and wc.token is None, f'Tier M {label}: {wr!r}')
            wc.open_session()
            ws = wc.request('status')
            check(ws.state == 'Pass' and ws.result['setup']['source'] == 'NONE' and ws.result['presence']['waiting'] is False,
                  f'Tier M {label}: something ran {ws!r}')
            wc.close()
            client_lines += w_ports[0].lines_out
        mv, _ = ctl.verify_transcript(mpath)
        check(mv.state == 'Pass', f'Tier M: transcript {mv.reason}')
        client_lines += m_ports[0].lines_out

    # Every line the client's encoder makes, for every op in its tables, sent
    # raw; each answer through the client's own parser and envelope checks.
    port = ServePort(a.exe)
    samples = {
        'hello': {}, 'status': {}, 'identify.cached': {}, 'config.get': {}, 'taskfile.snapshot': {},
        'log.read': {'cursor': 0, 'max_entries': 1}, 'hashes.read': {'cursor': 0, 'max_entries': 1},
        'debug.e': {}, 'selftest.status': {}, 'chip.info': {}, 'job.last': {},
        'wp.on': {}, 'pipelined.off': {}, 'identify.forbid': {},
        'unmount': {'expect_mount_seq': 1}, 'detach': {'expect_mount_seq': 1},
        'session.open': {'client': 'palimpsest', 'tui_lock': False, 'idle_s': 300}, 'session.close': {},
        'setup.ctrlg': {'cyls': 522, 'heads': 4, 'spt': 40, 'mode': 'native', 'device': 'master',
                        'drive_power_cycled': True, 'then_mount': True},
        'mount': {'expect': {'source': 'API_CTRLG_NATIVE', 'mode': 'CHS', 'device': 'MASTER',
                             'cyls': 522, 'heads': 4, 'spt': 40}},
    }
    rid = 100
    for op in ctl.OP_TABLE:
        rid += 1
        token = '00112233445566778899aabbccddeeff' if op in ctl.TOKEN_OPS else None
        line, errs = ctl.encode_request(rid, op, samples[op], token)
        check(not errs, f'the client would not encode {op}: {errs}')
        port.write(line)
        client_lines.append(line)
        deadline = time.time() + 10
        raw = b''
        while not raw.endswith(b'\n') and time.time() < deadline:
            raw += port.read(65536)
        try:
            resp = ctl.parse_response_line(raw)
            e = ctl.envelope_errors(resp)
        except ctl.ProtocolViolation as x:
            resp, e = None, str(x)
        check(resp is not None and e is None and resp['id'] == rid, f'{op}: the client rejects the answer: {e} {raw[:200]!r}')
        if resp is None:
            continue
        if op == 'taskfile.snapshot':
            # serve starts with a drive mounted (stage 2), and this op is
            # refused while the bus is the host's (spec 4.6); unmount follows.
            check(resp['outcome'] == 'NOT_EXECUTED' and resp['err']['code'] == 'E_STATE',
                  f"{op} while mounted: {resp['outcome']} {resp.get('err')}")
        elif op in tier_r or op in tier_s:
            check(resp['outcome'] == 'DONE', f"{op}: {resp['outcome']} {resp.get('err')}")
            if op in ctl.RESULT_SCHEMA and resp['outcome'] == 'DONE':
                check(ctl.check(resp['result'], ctl.RESULT_SCHEMA[op], 'result') is None,
                      f"{op}: {ctl.check(resp['result'], ctl.RESULT_SCHEMA[op], 'result')}")
        elif op == 'session.open':
            check(resp['outcome'] == 'DONE' and ctl.check(resp['result'], ctl.RESULT_SCHEMA[op], 'result') is None,
                  f"{op}: {resp['outcome']} {resp.get('err')}")
        else:
            # A lease is held (the line above), and these carry a stand-in token.
            check(resp['outcome'] == 'NOT_EXECUTED' and resp['err']['code'] == 'E_LEASE_INVALID',
                  f"{op}: {resp['outcome']} {resp.get('err')}")
    port.close()

    if a.write_corpus:
        os.makedirs(a.write_corpus, exist_ok=True)
        with open(os.path.join(a.write_corpus, 'client.hex'), 'w', newline='\n') as f:
            f.write('# every line api/ataboy_ctl.py sent in ctl_e2e.py (hex); regenerate with --write-corpus\n')
            for l in client_lines:
                f.write(l.rstrip(b'\n').hex() + '\n')
    print(f'end to end with {ctl.CLIENT_VERSION}: {len(client_lines)} client lines, {time.time() - t0:.1f} s')
    print(f'{checks} checks, {failures} failed')
    return min(failures, 255)


if __name__ == '__main__':
    sys.exit(_run())
