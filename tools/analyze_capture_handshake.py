#!/usr/bin/env python3
"""Measure the capture handshake race directly, from an archived control log.

The symptom -- a full-authority stop at the end of a pass -- is a race between
a service round trip and one control tick, so counting stops tells you as much
about the day's scheduling as about the code. These two quantities do not:

  * how long each handshake stayed pending, in control ticks;
  * whether the swerve controller latched Brake inside a PASS while it did.

Both are already archived per tick by execution_node: `capture_pending` is the
future being outstanding, `motion_state`/`motion_reason` are what the swerve
controller published, and `kind` says which step was running. Nothing has to be
added to the control path to read them, so a run recorded before this tool
existed can still be measured with it.

A handshake occupying a single tick cannot halt anything: the executor sets the
future after it has already issued that tick's command, and the next tick polls
it done. So `ticks - 1` is the number of ticks the executor spent in the halt
branch, and `ticks > 1` is the condition under which the close direction used
to command zero speed mid-pass.
"""
import argparse
import json
from pathlib import Path

BRAKE_REASON = 'STOP_REQUEST'


def run_name(log):
    """The run directory: <run>/session/tasks/<id>/mission/execution.jsonl."""
    for parent in log.parents:
        if parent.name == 'session':
            return parent.parent.name+'/'+log.parents[1].name
    return str(log.parent)


def read(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def handshakes(rows):
    """Every span of consecutive ticks with the capture future outstanding."""
    spans, start = [], None
    for i, row in enumerate(rows):
        if row.get('capture_pending'):
            if start is None:
                start = i
        elif start is not None:
            spans.append((start, i-1));start = None
    if start is not None:
        spans.append((start, len(rows)-1))

    out = []
    for a, b in spans:
        opened = rows[a]
        after = rows[b+1] if b+1 < len(rows) else {}
        # The transition the acknowledgement produced names the direction; the
        # first pending tick still carries the previous value.
        before_active, after_active = opened.get('capture_active'), after.get('capture_active')
        direction = ('open' if after_active is True else
                     'close' if before_active is True and after_active is False else
                     'initial' if before_active is None else 'unchanged')
        window = rows[a:b+2]
        latched = [r for r in window
                   if r.get('kind') == 'PASS' and r.get('motion_state') == 'BRAKE'
                   and r.get('motion_reason') == BRAKE_REASON]
        out.append(dict(
            direction=direction, ticks=b-a+1, halted_ticks=b-a,
            sim_time_s=round(opened['time_s'], 3), kind=opened.get('kind'),
            track_id=opened.get('track_id'), motion_state=opened.get('motion_state'),
            zero_commands=sum(1 for r in rows[a:b+1] if r.get('command_body') == [0, 0, 0]),
            pass_brake_latched=bool(latched)))
    return out


def summarize(path):
    rows = read(path)
    events = handshakes(rows)
    closes = [e for e in events if e['direction'] == 'close']
    opens = [e for e in events if e['direction'] == 'open']
    over = [e for e in closes if e['ticks'] > 1]
    # Latches anywhere in a PASS, whether or not a handshake was outstanding, so
    # a stop this tool failed to attribute still shows up in the totals.
    pass_brake = sum(1 for r in rows
                     if r.get('kind') == 'PASS' and r.get('motion_state') == 'BRAKE'
                     and r.get('motion_reason') == BRAKE_REASON)

    def spread(group):
        return dict(count=len(group),
                    ticks=sorted(e['ticks'] for e in group),
                    over_one_tick=sum(1 for e in group if e['ticks'] > 1),
                    brake_latched=sum(1 for e in group if e['pass_brake_latched']))

    return dict(log=str(path), ticks=len(rows),
                opens=spread(opens), closes=spread(closes),
                closes_over_one_tick_that_latched_brake=sum(1 for e in over if e['pass_brake_latched']),
                pass_brake_ticks_total=pass_brake, events=events)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('logs', type=Path, nargs='+', help='mission/execution.jsonl files')
    p.add_argument('--label', nargs='*', default=[])
    p.add_argument('--output', type=Path)
    p.add_argument('--expect-no-close-latch', action='store_true',
                   help='exit non-zero if any close handshake latched Brake inside a PASS')
    a = p.parse_args()
    runs = []
    for i, log in enumerate(a.logs):
        entry = summarize(log)
        entry['label'] = a.label[i] if i < len(a.label) else run_name(log)
        runs.append(entry)
    report = dict(schema='agv.capture_handshake.v1', runs=runs, totals=dict(
        closes=sum(r['closes']['count'] for r in runs),
        closes_over_one_tick=sum(r['closes']['over_one_tick'] for r in runs),
        closes_over_one_tick_that_latched_brake=sum(
            r['closes_over_one_tick_that_latched_brake'] for r in runs),
        opens=sum(r['opens']['count'] for r in runs),
        opens_over_one_tick=sum(r['opens']['over_one_tick'] for r in runs),
        opens_that_latched_brake=sum(r['opens']['brake_latched'] for r in runs)))
    text = json.dumps(report, indent=2)+'\n'
    if a.output:
        a.output.write_text(text)
    print(json.dumps({k: v for k, v in report.items() if k != 'runs'}, indent=2))
    for r in runs:
        print('%-42s closes=%d over1=%d latched=%d opens=%d over1=%d latched=%d'
              % (r['label'], r['closes']['count'], r['closes']['over_one_tick'],
                 r['closes_over_one_tick_that_latched_brake'],
                 r['opens']['count'], r['opens']['over_one_tick'], r['opens']['brake_latched']))
    if a.expect_no_close_latch:
        latched = report['totals']['closes_over_one_tick_that_latched_brake']
        over = report['totals']['closes_over_one_tick']
        # A run in which no close ever outlasted a tick proves nothing either
        # way: the condition the fix addresses never arose.
        if not over:
            raise SystemExit('no close handshake outlasted one control tick; '
                             'this sample cannot distinguish the fix from luck')
        if latched:
            raise SystemExit('%d of %d close handshakes still latched Brake' % (latched, over))
        print('%d close handshakes outlasted one tick; none latched Brake' % over)


if __name__ == '__main__':
    main()
