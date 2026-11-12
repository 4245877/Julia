"""Guarded OS keyboard-delivery probe for the stage7k desktop (post-freeze, after performance).

Starts the frozen GLFW desktop visibly in wall-clock mode (coord-shift, fixed tick limit),
finds its top-level window by process id and sends real OS key events with SendInput
ONLY while GetForegroundWindow() is that window; otherwise no key is sent and the
attempt is recorded. Expected effects are read back from coordinated-v1.csv: the
scripted A command gives serial 1 at tick 480; OS keys C (cancel) and O (return) must
add accepted serials with the cancel/return goals. Any other outcome is UNVERIFIED.
"""
from pathlib import Path
import argparse
import csv
import ctypes
import ctypes.wintypes as wt
import datetime
import json
import subprocess
import time

user32 = ctypes.WinDLL('user32', use_last_error=True)
INPUT_KEYBOARD, KEYEVENTF_KEYUP = 1, 2


class KEYBDINPUT(ctypes.Structure):
    _fields_ = [('wVk', wt.WORD), ('wScan', wt.WORD), ('dwFlags', wt.DWORD), ('time', wt.DWORD), ('dwExtraInfo', ctypes.c_size_t)]


class INPUT(ctypes.Structure):
    class _U(ctypes.Union):
        _fields_ = [('ki', KEYBDINPUT), ('padding', ctypes.c_byte*32)]
    _anonymous_ = ('u',)
    _fields_ = [('type', wt.DWORD), ('u', _U)]


def windows_of(pid):
    found = []
    @ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
    def callback(hwnd, _):
        owner = wt.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(owner))
        if owner.value == pid and user32.IsWindowVisible(hwnd):
            found.append(hwnd)
        return True
    user32.EnumWindows(callback, 0)
    return found


def foreground_pid():
    hwnd = user32.GetForegroundWindow()
    owner = wt.DWORD()
    user32.GetWindowThreadProcessId(hwnd, ctypes.byref(owner))
    return hwnd, owner.value


def press(vk):
    events = (INPUT*2)()
    events[0].type = events[1].type = INPUT_KEYBOARD
    events[0].ki = KEYBDINPUT(vk, 0, 0, 0, 0)
    events[1].ki = KEYBDINPUT(vk, 0, KEYEVENTF_KEYUP, 0, 0)
    return user32.SendInput(2, events, ctypes.sizeof(INPUT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    exe = Path('build/vs-release/apps/julia_body_physics/julia_body_physics_desktop.exe').resolve()
    raw = args.output/'raw'
    command = [str(exe), '--case', 'coord-shift', '--bounded-solver', 'optimized', '--ticks', '2280', '--output', str(raw)]
    record = dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), command=command, events=[])
    log = (args.output/'process.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    start = time.perf_counter(); hwnd = None
    while time.perf_counter()-start < 60 and process.poll() is None and hwnd is None:
        found = windows_of(process.pid)
        hwnd = found[0] if found else None
        time.sleep(.2)
    record['window_found'] = hwnd is not None
    if hwnd is not None:
        visible = time.perf_counter()
        record['window_seconds_after_start'] = visible-start
        for delay, key, name in ((9., 0x43, 'C cancel'), (14., 0x4F, 'O return')):
            while time.perf_counter()-visible < delay and process.poll() is None:
                time.sleep(.02)
            user32.ShowWindow(hwnd, 9); user32.SetForegroundWindow(hwnd)
            time.sleep(.3)
            current, owner = foreground_pid()
            event = dict(key=name, wall_after_window_s=time.perf_counter()-visible, foreground_is_julia=bool(current == hwnd and owner == process.pid))
            if event['foreground_is_julia']:
                event['sendinput_events'] = press(key)
            else:
                event['sent'] = False
            record['events'].append(event)
    process.wait(timeout=300); log.close()
    record['returncode'] = process.returncode
    serials = []
    if (raw/'coordinated-v1.csv').exists():
        with (raw/'coordinated-v1.csv').open(newline='') as stream:
            previous = 0
            for row in csv.DictReader(stream):
                serial = int(row['serial'])
                if serial != previous:
                    serials.append(dict(tick=int(row['tick']), serial=serial, cancelled=row['cancelled'] == '1',
                                        goal_offset_mm=[1000*float(row['goal_offset_x']), 1000*float(row['goal_offset_z'])],
                                        alpha_goal=float(row['alpha_goal']), alpha0=float(row['alpha0'])))
                    previous = serial
    record['serial_changes'] = serials
    sent = [e for e in record['events'] if e.get('sendinput_events') == 2]
    os_serials = [s for s in serials if s['serial'] >= 2]
    record['verified'] = bool(record['returncode'] == 0 and len(sent) == 2 and len(os_serials) == 2 and os_serials[0]['cancelled']
                              and not os_serials[1]['cancelled'] and os_serials[1]['goal_offset_mm'] == [0., 0.] and os_serials[1]['alpha_goal'] == os_serials[1]['alpha0'])
    record['status'] = 'OS keyboard delivery VERIFIED for C/O in coordinated mode' if record['verified'] else 'UNVERIFIED (see events and serial changes)'
    (args.output/'certificate.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
