"""Fail-closed CTest output redirection and actual Windows filesystem write auditing.

The watcher observes the workspace, including paths compiled into executables. A
metadata inventory also covers historical diagnostics and default verification
directories. Neither mechanism relies on CMake text alone. Only the caller's new
output root is writable; unrelated concurrent writes invalidate the observation.
"""
from pathlib import Path
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
import re
import struct
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


def within(path, parent):
    def canonical(value):
        # Notifications can name files already deleted/renamed by CTest. Resolving
        # those through the filesystem races deletion and can yield a different
        # Windows drive form. Roots are resolved once by the caller; compare event
        # names lexically, independent of whether the notified file still exists.
        text = os.path.abspath(os.fspath(value))
        if text.startswith('\\\\?\\UNC\\'):
            text = '\\\\'+text[8:]
        elif text.startswith('\\\\?\\'):
            text = text[4:]
        return os.path.normcase(text)
    child, base = canonical(path), canonical(parent)
    try:
        return os.path.commonpath((child, base)) == base
    except ValueError:
        return False


def prepare(build, destination, root=ROOT, source_text=None):
    """Rewrite original text once, then validate every absolute test argument."""
    build, destination, root = Path(build), Path(destination), Path(root)
    source = source_text if source_text is not None else (build/'tests/CTestTestfile.cmake').read_text(encoding='utf-8')
    dest = destination.resolve().as_posix()
    prefixes = {'historical': root.resolve().as_posix()+'/build/diagnostics/',
                'verification': build.resolve().as_posix()+'/verification'}
    counts = dict.fromkeys(prefixes, 0)
    pattern = re.compile('|'.join('(?P<'+name+'>'+re.escape(prefix)+')' for name, prefix in prefixes.items()), re.IGNORECASE)
    def replace(match):
        name = match.lastgroup
        counts[name] += 1
        return dest+('/artifacts/' if name == 'historical' else '/verification')
    redirected = pattern.sub(replace, source)
    if not sum(counts.values()):
        raise RuntimeError(f'No output paths redirected in {build}')
    arguments = 0
    for line in redirected.splitlines():
        if line.startswith('subdirs('):
            raise RuntimeError('Nested CTest directories require explicit isolated preparation')
        if not line.startswith('add_test('):
            continue
        for token in re.findall(r'"([^"\n]*)"', line):
            if not re.match(r'^[a-zA-Z]:[/\\]', token):
                continue
            if token.lower().endswith('.exe'):
                continue
            arguments += 1
            if not within(token.replace('\\', '/'), destination):
                raise RuntimeError(f'Unisolated absolute test argument: {token}')
    if not arguments:
        raise RuntimeError('No isolated output arguments found')
    remainder = re.sub(re.escape(dest), '', redirected, flags=re.IGNORECASE)
    if any(re.search(re.escape(prefix), remainder, re.IGNORECASE) for prefix in prefixes.values()):
        raise RuntimeError('Historical/default diagnostic path remains after redirection')
    destination.mkdir(parents=True, exist_ok=True)
    (destination/'CTestTestfile.cmake').write_text(redirected, encoding='utf-8')
    return dict(build=str(build), destination=dest, redirected_historical=counts['historical'],
                redirected_verification=counts['verification'], validated_output_arguments=arguments, leftovers=0)


def ctest_executable(build):
    cache = (Path(build)/'CMakeCache.txt').read_text(encoding='utf-8')
    match = re.search(r'^CMAKE_CTEST_COMMAND:INTERNAL=(.*)$', cache, re.MULTILINE)
    if not match or not Path(match.group(1).strip()).is_file():
        raise RuntimeError(f'No absolute CTest executable in {build}')
    return match.group(1).strip()


def inventory(root, allowed):
    roots = [root/'build/diagnostics', *(root/'build'/v/'verification' for v in ('vs-release', 'cpu-release', 'cpu-no-physics'))]
    files = {}
    for directory in roots:
        if not directory.exists():
            continue
        for current, dirs, names in os.walk(directory):
            dirs[:] = [d for d in dirs if not within(Path(current)/d, allowed)]
            if within(current, allowed):
                continue
            for name in names:
                path = Path(current)/name
                st = path.stat()
                files[path.relative_to(root).as_posix()] = [st.st_size, st.st_mtime_ns, st.st_ctime_ns]
    return files


class OutputGuard:
    def __init__(self, allowed, report, root=ROOT):
        self.root, self.allowed, self.report = Path(root).resolve(), Path(allowed).resolve(), Path(report)
        if not within(self.allowed, self.root) or self.allowed == self.root or not within(self.report, self.allowed):
            raise ValueError('A strict workspace child output and an in-output report are required')
        self.events, self.errors, self.event_count = [], [], 0
        self.recent = []
        self.stop = threading.Event()
        self.ready = threading.Event()

    @property
    def violated(self):
        return bool(self.events or self.errors)

    def start(self):
        if os.name != 'nt':
            raise RuntimeError('Actual write monitoring requires Windows; refusing an unaudited run')
        self.allowed.mkdir(parents=True, exist_ok=True)
        self.before = inventory(self.root, self.allowed)
        self.started_ns = time.time_ns()
        self.thread = threading.Thread(target=self._watch, daemon=True)
        self.thread.start()
        if not self.ready.wait(10) or self.errors:
            raise RuntimeError(f'Filesystem watcher could not start: {self.errors}')
        return self

    def _watch(self):
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        class Overlapped(ctypes.Structure):
            _fields_ = [('Internal', ctypes.c_size_t), ('InternalHigh', ctypes.c_size_t),
                        ('Offset', wintypes.DWORD), ('OffsetHigh', wintypes.DWORD), ('hEvent', wintypes.HANDLE)]
        kernel.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
        kernel.CreateFileW.restype = wintypes.HANDLE
        kernel.CreateEventW.argtypes = [ctypes.c_void_p, wintypes.BOOL, wintypes.BOOL, wintypes.LPCWSTR]
        kernel.CreateEventW.restype = wintypes.HANDLE
        kernel.ReadDirectoryChangesW.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD, wintypes.BOOL, wintypes.DWORD,
                                                ctypes.POINTER(wintypes.DWORD), ctypes.POINTER(Overlapped), ctypes.c_void_p]
        kernel.GetOverlappedResult.argtypes = [wintypes.HANDLE, ctypes.POINTER(Overlapped), ctypes.POINTER(wintypes.DWORD), wintypes.BOOL]
        kernel.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        kernel.ResetEvent.argtypes = [wintypes.HANDLE]
        kernel.CancelIoEx.argtypes = [wintypes.HANDLE, ctypes.POINTER(Overlapped)]
        kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        handle = kernel.CreateFileW(str(self.root), 1, 7, None, 3, 0x42000000, None)
        event = kernel.CreateEventW(None, True, False, None)
        if handle == ctypes.c_void_p(-1).value or not event:
            self.errors.append(f'Watcher handle failed: {ctypes.get_last_error()}')
            self.ready.set()
            return
        buffer = ctypes.create_string_buffer(65536)
        pending = False
        try:
            while not self.stop.is_set():
                overlap = Overlapped(); overlap.hEvent = event
                kernel.ResetEvent(event)
                if not kernel.ReadDirectoryChangesW(handle, buffer, len(buffer), True, 1|2|4|8|16|64, None, ctypes.byref(overlap), None):
                    if ctypes.get_last_error() != 997:
                        raise ctypes.WinError(ctypes.get_last_error())
                pending = True
                self.ready.set()
                while kernel.WaitForSingleObject(event, 50) == 258:
                    if self.stop.is_set():
                        break
                if self.stop.is_set():
                    break
                size = wintypes.DWORD()
                if not kernel.GetOverlappedResult(handle, ctypes.byref(overlap), ctypes.byref(size), False):
                    raise ctypes.WinError(ctypes.get_last_error())
                pending = False
                if size.value == 0:
                    raise RuntimeError('ReadDirectoryChangesW buffer overflow; observation incomplete')
                offset = 0
                while offset < size.value:
                    next_offset, action, length = struct.unpack_from('<III', buffer.raw, offset)
                    name = buffer.raw[offset+12:offset+12+length].decode('utf-16-le')
                    path = self.root/name
                    self.event_count += 1
                    self.recent.append(dict(name=name, action=action))
                    self.recent = self.recent[-24:]
                    if not within(path, self.allowed):
                        self.events.append(dict(path=name.replace('\\', '/'), action=action, observed_ns=time.time_ns()))
                    if not next_offset:
                        break
                    offset += next_offset
        except Exception as error:
            self.errors.append(repr(error))
        finally:
            self.ready.set()
            if pending:
                kernel.CancelIoEx(handle, ctypes.byref(overlap))
                size = wintypes.DWORD()
                kernel.GetOverlappedResult(handle, ctypes.byref(overlap), ctypes.byref(size), True)
            kernel.CloseHandle(event)
            kernel.CloseHandle(handle)

    def finish(self):
        # Let queued filesystem notifications drain before closing the observation.
        time.sleep(.15)
        after = inventory(self.root, self.allowed)
        self.stop.set()
        self.thread.join(5)
        if self.thread.is_alive():
            self.errors.append('Watcher did not stop cleanly')
        changed = sorted(p for p in self.before.keys() | after.keys() if self.before.get(p) != after.get(p))
        changes = []
        for path in changed:
            full = self.root/path
            digest = None
            if full.is_file():
                h = hashlib.sha256()
                with full.open('rb') as stream:
                    for block in iter(lambda: stream.read(1024*1024), b''):
                        h.update(block)
                digest = h.hexdigest()
            changes.append(dict(path=path, before=self.before.get(path), after=after.get(path), sha256_after=digest))
        result = dict(schema='diagnostic-output-isolation-v1', root=str(self.root), allowed_output=str(self.allowed),
                      started_ns=self.started_ns, finished_ns=time.time_ns(), watched='ReadDirectoryChangesW recursive workspace; names, sizes, creation, attributes, last write',
                      protected_inventory_files=len(self.before), observed_events=self.event_count,
                      outside_events=self.events, protected_inventory_changes=changes, monitoring_errors=self.errors,
                      last_events=self.recent,
                      attribution='Any concurrent outside write invalidates this run; the watcher alone does not identify its process',
                      clean=not (self.violated or changes))
        self.report.parent.mkdir(parents=True, exist_ok=True)
        self.report.write_text(json.dumps(result, indent=2), encoding='utf-8')
        return result


def tracked_popen(command, **kwargs):
    """Own a Windows job so scoped termination needs no process-list permission."""
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    class BasicLimits(ctypes.Structure):
        _fields_ = [('ProcessTime', ctypes.c_longlong), ('JobTime', ctypes.c_longlong), ('Flags', wintypes.DWORD),
                    ('MinWorkingSet', ctypes.c_size_t), ('MaxWorkingSet', ctypes.c_size_t), ('ActiveProcesses', wintypes.DWORD),
                    ('Affinity', ctypes.c_size_t), ('Priority', wintypes.DWORD), ('Scheduling', wintypes.DWORD)]
    class ExtendedLimits(ctypes.Structure):
        _fields_ = [('Basic', BasicLimits), ('Io', ctypes.c_ulonglong*6), ('ProcessMemory', ctypes.c_size_t),
                    ('JobMemory', ctypes.c_size_t), ('PeakProcessMemory', ctypes.c_size_t), ('PeakJobMemory', ctypes.c_size_t)]
    kernel.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
    kernel.CreateJobObjectW.restype = wintypes.HANDLE
    kernel.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
    kernel.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    job = kernel.CreateJobObjectW(None, None)
    limits = ExtendedLimits(); limits.Basic.Flags = 0x2000  # KILL_ON_JOB_CLOSE
    if not job or not kernel.SetInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        process = subprocess.Popen(command, **kwargs)
    except BaseException:
        kernel.CloseHandle(job)
        raise
    if not kernel.AssignProcessToJobObject(job, wintypes.HANDLE(int(process._handle))):
        error = ctypes.get_last_error()
        process.terminate(); process.wait()
        kernel.CloseHandle(job)
        raise ctypes.WinError(error)
    process._diagnostic_job = job
    return process


def close_process_job(process):
    """Close completed jobs before ending the write observation (kills own orphans)."""
    assert process.poll() is not None, 'Do not close a running process job'
    job = getattr(process, '_diagnostic_job', None)
    if job:
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        if not kernel.CloseHandle(job):
            raise ctypes.WinError(ctypes.get_last_error())
        process._diagnostic_job = None


def stop_children(processes):
    """Stop only process trees started by this caller, when a run is invalidated."""
    stopped = []
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
    for process in processes:
        if process.poll() is None:
            job = getattr(process, '_diagnostic_job', None)
            if job:
                ok = kernel.TerminateJobObject(job, 1)
                if not ok:
                    raise ctypes.WinError(ctypes.get_last_error())
            else:
                process.terminate()
            code = process.wait(timeout=5)
            close_process_job(process)
            stopped.append(dict(pid=process.pid, method='owned Windows job' if job else 'original process handle', returncode=code, stopped=True))
    return stopped
