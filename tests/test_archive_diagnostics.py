"""Maintenance regression tests; all temporary files stay under this project."""
import contextlib
import importlib.util
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import zipfile

PROJECT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('diagnostic_archive', PROJECT/'tools/archive-diagnostics.py')
archive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(archive)


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        parent = PROJECT/'build/maintenance-audit'
        parent.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=parent)
        self.root = Path(self.temp.name)
        self.patch = mock.patch.multiple(archive, ROOT=self.root,
            DIAGNOSTICS=self.root/'build/diagnostics', ARCHIVES=self.root/'build/research-archive')
        self.patch.start()
        self.stage = archive.DIAGNOSTICS/'sample'
        self.stage.mkdir(parents=True)
        self.original = b'time,x,y\n0,1,2\n' * 100
        (self.stage/'one.csv').write_bytes(self.original)
        (self.stage/'two.csv').write_bytes(self.original)
        (self.stage/'unique.csv').write_bytes(b'unique telemetry\n')
        (self.stage/'source.cpp').write_text('source must survive', encoding='utf-8')
        self.quiet = contextlib.redirect_stdout(io.StringIO())
        self.quiet.__enter__()

    def tearDown(self):
        self.quiet.__exit__(None, None, None)
        self.patch.stop()
        self.temp.cleanup()

    def test_compacted_round_trip_preserves_every_original(self):
        result = archive.pack('sample', True)
        self.assertEqual(result['files'], 3)
        self.assertFalse((self.stage/'one.csv').exists())
        self.assertTrue((self.stage/'source.cpp').exists())
        archive.compact('sample')
        path = archive.ARCHIVES/'sample-raw.zip'
        with zipfile.ZipFile(path) as stored:
            self.assertEqual(len(stored.namelist()), 3)  # two unique files + manifest
        self.assertEqual(len(archive.verify(path)), 3)
        archive.restore('sample')
        self.assertEqual((self.stage/'one.csv').read_bytes(), self.original)
        self.assertEqual((self.stage/'two.csv').read_bytes(), self.original)
        self.assertEqual((self.stage/'unique.csv').read_bytes(), b'unique telemetry\n')

    def test_failed_verification_never_prunes(self):
        with mock.patch.object(archive, 'verify', side_effect=ValueError('corrupt archive')):
            with self.assertRaises(ValueError):
                archive.pack('sample', True)
        self.assertEqual((self.stage/'one.csv').read_bytes(), self.original)
        self.assertTrue((self.stage/'two.csv').exists())

    def test_prune_rechecks_bytes_when_size_and_mtime_match(self):
        path = self.stage/'two.csv'
        verify = archive.verify

        def replace_after_verify(archive_path):
            records = verify(archive_path)
            before = path.stat()
            path.write_bytes(b'X' + self.original[1:])
            os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))
            return records

        with mock.patch.object(archive, 'verify', side_effect=replace_after_verify):
            with self.assertRaisesRegex(ValueError, 'Original changed'):
                archive.pack('sample', True)
        self.assertEqual(path.read_bytes(), b'X' + self.original[1:])
        self.assertEqual((self.stage/'one.csv').read_bytes(), self.original)

    def test_separate_part_and_selected_restore_preserve_certified_zip(self):
        archive.pack('sample', False)
        base = archive.archive_path('sample').read_bytes()
        with mock.patch.object(archive, 'CERTIFIED_STAGES', {'sample'}):
            with self.assertRaisesRegex(ValueError, 'accepted evidence'):
                archive.compact('sample')
            archive.pack('sample', False, 'later')
            archive.compact('sample', 'later')
        self.assertEqual(archive.archive_path('sample').read_bytes(), base)
        selected = self.root/'probe'
        archive.restore('sample', 'later', selected, {'sample/two.csv'})
        self.assertEqual((selected/'sample/two.csv').read_bytes(), self.original)
        self.assertFalse((selected/'sample/one.csv').exists())

    def test_restore_refuses_to_overwrite_changed_evidence(self):
        archive.pack('sample', True)
        (self.stage/'one.csv').write_bytes(b'new evidence')
        with self.assertRaises(FileExistsError):
            archive.restore('sample')
        self.assertEqual((self.stage/'one.csv').read_bytes(), b'new evidence')
        self.assertFalse((self.stage/'two.csv').exists())

    def test_paths_cannot_escape_diagnostics(self):
        with self.assertRaises(ValueError):
            archive.checked(archive.DIAGNOSTICS/'../../outside.csv', archive.DIAGNOSTICS)
        with self.assertRaises(ValueError):
            archive.candidates('../research-archive')
        with self.assertRaises(ValueError):
            archive.checked(archive.DIAGNOSTICS/'.. /outside.csv', archive.DIAGNOSTICS)
        with self.assertRaises(ValueError):
            archive.checked(archive.DIAGNOSTICS/'sample/one.csv:hidden', archive.DIAGNOSTICS)


if __name__ == '__main__':
    unittest.main()
