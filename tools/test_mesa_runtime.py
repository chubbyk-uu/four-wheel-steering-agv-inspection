"""Launcher regression tests; no GPU or private Mesa installation required."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('mesa_runtime', Path(__file__).with_name('with_mesa_runtime.py'))
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


class EnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.prefix = Path(self.directory.name)
        for name in ('lib/libgallium-25.2.8.so', 'lib/libgbm.so.1',
                     'lib/libGLX_mesa.so.0', 'lib/libEGL_mesa.so.0',
                     'share/glvnd/egl_vendor.d/50_mesa.json'):
            p = self.prefix / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.touch()
        for name in ('lib/dri', 'lib/gbm'):
            (self.prefix / name).mkdir()

    def test_round_trip_restores_inherited_paths_and_unset_values(self):
        original = {'LD_LIBRARY_PATH': '/opt/ros/lib:/opt/optix',
                    'LD_PRELOAD': '/opt/profiler.so', 'DISPLAY': ':0',
                    '__GLX_VENDOR_LIBRARY_NAME': 'original'}
        patched = runtime.environment('patched', self.prefix, original)
        self.assertEqual(runtime.environment('system', self.prefix, patched), original)
        self.assertEqual(original['LD_LIBRARY_PATH'], '/opt/ros/lib:/opt/optix')

    def test_repeated_selection_does_not_accumulate_overrides(self):
        first = runtime.environment('patched', self.prefix, {})
        self.assertEqual(runtime.environment('patched', self.prefix, first), first)

    def test_missing_build_fails_but_system_does_not_need_build(self):
        absent = self.prefix / 'absent'
        with self.assertRaisesRegex(ValueError, 'Missing private Mesa'):
            runtime.environment('patched', absent, {})
        self.assertEqual(runtime.environment('system', absent, {'DISPLAY': ':0'}), {'DISPLAY': ':0'})

    def test_ambiguous_preload_path_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'whitespace or colon'):
            runtime.environment('patched', self.prefix / 'space here', {})


if __name__ == '__main__':
    unittest.main()
