import io
import json
import os
import tarfile
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from robotx_graey_2026.api.recording_paths import session_record_path
import importlib.util
import test_unified_missions as fixtures
spec=importlib.util.spec_from_file_location('native_sonar_recorder',Path(__file__).parents[1]/'robotx_graey_2026/api/sonar/recorder.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
SonarRecorder=module.Recorder


class NativeSessionPaths(unittest.TestCase):
    def test_session_path_rejects_escape_and_foreign_owner(self):
        with tempfile.TemporaryDirectory() as folder:
            for name,owner in [('../escape','graey-'+'a'*16),('session','../../owner')]:
                with self.assertRaises(ValueError):session_record_path(folder,name,'camera',owner)

    def test_native_sonar_writes_into_shared_session_folder(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'GRAEY_RECORD_DIR':folder}):
            recorder=SonarRecorder(str(Path(folder)/'legacy'))
            expected=session_record_path(folder,'session','sonar','graey-'+'a'*16)
            actual=recorder.start('GRAEY_SESSION:session:graey-'+'a'*16,{})
            self.assertEqual(Path(actual),expected)
            recorder.stop()
            self.assertTrue((expected/'meta.json').is_file())


class FullSessionArchive(unittest.TestCase):
    setUp=fixtures.TestRecordingService.setUp
    request=fixtures.TestRecordingService.request
    def test_download_refuses_missing_native_data_then_includes_both_sources(self):
        self.service.start(dict(self.command,camera=True,sonar=True,sensors=False));self.service.sample();self.service.stop()
        session=self.service.session;name=session['name'];owner=session['owner_token']
        with self.assertRaisesRegex(ValueError,'partial data'):self.service.archive_into(name,io.BytesIO())
        camera=session_record_path(self.folder.name,name,'camera',owner);camera.parent.mkdir(parents=True,exist_ok=True);camera.write_bytes(b'synthetic camera stream')
        sonar=session_record_path(self.folder.name,name,'sonar',owner);sonar.mkdir(parents=True,exist_ok=True);(sonar/'sweep_00001.npz').write_bytes(b'synthetic sonar sweep')
        output=io.BytesIO();self.assertTrue(self.service.archive_into(name,output));output.seek(0)
        with tarfile.open(fileobj=output,mode='r:gz') as archive:
            files=archive.getnames()
            self.assertIn(name+'/camera/'+owner+'.mjpg',files)
            self.assertIn(name+'/sonar/'+owner+'/sweep_00001.npz',files)
    def test_recording_refuses_nonpersistent_mount(self):
        self.service.persistence=lambda _:('/','overlay',False)
        with self.assertRaisesRegex(ValueError,'persistent bind mount'):self.service.start(self.command)
