"""Transient checkpoint I/O recovery; no ROS/SDK or runtime deadline changes."""
import errno
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.compat import read_shared_text, replace_shared_file, write_shared_text, preserve_failure_checkpoint


class SharedIoRecoveryTests(unittest.TestCase):
    def test_native_first_failure_survives_shared_log_failure_and_is_not_overwritten(self):
        import json
        import io
        class FailedSharedLog(io.StringIO):
            def write(self, text): raise OSError(errno.EIO,'authored shared-log failure')
        record={'run_id':'a'*32,'stage':'startup','error':{'type':'OSError','message':'primary'}}
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)/'native'
            result=preserve_failure_checkpoint(record,directory,FailedSharedLog())
            self.assertTrue(result['saved'])
            self.assertIn('stream_error',result)
            path=Path(result['path']);self.assertEqual(json.loads(path.read_text(encoding='utf-8')),record)
            later=preserve_failure_checkpoint(dict(record,stage='cleanup'),directory,io.StringIO())
            self.assertTrue(later['existing_first_failure_retained'])
            self.assertEqual(json.loads(path.read_text(encoding='utf-8'))['stage'],'startup')

    def test_transient_write_read_and_replace_preserve_one_complete_checkpoint(self):
        with tempfile.TemporaryDirectory() as temporary:
            base=Path(temporary);part=base/'state.tmp';target=base/'state.json'
            target.write_text('old',encoding='utf-8')
            original_write=Path.write_text
            calls=0
            def write(path,*args,**kwargs):
                nonlocal calls
                calls+=1
                if calls==1: raise OSError(errno.EIO,'authored transient shared-write failure')
                return original_write(path,*args,**kwargs)
            with patch.object(Path,'write_text',write):write_shared_text(part,'new')
            self.assertEqual(target.read_text(encoding='utf-8'),'old')
            import os
            real_replace=os.replace
            with patch('factory_ops.compat.os.replace',side_effect=[OSError(errno.EIO,'transient replace'),None]) as replace:
                replace_shared_file(part,target)
                self.assertEqual(replace.call_count,2)
            real_replace(part,target)
            with patch.object(Path,'read_text',side_effect=[OSError(errno.EIO,'transient read'),'new']):
                self.assertEqual(read_shared_text(target),'new')
            self.assertEqual(target.read_text(encoding='utf-8'),'new')

    def test_permanent_and_persistent_errors_do_not_report_success_or_retry_forever(self):
        target=Path('unused-checkpoint.tmp')
        with patch.object(Path,'write_text',side_effect=OSError(errno.ENOSPC,'authored full storage')) as write:
            with self.assertRaises(OSError):write_shared_text(target,'data')
            self.assertEqual(write.call_count,1)
        with patch('factory_ops.compat.time.monotonic',side_effect=[0.,.3]),patch.object(Path,'write_text',side_effect=OSError(errno.EIO,'persistent')) as write:
            with self.assertRaises(OSError):write_shared_text(target,'data')
            self.assertEqual(write.call_count,1)
        with patch('factory_ops.compat.time.monotonic',side_effect=[0.,.3]),patch.object(Path,'read_text',side_effect=OSError(errno.EIO,'persistent')):
            with self.assertRaises(OSError):read_shared_text(target)


if __name__=='__main__':unittest.main()
