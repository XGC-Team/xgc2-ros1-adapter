"""Actual file/hash boundary tests; no claim of a live ROS acquisition."""
import ast
import copy
import hashlib
import os
from pathlib import Path
import re
import stat
import tempfile
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/rosbag_recorder.py'

class RecorderPartsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        tree = ast.parse(SCRIPT.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'publish_parts')
        self.ns = dict(os=os, re=re, stat=stat, hashlib=hashlib, output_directory=str(self.root))
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(SCRIPT), 'exec'), self.ns)
        self.publish = self.ns['publish_parts']
    def document(self):
        entries=[]
        for name,data in [('record-part01.bag',b'closed one'),('record-part02.bag',b'closed two')]:
            (self.root/name).write_bytes(data)
            entries.append(dict(name=name,state='finalized',sizeBytes=len(data)))
        (self.root/'pending.bag.active').write_bytes(b'partial')
        entries.append(dict(name='pending.bag.active',state='partial',sizeBytes=7))
        return dict(targetId='accepted-agent',slotId='selected',archiveFinalizationState='finalized',status='completed',bagFiles=entries)
    def test_real_multiple_volumes_hash_once_and_warm_opens_zero(self):
        doc=self.document();real_open=os.open;opened=[]
        def count(path,*args): opened.append(path);return real_open(path,*args)
        with patch.object(os,'open',count):
            self.publish(doc);self.publish(doc)
        self.assertEqual(len(opened),2)
        self.assertEqual(len(doc['parts']),2)
        self.assertEqual(doc['parts'][0]['sha256'],hashlib.sha256(b'closed one').hexdigest())
        self.assertEqual(doc['parts'][1]['sourcePath'],str(self.root/'record-part02.bag'))
        self.assertEqual(doc['parts'][0]['targetId'],'accepted-agent')
    def test_changed_same_size_and_restored_mtime_is_rejected(self):
        doc=self.document();self.publish(doc);p=self.root/'record-part01.bag';old=p.stat()
        p.write_bytes(b'changed!!!');os.utime(p,ns=(old.st_atime_ns,old.st_mtime_ns))
        with self.assertRaisesRegex(RuntimeError,'changed'):
            self.publish(doc)
    def test_partial_failed_and_shared_scope_are_not_fabricated(self):
        doc=self.document();doc.pop('slotId');doc['slotIds']=['one','two'];self.publish(doc)
        self.assertEqual({part['slotId'] for part in doc['parts']},{''})
        doc['status']='failed'
        with patch.object(os,'open',side_effect=AssertionError('failed capture payload read')): self.publish(doc)
        self.assertEqual(doc['parts'],[])
    def test_symlink_replacement_is_rejected(self):
        doc=self.document();p=self.root/'record-part01.bag';p.unlink();p.symlink_to(self.root/'record-part02.bag')
        with self.assertRaisesRegex(RuntimeError,'identity changed'): self.publish(doc)

if __name__=='__main__': unittest.main()
