import ast
import hashlib
import json
from pathlib import Path
import unittest


class MiramarCacheTests(unittest.TestCase):
    def test_every_saved_tile_is_allowlisted_and_hash_matches(self):
        root=Path(__file__).parents[1]
        tree=ast.parse((root/'robotx_graey_2026/api/gui/gui_node.py').read_text())
        statements=[n for n in tree.body if (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('FILES','TYPES') for t in n.targets))
                    or (isinstance(n,ast.For) and isinstance(n.target,ast.Tuple) and any(isinstance(t,ast.Name) and t.id=='_z' for t in n.target.elts))]
        ns={};exec(compile(ast.Module(body=statements,type_ignores=[]),'routes','exec'),ns)
        folder=root/'tools/vendor/miramar'
        manifest=json.loads((folder/'manifest.json').read_text())
        self.assertEqual(len(manifest['tiles']),58)
        for tile in manifest['tiles']:
            relative='vendor/miramar/'+tile['file']
            self.assertEqual(ns['FILES']['/'+relative],relative)
            data=(folder/tile['file']).read_bytes()
            self.assertTrue(data.startswith(b'\xff\xd8'))
            self.assertEqual(hashlib.sha256(data).hexdigest(),tile['sha256'])
        self.assertEqual(ns['TYPES']['.jpg'],'image/jpeg')
