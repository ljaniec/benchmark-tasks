"""Reject authored intersections that can release the strap during warm-up."""
import unittest
import os
from pathlib import Path

try:
    import numpy as np
    from pxr import Usd, UsdGeom
    AVAILABLE = Path(os.environ.get('TASK_SCENE_USD','')).is_file()
except ImportError:
    AVAILABLE = False


@unittest.skipUnless(AVAILABLE, 'Set TASK_SCENE_USD to the exported workbench.usda')
class GeometryTests(unittest.TestCase):
    def test_strap_contacts_surface_without_crossing_fixture(self):
        stage = Usd.Stage.Open(os.environ['TASK_SCENE_USD'])
        links = sorted(str(p.GetPath()) for p in stage.Traverse() if p.GetName().startswith('strap_') and p.IsA(UsdGeom.Xform))
        bonds = [p for p in stage.GetPrimAtPath('/World/joints').GetChildren() if p.GetName().startswith('overlap_')]
        self.assertEqual(len(bonds), 4)
        bbox = UsdGeom.BBoxCache(0, ['default','render'])
        def bounds(path):
            box = bbox.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
            return np.asarray(box.GetMin()), np.asarray(box.GetMax())
        solid_paths = ['/World/battery', '/World/module/shape', '/World/module/backplate',
                       '/World/module/shelf', '/World/module/side_mount']
        # The first segment is intentionally fixed into the permanent anchor.
        for link in links[1:]:
            lo, hi = bounds(link)
            for solid in solid_paths:
                other_lo, other_hi = bounds(solid)
                overlap = np.minimum(hi,other_hi)-np.maximum(lo,other_lo)
                self.assertFalse(np.all(overlap>1e-6), f'{link} intersects {solid}: {overlap}')


if __name__ == '__main__':
    unittest.main()
