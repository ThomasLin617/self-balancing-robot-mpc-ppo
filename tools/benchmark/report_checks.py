import json
import unittest

import numpy as np

from .report import plain_scalars, trial_label
from .audit import bearing_bin


class ReportChecks(unittest.TestCase):
    def test_trial_labels_use_actual_goal(self):
        self.assertEqual(trial_label({'name':'step_0','goal':[-.2,0],'force':0}), '-0.20 m')
        self.assertEqual(trial_label({'name':'push','goal':[.1,0],'force':-50}), '+0.10 m + pulse')

    def test_bearing_bins_are_symmetric_at_boundaries(self):
        for sign in [-1,1]:
            for angle,name in [(0,'front'),(np.pi/4,'front'),(np.pi/4+1e-9,'front-side'),
                               (np.pi/2,'front-side'),(np.pi/2+1e-9,'rear-side'),
                               (3*np.pi/4,'rear-side'),(3*np.pi/4+1e-9,'rear'),(np.pi,'rear')]:
                self.assertEqual(bearing_bin(sign*angle),name)

    def test_numpy_scalars_are_json_values(self):
        row=plain_scalars({'success':np.bool_(True),'time':np.float64(1.5),'count':np.int64(2),'undefined':None})
        self.assertEqual(json.loads(json.dumps(row,allow_nan=False)),
                         {'success':True,'time':1.5,'count':2,'undefined':None})


if __name__=='__main__':unittest.main()
