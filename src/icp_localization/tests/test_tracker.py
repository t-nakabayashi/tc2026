import math
import numpy as np
import pytest
from icp_localization.tracker import Tracker


GOOD = dict(converged=True, overlap=.9, median_m=.1, inliers=1000,
            information_ratio=.1, correction_m=.1, rotation_deg=.1)


def initialized():
    t = Tracker()
    t.seed(np.eye(4), np.eye(4))
    assert t.result(0., 0., 1., t.generation, np.eye(4), np.eye(4), GOOD)
    return t


@pytest.mark.parametrize('metric,value', [('converged', False), ('overlap', .1),
    ('median_m', 1.), ('information_ratio', 0.), ('inliers', 20),
    ('correction_m', 1.), ('rotation_deg', 20.), ('overlap', math.nan)])
def test_rejected_icp_keeps_relative_lio_prediction(metric, value):
    t = initialized()
    raw = np.eye(4); raw[0, 3] = .2
    bad = raw.copy(); bad[0, 3] = 100.
    assert not t.result(.1, .1, 1.1, t.generation, bad, raw, dict(GOOD, **{metric: value}))
    assert t.localized and t.state(.1) == 'LIO_PREDICTION'
    assert np.allclose(t.predict(raw), raw)
    assert t.initialized_once


def test_next_good_icp_is_used_without_fix_or_control_state():
    t = initialized()
    t.result(.1, .1, 1.1, t.generation, np.eye(4), np.eye(4), dict(GOOD, overlap=0))
    raw = np.eye(4); raw[0, 3] = .2
    assert t.result(.2, .2, 1.2, t.generation, raw, raw, GOOD)
    assert t.state(.2) == 'ICP_TRACKING'


def test_initialization_requires_actual_good_map_match():
    t = Tracker()
    t.seed(np.eye(4), np.eye(4))
    t.result(0., 0., 1., t.generation, np.eye(4), np.eye(4), dict(GOOD, overlap=0))
    assert not t.localized and not t.initialized_once
    assert t.state(0.) == 'INITIALIZING'


def test_old_results_and_reset_generation_cannot_change_pose():
    t = initialized()
    bad = np.eye(4); bad[0, 3] = 10.
    assert not t.result(1., .1, 1.1, t.generation, bad, np.eye(4), GOOD)
    assert np.allclose(t.pose, np.eye(4))
    old = t.generation
    t.clear('lio_discontinuity_set_initialpose')
    assert t.initialized_once and not t.localized
    t.seed(np.eye(4), np.eye(4))
    assert not t.result(2., 2., 2., old, bad, np.eye(4), GOOD)
    assert not t.localized


def test_no_scan_corrections_still_predicts_and_reports_age():
    t = initialized()
    raw = np.eye(4); raw[:2, 3] = [10., 2.]
    assert np.allclose(t.predict(raw), raw)
    assert t.state(10.) == 'LIO_PREDICTION'
