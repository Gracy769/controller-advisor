import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engine import Requirements, advise, parse_coefficients, validate_plant


def test_rejects_unstable_and_improper_plants():
    with pytest.raises(ValueError, match="stable"):
        validate_plant(parse_coefficients("1"), parse_coefficients("1,-1"))
    with pytest.raises(ValueError, match="strictly proper"):
        validate_plant(parse_coefficients("1,1"), parse_coefficients("1,2"))


def test_candidate_validation_respects_actuator_limit_and_scenario_count():
    req = Requirements(max_overshoot=20, max_settling=8, max_steady_error=0.05,
                       max_input=2, gain_uncertainty=0.2, horizon=25, samples=401)
    candidates = advise(parse_coefficients("1"), parse_coefficients("1,2,1"), req,
                        full_state_available=False, search_iterations=4)
    assert {c.name for c in candidates} == {"P", "PI", "PID"}
    for candidate in candidates:
        if candidate.predicted:
            assert set(candidate.scenarios) == {"Nominal constrained", "Low gain", "High gain"}
            for trace in candidate.scenarios.values():
                assert trace.peak_input <= req.max_input + 1e-9
                assert np.all(np.isfinite(trace.output))


def test_pole_placement_requires_state_availability():
    req = Requirements(max_settling=8, horizon=25, samples=401)
    n, d = parse_coefficients("1"), parse_coefficients("1,2,1")
    no_state = advise(n, d, req, False, 4)
    with_state = advise(n, d, req, True, 4)
    assert "Pole placement" not in {c.name for c in no_state}
    assert "Pole placement" in {c.name for c in with_state}
