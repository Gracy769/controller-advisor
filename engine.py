"""Simulation-only controller advisor for stable continuous-time SISO plants."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import warnings

import numpy as np
from scipy import optimize, signal


@dataclass(frozen=True)
class Requirements:
    max_overshoot: float = 10.0
    max_settling: float = 5.0
    max_steady_error: float = 0.02
    max_input: float = 5.0
    gain_uncertainty: float = 0.2
    horizon: float = 25.0
    samples: int = 1001
    preference: str = "Balanced"


@dataclass
class Trace:
    time: np.ndarray
    output: np.ndarray
    input: np.ndarray
    overshoot: float
    settling: float
    steady_error: float
    peak_input: float
    input_energy: float


@dataclass
class Candidate:
    name: str
    parameters: dict
    predicted: Optional[Trace]
    scenarios: dict[str, Trace]
    feasible: bool
    failures: list[str]
    score: float
    note: str = ""


def parse_coefficients(value: str) -> np.ndarray:
    parts = [p.strip() for p in value.replace(";", ",").split(",")]
    if not parts or any(not p for p in parts):
        raise ValueError("Enter comma-separated coefficients, such as 1, 3, 3, 1.")
    result = np.asarray([float(p) for p in parts], dtype=float)
    if not np.all(np.isfinite(result)):
        raise ValueError("Coefficients must be finite numbers.")
    result = np.trim_zeros(result, "f")
    if result.size == 0:
        raise ValueError("The polynomial cannot be zero.")
    return result


def validate_plant(num: np.ndarray, den: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    num, den = np.asarray(num, float), np.asarray(den, float)
    if den.size < 2 or den.size - 1 > 8:
        raise ValueError("Use a plant of order 1 through 8.")
    if num.size >= den.size:
        raise ValueError("The plant must be strictly proper (numerator order below denominator order).")
    if abs(den[-1]) < 1e-12 or abs(num[-1]) < 1e-12:
        raise ValueError("This version requires finite, nonzero DC gain and no pole at the origin.")
    den, num = den / den[0], num / den[0]
    if np.any(np.real(np.roots(den)) >= -1e-8):
        raise ValueError("This version accepts stable open-loop plants only.")
    return num, den


def validate_requirements(req: Requirements) -> None:
    if not (0 <= req.max_overshoot <= 100):
        raise ValueError("Overshoot bound must be between 0 and 100 percent.")
    if req.max_settling <= 0 or req.max_steady_error < 0 or req.max_input <= 0:
        raise ValueError("Settling time and input limit must be positive; error cannot be negative.")
    if not (0 <= req.gain_uncertainty < 1):
        raise ValueError("Gain uncertainty must be between 0 and 100 percent.")
    if req.horizon <= req.max_settling or req.samples < 201:
        raise ValueError("Use a horizon longer than the settling-time limit and at least 201 samples.")


def _metrics(time: np.ndarray, y: np.ndarray, u: np.ndarray) -> Trace:
    if not np.all(np.isfinite(y)) or not np.all(np.isfinite(u)):
        raise ValueError("Nonfinite simulation response.")
    overshoot = float(max(0.0, (np.max(y) - 1.0) * 100.0))
    bad = np.flatnonzero(np.abs(y - 1.0) > 0.02)
    settling = float(time[bad[-1] + 1]) if bad.size and bad[-1] + 1 < len(time) else (float("inf") if bad.size else 0.0)
    steady_error = float(abs(1.0 - np.mean(y[-max(5, len(y) // 20):])))
    peak_input = float(np.max(np.abs(u)))
    input_energy = float(np.trapezoid(u * u, time))
    return Trace(time, y, u, overshoot, settling, steady_error, peak_input, input_energy)


def _controller_tf(name: str, values: np.ndarray, tau: float) -> tuple[np.ndarray, np.ndarray]:
    if name == "P":
        return np.array([values[0]]), np.array([1.0])
    if name == "PI":
        return np.array([values[0], values[1]]), np.array([1.0, 0.0])
    kp, ki, kd = values
    return np.array([kp * tau + kd, kp + ki * tau, ki]), np.array([tau, 1.0, 0.0])


def predict_output_feedback(num: np.ndarray, den: np.ndarray, name: str, values: np.ndarray,
                            time: np.ndarray, tau: float) -> Trace:
    cn, cd = _controller_tf(name, values, tau)
    numerator = np.polymul(num, cn)
    denominator = np.polyadd(np.polymul(den, cd), numerator)
    if np.any(np.real(np.roots(denominator)) >= -1e-7):
        raise ValueError("Unstable closed loop.")
    u_num = np.polymul(den, cn)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=signal.BadCoefficients)
        _, y = signal.step((numerator, denominator), T=time)
        _, u = signal.step((u_num, denominator), T=time)
    return _metrics(time, y, u)


def _effort_weight(preference: str) -> tuple[float, float, float]:
    return {
        "Balanced": (0.45, 0.35, 0.20),
        "Lowest input effort": (0.20, 0.70, 0.10),
        "Fastest response": (0.75, 0.15, 0.10),
        "Simplest controller": (0.20, 0.20, 0.60),
    }.get(preference, (0.45, 0.35, 0.20))


def _violations(trace: Trace, req: Requirements, scenario: str) -> list[str]:
    issues = []
    if trace.overshoot > req.max_overshoot + 1e-5:
        issues.append(f"{scenario}: overshoot")
    if trace.settling > req.max_settling + 1e-5:
        issues.append(f"{scenario}: settling time")
    if trace.steady_error > req.max_steady_error + 1e-5:
        issues.append(f"{scenario}: steady-state error")
    if trace.peak_input > req.max_input + 1e-5:
        issues.append(f"{scenario}: actuator input")
    return issues


def _penalty(trace: Trace, req: Requirements) -> float:
    terms = [
        max(0.0, trace.overshoot / max(req.max_overshoot, 0.1) - 1),
        max(0.0, trace.settling / req.max_settling - 1) if np.isfinite(trace.settling) else 5,
        max(0.0, trace.steady_error / max(req.max_steady_error, 0.001) - 1),
        max(0.0, trace.peak_input / req.max_input - 1),
    ]
    return float(sum(min(v, 10) ** 2 for v in terms))


def _score(trace: Trace, req: Requirements, complexity: float) -> float:
    wt, we, wc = _effort_weight(req.preference)
    t_part = min(trace.settling / req.max_settling, 10) if np.isfinite(trace.settling) else 10
    e_part = trace.input_energy / (req.horizon * req.max_input ** 2)
    return float(wt * t_part + we * e_part + wc * complexity)


def simulate_feedback(num: np.ndarray, den: np.ndarray, name: str, values: np.ndarray,
                      req: Requirements, gain_factor: float, tau: float) -> Trace:
    a, b, c, d = signal.tf2ss(num, den)
    if np.max(np.abs(d)) > 1e-10:
        raise ValueError("Direct feedthrough is not supported.")
    dt = req.horizon / (req.samples - 1)
    ad, bd, _, _, _ = signal.cont2discrete((a, b * gain_factor, c, d), dt)
    time = np.linspace(0.0, req.horizon, req.samples)
    state = np.zeros(a.shape[0])
    output = np.zeros(req.samples)
    inp = np.zeros(req.samples)
    integ = 0.0
    deriv = 0.0
    prev_y = 0.0
    kp = values[0]
    ki = values[1] if len(values) >= 2 else 0.0
    kd = values[2] if len(values) >= 3 else 0.0
    alpha = tau / (tau + dt)
    for k in range(req.samples):
        y = float((c @ state).item())
        output[k] = y
        error = 1.0 - y
        deriv = alpha * deriv + (1 - alpha) * (y - prev_y) / dt
        raw = kp * error + ki * integ - kd * deriv
        u = float(np.clip(raw, -req.max_input, req.max_input))
        inp[k] = u
        if ki and (abs(raw - u) < 1e-12 or (raw > u and error < 0) or (raw < u and error > 0)):
            integ += error * dt
        state = ad @ state + bd[:, 0] * u
        prev_y = y
        if not np.all(np.isfinite(state)) or np.max(np.abs(state)) > 1e12:
            raise ValueError("Divergent simulated state.")
    return _metrics(time, output, inp)


def _pole_design(num: np.ndarray, den: np.ndarray, req: Requirements, speed: float):
    a, b, c, d = signal.tf2ss(num, den)
    n = a.shape[0]
    zeta = 0.75 if req.max_overshoot <= 5 else max(0.35, min(0.85, -np.log(max(req.max_overshoot, 0.1) / 100) / np.sqrt(np.pi ** 2 + np.log(max(req.max_overshoot, 0.1) / 100) ** 2)))
    wn = speed * 4.0 / (max(zeta, 0.3) * req.max_settling)
    if n == 1:
        poles = [-speed * 4.0 / req.max_settling]
    else:
        real = -zeta * wn
        imag = wn * np.sqrt(max(0, 1 - zeta ** 2))
        poles = [real + 1j * imag, real - 1j * imag]
        poles += [-wn * (2.5 + i * 0.55) for i in range(n - 2)]
    k = signal.place_poles(a, b, poles).gain_matrix
    inv = np.linalg.solve(a - b @ k, b)
    kr = -1.0 / float((c @ inv).item())
    if not np.isfinite(kr) or np.max(np.abs(k)) > 1e8:
        raise ValueError("Unreasonable state-feedback gain.")
    return a, b, c, k, kr


def _predict_pole(num: np.ndarray, den: np.ndarray, req: Requirements, speed: float,
                  time: np.ndarray) -> tuple[Trace, np.ndarray, float]:
    a, b, c, k, kr = _pole_design(num, den, req, speed)
    acl = a - b @ k
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=signal.BadCoefficients)
        _, y, _ = signal.lsim((acl, b * kr, c, np.zeros((1, 1))), U=np.ones_like(time), T=time)
        _, u, _ = signal.lsim((acl, b * kr, -k, np.array([[kr]])), U=np.ones_like(time), T=time)
    return _metrics(time, np.ravel(y), np.ravel(u)), k, kr


def _simulate_pole(num: np.ndarray, den: np.ndarray, req: Requirements, k: np.ndarray,
                   kr: float, gain_factor: float) -> Trace:
    a, b, c, _ = signal.tf2ss(num, den)
    dt = req.horizon / (req.samples - 1)
    ad, bd, _, _, _ = signal.cont2discrete((a, b * gain_factor, c, np.zeros((1, 1))), dt)
    time = np.linspace(0, req.horizon, req.samples)
    state = np.zeros(a.shape[0])
    y = np.zeros(req.samples)
    u = np.zeros(req.samples)
    for i in range(req.samples):
        y[i] = float((c @ state).item())
        u[i] = float(np.clip(kr - (k @ state).item(), -req.max_input, req.max_input))
        state = ad @ state + bd[:, 0] * u[i]
        if not np.all(np.isfinite(state)) or np.max(np.abs(state)) > 1e12:
            raise ValueError("Divergent simulated state.")
    return _metrics(time, y, u)


def advise(num: np.ndarray, den: np.ndarray, req: Requirements,
           full_state_available: bool = False, search_iterations: int = 12) -> list[Candidate]:
    num, den = validate_plant(num, den)
    validate_requirements(req)
    time = np.linspace(0, req.horizon, req.samples)
    dc_gain = abs(num[-1] / den[-1])
    sign = np.sign(num[-1] / den[-1])
    gain_scale = 1 / dc_gain
    tau = max(req.horizon / (req.samples - 1), min(0.1, req.max_settling * 0.03))
    scenarios = {"Nominal constrained": 1.0}
    if req.gain_uncertainty > 0:
        scenarios["Low gain"] = 1 - req.gain_uncertainty
        scenarios["High gain"] = 1 + req.gain_uncertainty
    candidates = []
    for name, count in (("P", 1), ("PI", 2), ("PID", 3)):
        bounds = [(0, 30 * gain_scale)] * count
        if name == "PID":
            bounds[2] = (0, 10 * gain_scale * req.max_settling)

        def objective(x):
            try:
                tr = predict_output_feedback(num, den, name, sign * np.asarray(x), time, tau)
                return 20 * _penalty(tr, req) + _score(tr, req, count / 3)
            except (ValueError, OverflowError, np.linalg.LinAlgError):
                return 1e6

        result = optimize.differential_evolution(
            objective, bounds, seed=17, maxiter=search_iterations, popsize=5,
            polish=False, tol=0.05, updating="immediate")
        values = sign * result.x
        try:
            prediction = predict_output_feedback(num, den, name, values, time, tau)
            actual = {label: simulate_feedback(num, den, name, values, req, factor, tau)
                      for label, factor in scenarios.items()}
            failures = [failure for label, tr in actual.items() for failure in _violations(tr, req, label)]
            worst = max(actual.values(), key=lambda tr: _penalty(tr, req) + _score(tr, req, count / 3))
            score = _score(worst, req, count / 3) + 100 * sum(_penalty(tr, req) for tr in actual.values())
            params = dict(zip(("Kp", "Ki", "Kd"), map(float, values)))
            if name == "PID":
                params["derivative_filter_tau"] = tau
            candidates.append(Candidate(name, params, prediction, actual, not failures, failures, score))
        except (ValueError, OverflowError, np.linalg.LinAlgError) as exc:
            candidates.append(Candidate(name, {}, None, {}, False, ["simulation failure"], 1e9, str(exc)))

    if full_state_available:
        best = None
        for speed in np.linspace(0.6, 2.4, 10):
            try:
                prediction, k, kr = _predict_pole(num, den, req, float(speed), time)
                actual = {label: _simulate_pole(num, den, req, k, kr, factor)
                          for label, factor in scenarios.items()}
                failures = [failure for label, tr in actual.items() for failure in _violations(tr, req, label)]
                worst = max(actual.values(), key=lambda tr: _penalty(tr, req) + _score(tr, req, 1))
                score = _score(worst, req, 1) + 100 * sum(_penalty(tr, req) for tr in actual.values())
                item = Candidate("Pole placement", {"K": k.flatten().tolist(), "Kr": float(kr)},
                                 prediction, actual, not failures, failures, score,
                                 "Assumes ideal measurement of every state; reference prefilter is nominal.")
                if best is None or item.score < best.score:
                    best = item
            except (ValueError, OverflowError, np.linalg.LinAlgError):
                continue
        if best:
            candidates.append(best)
        else:
            candidates.append(Candidate("Pole placement", {}, None, {}, False,
                                        ["design failed"], 1e9,
                                        "Could not place stable, usable poles for this plant."))
    return sorted(candidates, key=lambda item: (not item.feasible, item.score))
