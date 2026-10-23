"""Streamlit interface for the simulation-only Controller Advisor."""

import json
import time

import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

from engine import Requirements, advise, parse_coefficients


st.set_page_config(page_title="Controller Advisor", layout="wide")
st.title("Controller Advisor")
st.caption("Enter a linear plant and client requirements. Compare predicted and constrained simulated responses.")

with st.sidebar:
    st.header("Plant G(s)")
    numerator = st.text_input("Numerator coefficients (descending powers of s)", "1")
    denominator = st.text_input("Denominator coefficients", "1, 2, 1")
    st.caption("Example: 1 / (s² + 2s + 1). Continuous time; unit-step reference.")
    full_state = st.checkbox("Every state is available for feedback", False,
                             help="Required for pole placement. Ideal state measurement is assumed.")
    st.header("Client requirements")
    max_os = st.number_input("Maximum overshoot (%)", 0.0, 100.0, 10.0, 1.0)
    max_ts = st.number_input("Maximum settling time (s)", 0.1, 1000.0, 8.0, 0.5)
    max_ess = st.number_input("Maximum steady-state error (fraction of unit step)", 0.0, 1.0, 0.05, 0.01)
    max_u = st.number_input("Actuator input limit |u|", 0.01, 100000.0, 10.0, 0.5)
    unc = st.slider("Plant input-gain uncertainty (%)", 0, 50, 10, 5)
    horizon = st.number_input("Simulation horizon (s)", 2.0, 2000.0, 30.0, 1.0)
    preference = st.selectbox("Rank feasible choices by", [
        "Balanced", "Lowest input effort", "Fastest response", "Simplest controller"])
    search = st.slider("Search budget", 4, 24, 12, 2,
                       help="More iterations may find better P/PI/PID gains but take longer.")
    run = st.button("Design and simulate", type="primary", width="stretch")

if not run:
    st.info("Adjust the plant and requirements, then select **Design and simulate**.")
    st.markdown("**What the outputs mean:** *Prediction* is an unconstrained nominal linear model. "
                "*Simulation* includes the input limit and the selected plant-gain changes. "
                "A candidate is recommended only if every simulated scenario meets every hard requirement.")
    st.stop()

try:
    num = parse_coefficients(numerator)
    den = parse_coefficients(denominator)
    req = Requirements(max_overshoot=max_os, max_settling=max_ts,
                       max_steady_error=max_ess, max_input=max_u,
                       gain_uncertainty=unc / 100, horizon=horizon,
                       samples=1001, preference=preference)
    start = time.perf_counter()
    with st.spinner("Tuning candidates and validating full-plant simulations..."):
        candidates = advise(num, den, req, full_state, search)
    elapsed = time.perf_counter() - start
except Exception as exc:
    st.error(f"Cannot analyse this input: {exc}")
    st.stop()

feasible = [c for c in candidates if c.feasible]
if feasible:
    winner = feasible[0]
    st.success(f"Recommended among tested candidates: **{winner.name}**")
else:
    winner = candidates[0]
    st.warning("No tested candidate met every hard requirement in every selected scenario. "
               f"Closest tested candidate: **{winner.name}**. Review the failed limits below.")
st.caption(f"Evaluated {len(candidates)} controller structures in {elapsed:.1f} s. "
           "The finite search budget does not prove global optimality or mathematical infeasibility.")

rows = []
for c in candidates:
    nominal = c.scenarios.get("Nominal constrained")
    rows.append({
        "Controller": c.name,
        "Passes all scenarios": "Yes" if c.feasible else "No",
        "Predicted Ts (s)": round(c.predicted.settling, 3) if c.predicted and np.isfinite(c.predicted.settling) else None,
        "Simulated Ts (s)": round(nominal.settling, 3) if nominal and np.isfinite(nominal.settling) else None,
        "Simulated OS (%)": round(nominal.overshoot, 2) if nominal else None,
        "Simulated e_ss": round(nominal.steady_error, 4) if nominal else None,
        "Peak |u|": round(nominal.peak_input, 3) if nominal else None,
        "Failed checks": ", ".join(c.failures) if c.failures else "None",
    })
st.subheader("Controller comparison")
st.dataframe(rows, width="stretch", hide_index=True)

choice = st.selectbox("Inspect candidate", [c.name for c in candidates],
                      index=[c.name for c in candidates].index(winner.name))
selected = next(c for c in candidates if c.name == choice)
if selected.predicted is None:
    st.error(selected.note or "This candidate could not be simulated.")
    st.stop()

fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
axes[0].plot(selected.predicted.time, selected.predicted.output, "k--", lw=2,
             label="Nominal linear prediction")
for label, trace in selected.scenarios.items():
    axes[0].plot(trace.time, trace.output, lw=1.6, label=label)
axes[0].axhline(1, color="gray", lw=0.8)
axes[0].axhline(1 + req.max_overshoot / 100, color="red", ls=":", lw=1,
                label="Overshoot limit")
axes[0].axvline(req.max_settling, color="purple", ls=":", lw=1,
                label="Settling deadline")
axes[0].set_ylabel("Output y")
axes[0].grid(alpha=0.25)
axes[0].legend(fontsize=8)

axes[1].plot(selected.predicted.time, selected.predicted.input, "k--", lw=2,
             label="Predicted input")
for label, trace in selected.scenarios.items():
    axes[1].plot(trace.time, trace.input, lw=1.4, label=label)
axes[1].axhline(req.max_input, color="red", ls=":", lw=1)
axes[1].axhline(-req.max_input, color="red", ls=":", lw=1)
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Control input u")
axes[1].grid(alpha=0.25)
axes[1].legend(fontsize=8)
st.pyplot(fig, clear_figure=True)

left, right = st.columns(2)
with left:
    st.subheader("Designed parameters")
    st.json(selected.parameters)
with right:
    st.subheader("Interpretation")
    st.write("**Status:** " + ("All tested limits met" if selected.feasible else "Some limits failed"))
    if selected.failures:
        st.write("**Failed checks:** " + ", ".join(selected.failures))
    if selected.note:
        st.caption(selected.note)

payload = {
    "plant": {"numerator": num.tolist(), "denominator": den.tolist()},
    "requirements": req.__dict__,
    "full_state_available": full_state,
    "candidates": [{
        "name": c.name, "parameters": c.parameters, "feasible": c.feasible,
        "failed_checks": c.failures, "score": c.score,
        "scenarios": {label: {"overshoot_percent": tr.overshoot,
                               "settling_time_s": tr.settling if np.isfinite(tr.settling) else None,
                               "steady_error": tr.steady_error, "peak_input": tr.peak_input,
                               "input_energy": tr.input_energy}
                      for label, tr in c.scenarios.items()}
    } for c in candidates]
}
st.download_button("Download numerical results (JSON)",
                   json.dumps(payload, indent=2), "controller_results.json", "application/json")
st.caption("Research prototype: stable, strictly proper SISO plants of order 1–8. "
           "Gain uncertainty is applied at the plant input. Results are simulation-only.")
