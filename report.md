# Requirement-Based Controller Recommendation for Higher-Order SISO Systems

**Simulation-only BTech control systems project | Research proposal and prototype specification | 23 October 2026**

## Abstract

Selecting a controller from client requirements can be difficult when a plant has several dynamic modes and the controller must satisfy competing limits on overshoot, settling time, steady-state error and actuator input. This project develops an interactive, simulation-only decision aid for stable, continuous-time, single-input single-output (SISO) plants of order one to eight. Given a transfer function and numerical requirements, the prototype tunes proportional (P), proportional-integral (PI) and filtered proportional-integral-derivative (PID) controllers; it optionally considers state-feedback pole placement when all states are available. It predicts each nominal unconstrained response, then validates the candidate on the full plant with actuator saturation and specified input-gain variation. Candidates that violate any hard requirement are rejected, while feasible designs are ranked by the user's preference for speed, control effort or simplicity. The proposed research evaluates how often nominal prediction produces a feasible recommendation after constrained validation. No hardware result or numerical improvement is claimed before the planned benchmark is run.

**Keywords:** controller selection, higher-order systems, PID, pole placement, simulation, actuator constraints, robustness.

## 1. Problem Statement and Motivation

A client may specify a maximum overshoot, settling deadline, steady-state error and input limit without knowing which controller architecture to select. Higher-order plants make intuition based on second-order approximations less reliable, while actuator limits and parameter variation can invalidate a nominal design. The engineering task is to generate candidate controllers, check the complete requirements on the original plant and explain why each design passes or fails. The term "best" only has meaning after hard constraints and a ranking preference are defined.

## 2. Literature and Research Gap

Automatic controller tuning and architecture search already exist. Yang *et al.* combine structure search with parameter optimisation [1]. Fenili *et al.* design robust PID control for uncertain plants of arbitrary order [2]. Tang translates user response specifications into systematic MPC tuning [3]. Weekers *et al.* study automated settling-time optimisation under uncertainty [4]. Chen *et al.* optimise PID parameters against several process requirements [5]. Dyvik and Rotondo show that model mismatch can still cause constraint violations in a robust MPC application [6]. Therefore this project does **not** claim to invent automatic controller design.

The focused gap for this BTech study is an **auditable, requirement-level comparison of simple controller families when nominal predictions are rechecked under actuator saturation and plant-gain variation on the full higher-order plant**. The research contribution is the benchmark protocol and measured feasibility disagreement between prediction and validation, together with explicit explanations of failed requirements. This is a project-specific investigation; it is not a claim that no previous paper has compared controllers or model uncertainty.

## 3. Aim, Objectives and Research Questions

**Aim:** Develop and evaluate an interactive decision aid that recommends a feasible controller for a specified class of linear plants and client requirements.

**Objectives:**

1. Accept a continuous-time transfer function and numerical requirements.
2. Tune P, PI and filtered PID designs; consider pole placement only when full-state feedback is available.
3. Produce a nominal unconstrained prediction and constrained time-domain simulations for each candidate.
4. Reject designs that fail any hard requirement and rank passing designs using stated preferences.
5. Benchmark prediction-versus-validation disagreement across several higher-order plants and requirement sets.

**RQ1:** How frequently does a controller that appears acceptable in nominal linear prediction violate a requirement under actuator saturation or input-gain variation?

**RQ2:** How does the selected architecture change when the priority changes between settling time, control effort and controller simplicity?

**RQ3:** How does increasing plant order affect runtime and feasibility for the tested search budget?

## 4. System Scope and Definitions

The prototype accepts real-coefficient, stable, strictly proper, continuous-time SISO plants of order 1-8 with finite nonzero DC gain. The reference is a unit step and the simulations have a user-selected finite horizon. A hard requirement consists of maximum percentage overshoot, maximum 2% settling time, maximum steady-state tracking error and maximum absolute actuator input. The uncertainty experiment varies plant input gain by plus and minus a specified percentage. The system is simulation-only. Time delay, MIMO dynamics, measurement noise, unmodelled nonlinearities and physical implementation are outside the present scope.

The client's requested "type" must be defined carefully. In classical control, **system type** is the number of open-loop integrators and relates to steady-state tracking of polynomial inputs. This prototype instead accepts a direct unit-step steady-state error requirement. A later extension can support ramp tracking and explicit type constraints.

## 5. Proposed Methodology

**Step 1 - Input and eligibility checks.** Parse numerator and denominator coefficients in descending powers of s. Reject zero, improper, unstable, integrating and above-order-eight plants. Check that the simulation horizon exceeds the requested settling deadline. Pole placement is eligible only if the user confirms that all states can be measured or otherwise made available; the current prototype assumes ideal measurement and does not design an observer.

**Step 2 - Candidate generation.** Use bounded differential-evolution search to tune P, PI and filtered PID gains against a penalty for requirement violations plus a preference score. Use a small sweep of desired pole speeds to produce a state-feedback candidate. The finite search is heuristic and cannot certify global optimality or mathematical infeasibility.

**Step 3 - Nominal prediction.** For each output-feedback candidate, form the closed-loop transfer functions from reference to output and to controller input. Simulate an unconstrained unit step on the nominal plant. This is called the prediction. For pole placement, predict using the nominal state-space closed loop and a reference prefilter.

**Step 4 - Constrained validation.** Discretise the original full-order plant with a zero-order hold. Apply each controller step by step with actuator input clipping. For the PID family, filter the derivative term and conditionally integrate during saturation. Repeat for nominal, low-gain and high-gain plants. Report each scenario separately.

**Step 5 - Eligibility and ranking.** A candidate passes only when every simulated scenario meets every hard limit. Among feasible candidates, calculate a preference score based on normalised settling time, integrated squared input and a simple architecture-complexity index. Report the lowest-scoring feasible choice and the full comparison. If none passes, say "no tested candidate meets all requirements" and show the closest tested option solely for diagnosis. Control effort and complexity are **proxies**, not currency costs.

## 6. Performance Measures and Equations

For the reference r(t)=1, use tracking error e(t)=1-y(t). Overshoot is max(0, max(y)-1) x 100%. Settling time is the first instant after which |e(t)| remains at or below 0.02 for the remainder of the chosen horizon. Steady-state error is the absolute mean error over the final 5% of simulation samples. Peak input is max|u(t)|. Input-energy proxy is the numerical integral of u(t)^2 over the horizon. The candidate is feasible only when every scenario satisfies all four client bounds. A non-settled response has infinite reported settling time. These finite-horizon definitions must be retained when MATLAB and Python results are compared.

## 7. Experimental Design and Analysis Plan

Use a benchmark set of at least 12 stable plants spanning orders 2, 3, 5 and 8, including repeated real poles, separated time scales and lightly damped modes. Use physically interpretable examples where possible, such as cascaded tanks, motor speed dynamics and flexible modes. For each plant, test at least three requirement profiles (relaxed, medium and stringent), two actuator limits and gain uncertainties of 0%, 10% and 20%. Keep the same random seed and search budget for every controller family. Record feasible rate, the fraction of nominally passing designs that fail constrained validation, worst requirement violation, runtime, input energy and architecture selected. Do not infer significance from a single example; report distributions and individual counterexamples. Include a baseline that selects the nominally best PID without uncertainty validation, then compare its pass rate with the proposed validation-and-rejection workflow.

The supplied app is the working prototype for this experiment. The benchmark results table and claims of improvement should be completed only after the full experiment is run. A MATLAB replication should implement the same transfer functions, time grid, saturation logic and metric definitions before numerical comparison. Differences between solvers and discrete stepping should be reported rather than hidden.

## 8. Expected Deliverables

- Interactive Python interface with requirements entry, controller parameters, response plots, feasibility explanations and JSON export.
- Reproducible benchmark dataset, settings and numerical results.
- MATLAB cross-check of selected cases using the same plant and metric definitions.
- Final analysis of where nominal design succeeds or fails after constraints and uncertainty are added.

## 9. Limitations and Research Integrity

The tool recommends among tested candidates only. An empty feasible set is not proof that no possible controller can meet the requirements. Pole placement assumes full-state measurement and a nominal reference prefilter; it must not be offered as an equivalent output-feedback solution when states are unavailable. The gain-uncertainty test is one narrow form of model mismatch. The simulation cannot establish real hardware performance, reliability or monetary cost. No artificial intelligence model is required: "prediction" here means a model-based response, and the actual value of the research is transparent verification.

## References (IEEE style; journal articles only)

[1] K. Yang, Z. Wang, P. Guo, and Q. Li, "A controller design method based on the integration of structure search and parameter optimization," *Advances in Mechanical Engineering*, vol. 16, no. 1, 2024, doi: 10.1177/16878132231223387.

[2] E. P. Fenili, L. A. Mozelli, and F. O. Souza, "LMI-based robust PID controller with measurement filter for nth-order systems," *European Journal of Control*, vol. 77, art. no. 100980, 2024, doi: 10.1016/j.ejcon.2024.100980.

[3] W. Tang, "Systematic MPC tuning with direct response shaping: Parameterization and Inverse optimization-based Tuning Approach (PITA)," *Control Engineering Practice*, vol. 153, art. no. 106103, 2024, doi: 10.1016/j.conengprac.2024.106103.

[4] W. Weekers, D. Kostic, A. Saccon, and N. van de Wouw, "Data-Based Settling-Time Optimization for Linear Feedback Control Systems Using Global Extremum Seeking," *IEEE Transactions on Control Systems Technology*, vol. 33, no. 1, pp. 343-353, 2025, doi: 10.1109/TCST.2024.3473300.

[5] H. Chen, S. Bowels, B. Zhang, and T. Fuhlbrigge, "Controller parameter optimization for complex industrial system with uncertainties," *Measurement and Control*, vol. 52, no. 7-8, pp. 888-895, 2019, doi: 10.1177/0020294019830108.

[6] M. Dyvik and D. Rotondo, "Tube MPC for a two-tank system based on Eigensystem Realization Algorithm," *Journal of Process Control*, vol. 151, art. no. 103434, 2025, doi: 10.1016/j.jprocont.2025.103434.
