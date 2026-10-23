# Controller Advisor

An educational, simulation-only tool for requirement-based controller selection on continuous-time, strictly proper, stable SISO plants of order 1-8.

## Run

```powershell
cd controller_advisor
python -m pip install -r requirements.txt
python -m streamlit run app.py
```

Enter numerator and denominator coefficients in descending powers of `s`. For example, `1` and `1, 3, 3, 1` mean `1/(s+1)^3`. The reference is a unit step. The app tunes P, PI and filtered PID output-feedback controllers. It also considers full-state pole placement **only if you confirm that every state is available**.

The tool first predicts an unconstrained nominal response. It then simulates the same designs with an actuator amplitude limit and with the entered plant-gain uncertainty. A recommendation requires every hard specification to pass in every selected validation scenario. Ranking among feasible designs uses the stated preference; it is a design aid, not a proof of global optimality or hardware performance. Modelled cost is control effort and controller complexity, not a monetary price.

Settling time means the first time after which the response stays within 2% of the unit reference until the end of the simulation. A dash means it did not settle within that horizon. Check the chosen horizon before interpreting a failure. The app uses a fixed simulation step and a filtered derivative term with conditional integration to avoid windup during actuator saturation.

## Project files

- `app.py`: interactive interface and plots
- `engine.py`: controller design, prediction, validation and ranking
- `tests/test_engine.py`: core behaviour checks
- `report.md`: research report with journal-only IEEE references

## Scope limits

No time delay, MIMO, noisy sensors, nonlinear plants or discrete-time plant inputs. The uncertainty test varies the plant input gain only. The state-feedback option assumes ideal full-state measurement. The optimiser has a finite search budget, so failure means **none of the generated candidates passed**, not mathematical infeasibility. The app returns the closest tested design for diagnosis when there is no feasible candidate.
