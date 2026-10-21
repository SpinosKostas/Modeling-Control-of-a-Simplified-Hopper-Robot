# Modeling and Control of a Simplified Hopper Robot

Simulation and control of a simplified 2D hopper robot, using a **PD controller** first and then a **Quadratic Programming (QP)** controller solved with a **Primal-Dual Augmented Lagrangian Method (ALM) with slack variables**.

Written in Python with NumPy and Matplotlib only, with no external optimization solver.

📄 Full write-up: [`Modeling_Control_of_a_Simplified_Hopper_Robot.pdf`](Modeling_Control_of_a_Simplified_Hopper_Robot.pdf)

---

## Contents

| File | Description |
|------|-------------|
| [`Hopper_robot.ipynb`] (Hopper_robot.ipynb) | Dynamics, integrators, ALM-based QP solver, trajectory-tracking simulation, plots and animation |
| `Modeling_Control_of_a_Simplified_Hopper_Robot.pdf` | Paper describing the model, controller, and QP/ALM derivation |

---

## The System

The robot lives in the 2D plane. A point mass `m = 5 kg` sits at the end of a rod whose base is fixed at the origin `(0, 0)`, and the rod length is dynamic.

**State and inputs**

```
x = [x, y, ẋ, ẏ]ᵀ ∈ ℝ⁴        u = [f, τ]ᵀ ∈ ℝ²
```

- `f`: force applied along the rod, between the body and the ground
- `τ`: force applied to the body, perpendicular to `f`

**Continuous dynamics**

```
        ⎡ ẋ                    ⎤
f(x,u) =⎢ ẏ                    ⎥ ,     r = [x, y]ᵀ / √(x² + y²),   g = 9.81
        ⎢ (rₓ·f − r_y·τ)/m     ⎥
        ⎣ (r_y·f + rₓ·τ)/m − g ⎦
```

---

## Methods

### 1. Numerical integration

The paper discretizes the system with four integrators:

1. Euler
2. Semi-implicit Euler
3. Runge-Kutta 4th order (RK4)
4. Midpoint

For the PD set-point experiment (`dt = 0.05 s`, target `[1, 3]`), the number of iterations to converge was:

| Integrator | Iterations to converge |
|------------|:----------------------:|
| Euler | 58 |
| Semi-implicit Euler | 59 |
| RK4 | 41 |
| Midpoint | 40 |

> The notebook implements **Euler** and **RK4** for simplicity and RK4 is used in the QP simulation.

### 2. PD controller

The PD law produces a desired acceleration from the position and velocity errors:

```
a_des = Kp·e + Kd·ė
```

Inverting the dynamics gives the input forces:

```
f = m ( rₓ·aₓ,des + r_y·(a_y,des + g) )
τ = m ( −r_y·aₓ,des + rₓ·(a_y,des + g) )
```

### 3. QP controller with Primal-Dual ALM

At every time step the controller solves

```
min   ½ uᵀQu + qᵀu
 u,s≥0
s.t.  Au − b = 0              (Newton's second law)
      Cu + s − d = 0          (optional input limits, via slack s)
```

where

- `A = (1/m)·[[rₓ, −r_y], [r_y, rₓ]]` and `b = a_des + [0, g]ᵀ`, with `a_des` coming from the PD law
- `Q = I` and `q = −u_prev`, which penalizes large forces and encourages smooth motion
- `C`, `d` encode optional limits `0 ≤ f ≤ f_max`, `|τ| ≤ τ_max`

The solver alternates between:

1. **Slack update**
2. **Primal update**: a linear solve on the Augmented Lagrangian
3. **Dual update** of the multipliers `λ`, `μ`
4. **Penalty increase**: `ρ` is multiplied by 5 when feasibility stalls
5. **KKT check**: primal feasibility `‖·‖ < ε_feas` and stationarity `‖·‖ < ε_stat`

The optimal `u_opt` is then integrated through the dynamics and stored as `u_prev` for the next step.

---

## Results

The QP controller tracks sinusoidal reference trajectories (`ω = 2π`):

| Figure | Reference trajectory |
|:------:|----------------------|
| 5 | `x = 0.3·sin(ωt)`, `y = 1 + 0.2·sin(2ωt)` |
| 6 | `x = 0.5·sin³(ωt)`, `y = 1 + 0.4·sin(2ωt)` |
| 7 | `x = sin(ωt) + sin(2ωt)`, `y = 1 + 0.4·sin(2ωt)` |

Tracking is very close for smooth references. As the trajectory becomes more complex, the solver has a harder time following it, with visible deviation near sharp changes and at the start of each run.

---

## Getting Started

### Requirements

- Python 3.8+
- `numpy`
- `matplotlib`
- `jupyter` (and `ipympl` for the interactive animation)

```bash
pip install numpy matplotlib jupyter ipympl
```

### Run

```bash
jupyter notebook Hopper_robot.ipynb
```

Run the cells in order. The notebook is organized as:

1. System dynamics
2. Integrators (Euler, RK4)
3. Primal-Dual ALM solver (`primal_dual_alm`)
4. Closed-loop simulation and tracking plot
5. Animation of the hopper (`%matplotlib widget`, requires `ipympl`)

### Configuration

Main parameters in the simulation cell:

| Parameter | Value in notebook | Meaning |
|-----------|-------------------|---------|
| `dt` | `0.01` | Time step [s] |
| `T` | `4.0` | Simulation length [s] |
| `Kp` | `diag(150, 80)` | Proportional gains |
| `Kd` | `diag(18, 15)` | Derivative gains |
| `Ax, Ay, omega` | `0.5, 0.4, 2π` | Reference trajectory parameters |
| `f_max, tau_max` | `10.0, 5.0` | Input limits (defined, see note below) |

Edit the `xd`, `xd_dot` and `xd_ddot` expressions to change the reference trajectory.

---

## Notes and Limitations

- **Inequality constraints are implemented but disabled.** `primal_dual_alm` supports `C`, `d` and slack variables, but the simulation passes empty matrices (no input limits), as in the paper. To enable them, pass the `C` and `d` defined in the loop instead of `np.zeros((0, 2))` and `np.zeros((0, 1))`. Note that the exact-tracking equality `Au = b` can then become infeasible when the limits are active.
- **Gains are tuned by trial and error** for a given `dt`. The paper reports `Kp = [180, 80]`, `Kd = [18, 15]` with `dt = 0.05 s` for the PD set-point test. Large gains can cause instability, so a smaller `dt` and smaller gains give better robustness.
- **Singularity at the origin.** The rod direction `r = [x, y]/‖[x, y]‖` is undefined when the mass reaches `(0, 0)`.

---

## Author

Konstantinos Spinos

## License

Open-source project. Use freely.
