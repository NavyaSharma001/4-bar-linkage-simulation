<img width="800" height="450" alt="IMG_6344-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/e2323455-e232-491c-b63e-09754b9f0e52" />

# 4-Bar Crank-Rocker Linkage Simulation

# Analytical 4-Bar Linkage Planar Kinematics Engine

A high-performance MATLAB kinematic simulator that models the planar motion profiles of a closed-loop 4-bar mechanism. This repository implements **Freudenstein’s Analytical Vector-Loop Equations** to resolve exact joint coordinates, position traces, and instantaneous angular velocities. It utilizes a numerical differentiation pass to cleanly bypass structural toggle singularities.

---

## ⚙️ Design Parameters & Grashof Sufficiency

The simulation utilizes a **Crank-Rocker** configuration layout with the following precise link lengths calibrated in millimeters:
* **Ground Link ($l_1$):** $160\text{ mm}$ (Fixed base pivot spacing)
* **Input Crank ($l_2$):** $40\text{ mm}$ (Driven at $100\text{ RPM} \approx 10.472\text{ rad/s}$)
* **Coupler Link ($l_3$):** $120\text{ mm}$ (Floating mechanical connector link)
* **Output Rocker ($l_4$):** $100\text{ mm}$ (Oscillating rocker arm)

### Grashof Condition Verification:
To guarantee continuous, non-binding rotation of the input driving crank, the geometry must strictly satisfy the Grashof Inequality constraint ($s + l \le p + q$):

$$l_2 + l_1 \le l_3 + l_4 \implies 40 + 160 \le 120 + 100 \implies 200 \le 220 \quad \mathbf{[VERIFIED]}$$

Because the condition is satisfied, the system achieves full continuous $360^\circ$ rotational input capability without mechanical jamming or structural dead-center lockup.

---

## 📐 Mathematical Formulation & Loop Closure

The underlying kinematics uses a closed vector circuit mapped across the mechanism pins:

$$\mathbf{R}_1 + \mathbf{R}_2 + \mathbf{R}_3 + \mathbf{R}_4 = 0$$

Projecting this vector configuration into scalar Cartesian components yields two coupled nonlinear trigonometric equations:

$$l_1 + l_4\cos\theta_4 - l_2\cos\theta_2 - l_3\cos\theta_3 = 0$$

$$l_4\sin\theta_4 - l_2\sin\theta_2 - l_3\sin\theta_3 = 0$$

By isolating the unknown interior parameter ($\theta_3$) and applying Freudenstein's Identity Matrix Constants, the closed-form quadratic roots for the output rocker angle ($\theta_4$) are evaluated explicitly across the operational tracking domain:

$$K_1 = \frac{l_1}{l_2}, \quad K_2 = \frac{l_1}{l_4}, \quad K_3 = \frac{l_1^2 + l_2^2 - l_3^2 + l_4^2}{2l_2l_4}$$

Using the Tangent Half-Angle substitution ($x = \tan(\theta_4/2)$), the nonlinear loop reduces to a standard quadratic structure ($Ax^2 + Bx + C = 0$), solved via:

$$\theta_4 = 2\tan^{-1}\left(\frac{-B - \sqrt{B^2 - 4AC}}{2A}\right)$$

Where the geometric coefficients are driven dynamically by the input crank position:
* $A = \cos\theta_2 - K_1 - K_2\cos\theta_2 + K_3$
* $B = -2\sin\theta_2$
* $C = K_1 - (K_2+1)\cos\theta_2 + K_3$

*(The negative sign before the discriminant enforces the open-branch assembly layout, matching physical CAD configuration behaviors).*

---

## ⚡ Singularity Mitigation via Numerical Differentiation

In pure analytical differentiation, calculating the rocker angular velocity ($\omega_4$) relies on the geometric ratio equation:

$$\omega_4 = \left( \frac{l_2\sin(\theta_3 - \theta_2)}{l_4\sin(\theta_4 - \theta_3)} \right)\omega_2$$

At the limiting dead-center toggle positions ($\theta_2 \approx 130^\circ$ and $\theta_2 \approx 310^\circ$), the coupler and rocker links become collinear, causing $\sin(\theta_4 - \theta_3) \to 0$. In a theoretical numerical script, this division-by-zero forces the velocity outputs to explode into infinite asymptotic spikes.

To bypass this mathematical instability, this framework implements a **discrete numerical differentiation pass**:

$$\omega_4 \approx \left(\frac{\Delta \theta_4}{\Delta \theta_2}\right) \cdot \omega_2 = \left(\frac{\theta_{4, i+1} - \theta_{4, i}}{\theta_{2, i+1} - \theta_{2, i}}\right) \cdot \omega_2$$

By evaluating raw step changes in position arrays rather than trigonometric fractions, the solver completely eliminates singular asymptotic spikes, generating a smooth, physical velocity profile that matches real-world data acquisition systems.

---

## 📁 Repository Directory Layout
```directory
├── /CAD_Models/
│   ├── 4link.SLDASM        # Main SolidWorks assembly mapping mechanical joint links
│   ├── 4link.STEP          # Universal CAD format for cross-platform software integration
│   ├── 100mmlink.SLDPRT    # Bounded Output Rocker link component
│   ├── 160mmlink.SLDPRT    # Rigid base Ground link geometry component
│   ├── 40mmlink.SLDPRT     # Fully rotating input Driving Crank link component
│   └── Part1.SLDPRT        # Base plate / Support fixture geometry
├── four_bar_kinematics.m   # Core closed-loop vector solver and animation script
└── README.md               # Kinematic formulation and parameter documentation
