## 1. Introduction

This document presents a detailed, step-by-step derivation of the boundary value problem (BVP) equations for a self-propelled trajectory under thrust, drag, and gravity. We also derive the related and more intuitive initial value problem (IVP) ODE system used to generate a seed solution via `solve_ivp`.

---

## 2. Governing Dynamics in Time

We begin with Newton’s second law for motion under thrust $T$, aerodynamic drag $D$, and gravity $mg$. In two dimensions, with state variables $(x(t),y(t))$ and speed $v(t)$, the vector form is:

$$
 m\frac{d\mathbf{v}}{dt} = \underbrace{T\,\hat{\mathbf{e}}_{\theta}}_{\text{thrust}} - \underbrace{D\,\hat{\mathbf{e}}_{v}}_{\text{drag}} - \underbrace{m g\, \hat{\mathbf{j}}}_{\text{gravity}}.
$$

* $\hat{\mathbf{e}}_{v}$ is the unit vector along the velocity.
* $\hat{\mathbf{e}}_{\theta} = (\cos\theta,\,\sin\theta)$ is the thrust direction (flight-path angle $\theta$).
* In our case, we assume thrust is always along flight path and thus, $\hat{\mathbf{e}}_{v} = \hat{\mathbf{e}}_{\theta}$
* Drag magnitude: $D = \tfrac12\,\rho\,C_d\,A\,v^2$.

Splitting into components:

$$
\begin{aligned}
 m\,\frac{dv}{dt} &= T - D - mg\sin\theta, \\
 m\,v\,\frac{d\theta}{dt} &= -\bigl(T - D\bigr)\sin\theta - mg\cos\theta.
\end{aligned}
$$

Dividing the second equation by $m$ and rearranging gives:

$$
 \frac{d\theta}{dt} = \frac{-\bigl(T - D\bigr)\sin\theta - mg\cos\theta}{m\,v}.
$$

With $v(t) = \|\mathbf{v}\|$, we obtain the IVP system:

$$
 \begin{cases}
 \displaystyle \frac{dx}{dt} = v\cos\theta, & x(0)=0, \\ \\
 \displaystyle \frac{dy}{dt} = v\sin\theta, & y(0)=0, \\ \\
 \displaystyle \frac{dv}{dt} = \frac{T - D}{m} - g\sin\theta, & v(0)=v_0, \\ \\
 \displaystyle \frac{d\theta}{dt} = \frac{-(T - D)\sin\theta - mg\cos\theta}{m\,v}, & \theta(0)=\theta_0.
 \end{cases}
$$

These equations were integrated with `solve_ivp` to generate the seed trajectory.

---

## 3. Reparametrization for the BVP

To solve the trajectory as a boundary value problem (BVP), it's advantageous to reparametrize the system so that the **horizontal position** $x$ becomes the independent variable, rather than time $t$. This is particularly useful because boundary conditions are typically specified in terms of initial and final positions (like start and end $x$-coordinates), rather than start and end times.

### Step 1: Define New Dependent Variables

We define the following functions with respect to $x$:

$$
y = y(x), \quad v = v(x), \quad p = \frac{dy}{dx}, \quad q = \frac{dv}{dx}.
$$

Here:
- $y(x)$ is the vertical position,
- $v(x)$ is the speed,
- $p$ is the slope of the trajectory (i.e., $\tan\theta$),
- $q$ is the spatial rate of change of speed.

### Step 2: Use Chain Rule to Relate $d/dx$ and $d/dt$

We use the chain rule to convert time derivatives into spatial derivatives. The idea is to write $\frac{d}{dx}$ in terms of $\frac{d}{dt}$ using:

$$
\frac{d}{dx} = \frac{1}{\tfrac{dx}{dt}} \frac{d}{dt} = \frac{1}{v\cos\theta} \frac{d}{dt}.
$$

This works because $dx/dt = v\cos\theta$, which is the horizontal component of velocity.

### Step 3: Convert Second Derivatives

We now want to express $y''(x) = \frac{d^2y}{dx^2}$ in terms of $\theta$ and known quantities. A direct way to do this is:

$$
\frac{d^2y}{dx^2} = \frac{d}{dx} \left( \frac{dy}{dx} \right)
= \frac{d}{dx}(p)
= \frac{1}{v\cos\theta} \frac{d}{dt}(p).
$$

But since $p = \tan\theta$, we write:

$$
\frac{d^2y}{dx^2} = \frac{1}{v\cos\theta} \frac{d}{dt}(\tan\theta)
= \frac{1}{v\cos\theta} \cdot \frac{d\theta}{dt} \cdot \frac{1}{\cos^2\theta}.
$$

So far we have:

$$
y''(x) = \frac{1}{v\cos\theta} \cdot \frac{1}{\cos^2\theta} \cdot \frac{d\theta}{dt}.
$$

### Step 4: Substitute for $\frac{d\theta}{dt}$

From Newton’s second law in angular form (or from prior derivation), we know:

$$
\frac{d\theta}{dt} = \frac{-(T - D)\sin\theta - mg\cos\theta}{mv}.
$$

Substitute this into the expression for $y''(x)$:

$$
y''(x) = \frac{1}{v\cos\theta} \cdot \frac{1}{\cos^2\theta} \cdot \frac{-(T - D)\sin\theta - mg\cos\theta}{mv}.
$$

This expression can be simplified by expressing all trigonometric functions in terms of $p = \tan\theta$.

### Step 5: Use Trig Identities

From $p = \tan\theta$, we have:

$$
\cos\theta = \frac{1}{\sqrt{1 + p^2}}, \quad
\sin\theta = \frac{p}{\sqrt{1 + p^2}}.
$$

Substitute these into the above:

$$
y''(x) = \frac{1}{v \cdot \tfrac{1}{\sqrt{1 + p^2}}} \cdot \frac{1}{\left(\tfrac{1}{\sqrt{1 + p^2}}\right)^2}
\cdot \frac{- (T - D) \cdot \tfrac{p}{\sqrt{1 + p^2}} - mg \cdot \tfrac{1}{\sqrt{1 + p^2}}}{mv}.
$$

Simplify all terms:

- The first fraction becomes $\frac{\sqrt{1 + p^2}}{v}$,
- The second becomes $(1 + p^2)$,
- The numerator becomes $-\left((T - D)p + mg\right) / \sqrt{1 + p^2}$.

Putting all this together and simplifying gives:

$$
y''(x) = -\frac{g(1 + p^2)}{v^2}.
$$

---

### 3.1 Second Derivative of $y$

Final simplified form:

$$
y''(x) = -\frac{g(1 + p^2)}{v^2}.
$$

This tells us how the curvature of the trajectory depends on the local slope $p$ and speed $v$.

---

### 3.2 First Derivative of $v$

To compute $v' = \frac{dv}{dx}$, start with:

$$
v' = \frac{1}{v\cos\theta} \frac{dv}{dt}.
$$

From Newton's second law in the tangential direction:

$$
\frac{dv}{dt} = \frac{T - D}{m} - g\sin\theta.
$$

Substitute:

$$
v' = \frac{1}{v\cos\theta} \left( \frac{T - D}{m} - g\sin\theta \right).
$$

Now express $\cos\theta$ and $\sin\theta$ in terms of $p$:

$$
\cos\theta = \frac{1}{\sqrt{1 + p^2}}, \quad \sin\theta = \frac{p}{\sqrt{1 + p^2}}.
$$

So:

$$
v' = \frac{\sqrt{1 + p^2}}{v} \cdot \left( \frac{T - D}{m} \right) - \frac{g\,p}{v}.
$$

---

### Final Residual Form

To prepare this for a numerical BVP solver, you may write the equation in residual form:

$$
v' - \frac{\sqrt{1 + p^2}}{v} \left( \frac{T - D}{m} \right) + \frac{g\,p}{v} = 0.
$$

This form is well-suited for implementation in a solver like `scipy.integrate.solve_bvp`, where you define a system of first-order ODEs.

---

## 4. Final Residual Equations for the BVP

Define residual vector $\mathbf{R} = (R_1,R_2)$ with:

$$
 \begin{aligned}
 R_1(y,v;p)&= y''+g\frac{1+p^2}{v^2}, \\
 R_2(y,v;p)&= v' - \frac{\sqrt{1+p^2}}{v}\Bigl(\frac{T-D}{m}\Bigr) + g\frac{p}{v}.
 \end{aligned}
$$

These match the implementation in `TrajectoryEquation.residuals()`.

---

## 5. Angle-Derivative Focus

The key step is the derivation of

$$
 \frac{d\theta}{dt} = \frac{-(T-D)\sin\theta - mg\cos\theta}{m\,v},
$$

which arises directly from the transverse component of Newton’s law under rotating axes:

$$
m v \frac{d\theta}{dt} = \left( \mathbf{F}_T - \mathbf{F}_D \right) \cdot \hat{\mathbf{e}}_{\perp} - mg\cos\theta.
$$

Projecting thrust minus drag onto the normal direction $\hat{\mathbf{e}}_{\perp}=(-\sin\theta,\cos\theta)$ gives $-(T-D)\sin\theta$, and gravity contributes $-mg\cos\theta$. Dividing by $m v$ yields the above expression.

---

## 6. Conclusion

We have systematically derived the time-based IVP used for seed generation and the spatially reparametrized BVP residuals. The critical angle-derivative formula connects the two formulations and ensures consistency between the IVP seed and the BVP solver.
