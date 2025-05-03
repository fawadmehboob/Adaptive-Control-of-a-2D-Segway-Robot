---
layout: default
title: Segway Robot Dynamics
---

<script type="text/javascript" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

# Segway Robot Dynamics via Lagrange Approach

This document provides a detailed derivation of the dynamics of a 2D Segway robot using the Lagrangian mechanics approach. The Segway is modeled as a two-wheeled inverted pendulum with a wheel assembly and a body pivoting above the wheel axle. The equations of motion are derived using the Lagrange method, which involves computing the kinetic and potential energies, forming the Lagrangian, and applying the Euler-Lagrange equations.

---

## System Description

The Segway robot consists of:

- **Wheels**: A combined wheel assembly with mass \( m_w \), radius \( r \), and moment of inertia \( I_w \).
- **Body (Link)**: A rigid body with mass \( m_b \), length \( L \), and center of mass at a distance \( l = L/2 \) from the pivot (wheel axle). The moment of inertia of the body about its center of mass is \( I_b \).
- **Coordinates**: Generalized coordinates \( x \) (horizontal position of the wheel axle) and \( \theta \) (angle of the body from the vertical, where \( \theta = 0 \) is upright).
- **External Torque**: A torque \( \tau \) is applied to the wheels.

---

## Parameters

- \( m_w = 1.0 \, \text{kg} \)
- \( m_b = 5.0 \, \text{kg} \)
- \( r = 0.1 \, \text{m} \)
- \( L = 1.0 \, \text{m} \)
- \( l = L/2 = 0.5 \, \text{m} \)
- \( I_w = \frac{1}{2} m_w r^2 \)
- \( I_b = \frac{1}{12} m_b L^2 \)
- \( g = 9.81 \, \text{m/s}^2 \)
- \( \tau \): Torque applied to the wheels

---

## Lagrange Approach

The Lagrange method involves the following steps:

1. Compute the kinetic energy \( T \) of the system.
2. Compute the potential energy \( V \) of the system.
3. Form the Lagrangian \( \mathcal{L} = T - V \).
4. Apply the Euler-Lagrange equations:

\[
\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}_i} \right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i
\]

---

## Step 1: Kinetic Energy (T)

### Wheel Kinetic Energy

- Translational: The wheel’s center moves with velocity \( \dot{x} \)
- Rotational: The wheel rotates with angular velocity \( \dot{\phi} = \dot{x}/r \)

\[
T_{w,\text{trans}} = \frac{1}{2} m_w \dot{x}^2
\]
\[
T_{w,\text{rot}} = \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2
\]
\[
T_w = \frac{1}{2} m_w \dot{x}^2 + \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2
\]

### Body Kinetic Energy

The position of the body’s center of mass:

\[
x_{\text{cm}} = x + l \sin \theta, \quad y_{\text{cm}} = r + l \cos \theta
\]

Velocities:

\[
\dot{x}_{\text{cm}} = \dot{x} + l \dot{\theta} \cos \theta, \quad \dot{y}_{\text{cm}} = -l \dot{\theta} \sin \theta
\]

Translational kinetic energy:

\[
T_{b,\text{trans}} = \frac{1}{2} m_b \left[ \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \right]
\]

Rotational kinetic energy:

\[
T_{b,\text{rot}} = \frac{1}{2} I_b \dot{\theta}^2
\]

Total:

\[
T_b = \frac{1}{2} m_b \left[ \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \right] + \frac{1}{2} I_b \dot{\theta}^2
\]

### Total Kinetic Energy

\[
T = \frac{1}{2} \left( m_w + m_b + \frac{I_w}{r^2} \right) \dot{x}^2 + m_b l \dot{x} \dot{\theta} \cos \theta + \frac{1}{2} \left( m_b l^2 + I_b \right) \dot{\theta}^2
\]

---

## Step 2: Potential Energy

\[
V = m_b g (r + l \cos \theta) \Rightarrow V = m_b g l \cos \theta
\]

---

## Step 3: Lagrangian

\[
\mathcal{L} = T - V = \frac{1}{2} \left( m_w + m_b + \frac{I_w}{r^2} \right) \dot{x}^2 + m_b l \dot{x} \dot{\theta} \cos \theta + \frac{1}{2} \left( m_b l^2 + I_b \right) \dot{\theta}^2 - m_b g l \cos \theta
\]

---

## Step 4: Euler-Lagrange Equations

### For \( x \)

\[
\left( m_w + m_b + \frac{I_w}{r^2} \right) \ddot{x} + m_b l \ddot{\theta} \cos \theta - m_b l \dot{\theta}^2 \sin \theta = \frac{\tau}{r}
\]

### For \( \theta \)

\[
m_b l \ddot{x} \cos \theta + \left( m_b l^2 + I_b \right) \ddot{\theta} + m_b g l \sin \theta = -\tau
\]

---

## Equations of Motion in Matrix Form

### Standard form:

\[
\begin{bmatrix}
\left( m_w + m_b + \frac{I_w}{r^2} \right) & m_b l \cos \theta \\
m_b l \cos \theta & m_b l^2 + I_b
\end{bmatrix}
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
+
\begin{bmatrix}
- m_b l \dot{\theta}^2 \sin \theta \\
m_b g l \sin \theta
\end{bmatrix}
=
\begin{bmatrix}
\frac{\tau}{r} \\
-\tau
\end{bmatrix}
\]

### Code-scaled form:

\[
\begin{bmatrix}
( m_w + m_b ) r^2 + I_w & m_b r l \cos \theta \\
m_b r l \cos \theta & m_b l^2 + I_b
\end{bmatrix}
\begin{bmatrix}
\frac{\ddot{x}}{r} \\
\ddot{\theta}
\end{bmatrix}
+
\begin{bmatrix}
m_b r l \dot{\theta}^2 \sin \theta \\
m_b g l \sin \theta
\end{bmatrix}
=
\begin{bmatrix}
\tau \\
0
\end{bmatrix}
\]

---

## Conclusion

The Lagrangian approach provides a systematic method to derive the nonlinear dynamics of the Segway robot, capturing the coupling between the wheel’s motion \( x \) and the body’s angle \( \theta \). These equations form the basis for implementing control strategies, such as adaptive control or nonlinear stabilization.