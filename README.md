# Adaptive-Control-of-a-2D-Segway-Robot

This repository defines a systematic approach of solving a control problem of a 2D Segway robot using Lyapunov-based Adaptive Control. The idea of adaptive control is to control a system under uncertain conditions where some parameter $\theta$ of the system is unknown. In the case of a Segway robot this could be thought of as the friction coefficient at the pivot joint between the wheels and link. The dynamics of this system are given as:

$$ M \ddot{x} + C \ddot{\phi} cos(\phi) - C \dot{\phi}^2 sin(\phi) = \frac{T}{R}$$
$$ I \ddot{\phi} + C \ddot{x} cos(\phi) + \theta \dot{\phi} - C g sin(\phi) = - T$$

Here $M$, $I$, and $C$ are constant system parameters defined as:
$M = m_1 + m_2 +\frac{I_2}{R^2}$, $I = I_1 + m_1 (\frac{l}{2})^2$, and $C = m_1 \frac{l}{2}$; $m_1$, and $m_2$ are the masses of the wheel and the link respectively, $l$ is the link length, $g$ is gravitational constant, $R$ is the wheel radius, $I_2$ is the link inertia, $T$ is the torque input to the wheel, and $\theta$ is the unknown parameter that we shall handle using the adaptive control. A more elaborate detail of the dynamics can be found in the markdown "SegWay Dynamics.md".

# Dynamics Linearization
Since the segway is intended to operate around an equilibrium position, it is reasonable to linearize the dynamics about the point $\phi = 0$, $\dot{\phi} \approx0$, which lead to the dynamics

$$ M \ddot{x} + C \ddot{\phi} = \frac{T}{R} $$
$$ I \ddot{\phi} + C \ddot{x}  = - T - \theta \dot{\phi} - C g \phi $$

### matrix representation
