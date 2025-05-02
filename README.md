# Adaptive-Control-of-a-2D-Segway-Robot

This repository defines a systematic approach of solving a control problem of a 2D Segway robot using Lyapunov-based Adaptive Control. The idea of adaptive control is to control a system under uncertain conditions where some parameter $\theta$ of the system is unknown. In the case of a Segway robot this could be thought of as the friction coefficient at the pivot joint between the wheels and link. The dynamics of this system are given as:

$$ M \ddot{x} + C \ddot{\phi} cos(\phi) - C \dot{\phi}^2 sin(\phi) = \frac{T}{R}$$
$$ I \ddot{\phi} + C \ddot{x} cos(\phi) + \theta \dot{\phi} - C g sin(\phi) = - T$$

Here $M$, $I$, and $C$ are constant system parameters defined as:
$M = m_1 + m_2 +\frac{I_2}{R^2}$, $I = I_1 + m_1 (\frac{l}{2})^2$, and $C = m_1 \frac{l}{2}$; $m_1$, and $m_2$ are the masses of the wheel and the link respectively, $l$ is the link length, $g$ is gravitational constant, $R$ is the wheel radius, $I_2$ is the link inertia, $T$ is the torque input to the wheel, and $\theta$ is the unknown parameter that we shall handle using the adaptive control. A more elaborate detail of the dynamics can be found in the markdown "SegWay Dynamics.md".


# Dynamics Linearization
Since the segway is intended to operate around an equilibrium position, it is reasonable to linearize the dynamics about the point $\phi = 0$, $\dot{\phi} \approx0$, which lead to the dynamics

$$ M \ddot{x}+C\ddot{\phi}=\frac{T}{R}$$
$$ I \ddot{\phi}+C\ddot{x}  = -T-\theta\dot{\phi}-Cg\phi$$

Matrix Representation
$$
\begin{bmatrix}
M&C\\
C&I\\
\end{bmatrix} 
\begin{bmatrix}
\ddot{x}\\
\ddot{\phi}\\
\end{bmatrix}
= \begin{bmatrix}
\frac{T}{R} \\ 
-T + Cg \phi - \theta \dot{\phi}
\end{bmatrix}
$$

Now we define a variable $s$ which we call the sliding surface. This will help us define all the states in a single variable and help implementation of the Lyapunov method.
$$ s = k_1x + k_2\dot{x} + k_3\phi + k_4\dot{\phi} $$

Here, $k_1$, $k_2$, $k_3$, $k_4$ are tuneable parameters.

Our Goal is to drive s to zero so that all the states are driven to zero.

we take the derivative of s:
$$ \dot{s} = k_1\dot{x} + k_2\ddot{x} + k_3\dot{\phi} + k_4\ddot{\phi} $$

the derivative of s can be defined in a simpler form by isolating the terms $T$, $\phi$ and $\theta\dot{\phi}$.
$$ \dot{s} = k_1\dot{x} + k_3\dot{\phi} + aT + b \phi + c \theta\dot{\phi} $$
where
$$a = \frac{1}{MI - C^2} {k_2{\frac{I}{R}+C}- k_4{\frac{C}{R}+M}}$$
$$b = \frac{Cg}{MI - C^2} {-k_2C+ k_4M}$$
$$b = \frac{1}{MI - C^2} {k_2C- k_4M}$$

## Lyapunov Formulation

we define a Lyapunov function:

$$L = \frac{1}{2} s^2 + \frac{1}{2} (\hat{\theta} - \theta)^2$$

The derivative of this Lyapunov function is:

$$\dot{L} = s\dot{s} +(\hat{\theta} - \theta)\dot{\hat{\theta}}$$

Now Substituting $\dot{s}$ and grouping the terms involving $\theta$ together,
here we treat $\dot{s}$ differently by relpacing $\theta$ as an error term of the estimated $\hat{\theta}$ and $\theta$:
$\dot{s} = k_1\dot{x} + k_3\dot{\phi} + aT + b \phi + c {\theta - \dot{\hat{\theta}}}\dot{\phi}$
$$\dot{L} = s{k_1\dot{x} + k_3\dot{\phi} + aT + b \phi + c \hat{\theta}\dot{\phi}} +sc(\theta - \hat{\theta}) + (\theta- \hat{\theta})\dot{\hat{\theta}}$$
Now We want the part of our lyapunov derivative that has the state $s$ and control action $T$ to be $\leq-ps$, where $\p$ is a design parameter, this will ensure that our control action keeps the lyapunov derivative strictly negative and thus ensuring a stable system.
we choose the control action T as:

$$ T = \frac{-ps + k_1\dot{x} + k_3 \dot{\phi} + b \phi + c\hat{\theta} \dot{\phi}}{a}$$

Substituting this T into $\dot{L}$ we get:
$$\dot{L} = -ps^2 +sc(\theta - \hat{\theta}) + (\theta- \hat{\theta})\dot{\hat{\theta}}$$

To get rid of the unknown $\theta$ term we propose an update law for $hat{\theta}$ as:

$$ \dot{\hat{\theta}} = -sc\dot{\phi}$$

This cancels the terms incluing $\theta$ and leaves us with a Lyapunov function derivative:

$$\dot{L} = -ps^2$$, which is strictly negative and satisfies our goal of driving the states to zero.

