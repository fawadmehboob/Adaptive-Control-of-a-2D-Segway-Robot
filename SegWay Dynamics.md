Segway Robot Dynamics via Lagrange Approach
This document provides a detailed derivation of the dynamics of a Segway robot using the Lagrangian mechanics approach. The Segway is modeled as a two-wheeled inverted pendulum, with a wheel assembly and a body (link) pivoting above the wheel axle. We derive the equations of motion (EOM) using the Lagrange method, which involves computing the kinetic and potential energies, forming the Lagrangian, and applying the Euler-Lagrange equations.
System Description
The Segway robot consists of:

Wheels: Two wheels approximated as a single wheel with mass ( m_w ), radius ( r ), and moment of inertia ( I_w ).
Body (Link): A rigid body with mass ( m_b ), length ( L ), and center of mass at a distance ( l = L/2 ) from the pivot (wheel axle). The moment of inertia of the body about its center of mass is ( I_b ).
Coordinates: We use generalized coordinates ( x ) (horizontal position of the wheel axle) and ( \theta ) (angle of the body from the vertical, where ( \theta = 0 ) is upright).
External Torque: A torque ( \tau ) is applied to the wheels to control the system.

Parameters

( m_w ): Mass of the wheel assembly (( 1.0 , \text{kg} ))
( m_b ): Mass of the body (( 5.0 , \text{kg} ))
( r ): Radius of the wheel (( 0.1 , \text{m} ))
( L ): Length of the body (( 1.0 , \text{m} ))
( l ): Distance from the pivot to the body’s center of mass (( l = L/2 = 0.5 , \text{m} ))
( I_w ): Moment of inertia of the wheel (( I_w = \frac{1}{2} m_w r^2 ))
( I_b ): Moment of inertia of the body about its center of mass (( I_b = \frac{1}{12} m_b L^2 ))
( g ): Gravitational acceleration (( 9.81 , \text{m/s}^2 ))
( \tau ): Torque applied to the wheels

Lagrange Approach
The Lagrange method involves:

Compute the kinetic energy ( T ) of the system.
Compute the potential energy ( V ) of the system.
Form the Lagrangian ( \mathcal{L} = T - V ).
Apply the Euler-Lagrange equations to derive the equations of motion:[\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}_i} \right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i]where ( q_i ) are the generalized coordinates (( x ), ( \theta )), ( \dot{q}_i ) are their time derivatives, and ( Q_i ) are the generalized forces.

Step 1: Kinetic Energy ( T )
The kinetic energy consists of contributions from the wheel and the body.
Wheel Kinetic Energy
The wheel has both translational and rotational motion:

Translational: The wheel’s center moves with velocity ( \dot{x} ).
Rotational: The wheel rotates with angular velocity ( \dot{x}/r ) (since ( \dot{x} = r \dot{\phi} ), where ( \phi ) is the wheel’s rotation angle).

Translational kinetic energy of the wheel:[T_{w,\text{trans}} = \frac{1}{2} m_w \dot{x}^2]
Rotational kinetic energy of the wheel:[T_{w,\text{rot}} = \frac{DIT: The wheel’s angular velocity is ( \dot{\phi} = \dot{x}/r ), so:[T_{w,\text{rot}} = \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2]
Total wheel kinetic energy:[T_w = T_{w,\text{trans}} + T_{w,\text{rot}} = \frac{1}{2} m_w \dot{x}^2 + \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2]
Body Kinetic Energy
The body has both translational and rotational motion:

Translational: The body’s center of mass moves due to the motion of the wheel and the rotation of the body.
Rotational: The body rotates about the pivot with angular velocity ( \dot{\theta} ).

The position of the body’s center of mass (relative to a fixed origin) is:[x_{\text{cm}} = x + l \sin \theta, \quad y_{\text{cm}} = r + l \cos \theta]
The velocities are:[\dot{x}{\text{cm}} = \dot{x} + l \dot{\theta} \cos \theta, \quad \dot{y}{\text{cm}} = -l \dot{\theta} \sin \theta]
The translational kinetic energy of the body:[T_{b,\text{trans}} = \frac{1}{2} m_b (\dot{x}{\text{cm}}^2 + \dot{y}{\text{cm}}^2) = \frac{1}{2} m_b \left( (\dot{x} + l \dot{\theta} \cos \theta)^2 + (-l \dot{\theta} \sin \theta)^2 \right)]
Simplify:[T_{b,\text{trans}} = \frac{1}{2} m_b \left( \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \cos^2 \theta + l^2 \dot{\theta}^2 \sin^2 \theta \right)][= \frac{1}{2} m_b \left( \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \right)]
The rotational kinetic energy of the body about its center of mass:[T_{b,\text{rot}} = \frac{1}{2} I_b \dot{\theta}^2]
Total body kinetic energy:[T_b = T_{b,\text{trans}} + T_{b,\text{rot}} = \frac{1}{2} m_b \left( \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \right) + \frac{1}{2} I_b \dot{\theta}^2]
Total Kinetic Energy
[T = T_w + T_b = \frac{1}{2} m_w \dot{x}^2 + \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2 + \frac{1}{2} m_b \left( \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \right) + \frac{1}{2} I_b \dot{\theta}^2]
Step 2: Potential Energy ( V )
The potential energy is due to the body’s center of mass in the gravitational field. The height of the body’s center of mass above the ground (taking ( y = 0 ) at the ground):[y_{\text{cm}} = r + l \cos \theta]
Potential energy:[V = m_b g (r + l \cos \theta)]
Since ( r ) is constant, it does not affect the dynamics, so we can simplify:[V = m_b g l \cos \theta]
Step 3: Form the Lagrangian
[\mathcal{L} = T - V][\mathcal{L} = \frac{1}{2} (m_w + m_b) \dot{x}^2 + \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2 + m_b l \dot{x} \dot{\theta} \cos \theta + \frac{1}{2} (m_b l^2 + I_b) \dot{\theta}^2 - m_b g l \cos \theta]
Step 4: Euler-Lagrange Equations
For ( x ):
[\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{x}} \right) - \frac{\partial \mathcal{L}}{\partial x} = Q_x]

( \frac{\partial \mathcal{L}}{\partial \dot{x}} = (m_w + m_b) \dot{x} + \frac{I_w}{r^2} \dot{x} + m_b l \dot{\theta} \cos \theta )
( \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{x}} \right) = (m_w + m_b + \frac{I_w}{r^2}) \ddot{x} + m_b l \ddot{\theta} \cos \theta - m_b l \dot{\theta}^2 \sin \theta )
( \frac{\partial \mathcal{L}}{\partial x} = 0 )
( Q_x = \tau / r ) (generalized force due to torque)

[(m_w + m_b + \frac{I_w}{r^2}) \ddot{x} + m_b l \ddot{\theta} \cos \theta - m_b l \dot{\theta}^2 \sin \theta = \frac{\tau}{r}]
For ( \theta ):
[\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) - \frac{\partial \mathcal{L}}{\partial \theta} = Q_\theta]

( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} = m_b l \dot{x} \cos \theta + (m_b l^2 + I_b) \dot{\theta} )
( \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) = m_b l \ddot{x} \cos \theta - m_b l \dot{x} \dot{\theta} \sin \theta + (m_b l^2 + I_b) \ddot{\theta} )
( \frac{\partial \mathcal{L}}{\partial \theta} = -m_b l \dot{x} \dot{\theta} \sin \theta + m_b g l \sin \theta )
( Q_\theta = -\tau ) (torque acts oppositely on the body)

[m_b l \ddot{x} \cos \theta + (m_b l^2 + I_b) \ddot{\theta} + m_b g l \sin \theta = -\tau]
Equations of Motion
Rewrite the EOM in matrix form:[\begin{bmatrix}(m_w + m_b) r^2 + I_w & m_b r l \cos \theta \m_b r l \cos \theta & m_b l^2 + I_b\end{bmatrix}\begin{bmatrix}\ddot{x}/r \\ddot{\theta}\end{bmatrix}+\begin{bmatrix}-m_b r l \dot{\theta}^2 \sin \theta \m_b g l \sin \theta\end{bmatrix}
\begin{bmatrix}\tau \0\end{bmatrix}]
This matches the form implemented in the code:[M \begin{bmatrix} \ddot{x}/r \ \ddot{\theta} \end{bmatrix} + \begin{bmatrix} C \ G \end{bmatrix} = B \tau]
Where:

( M_{11} = (m_w + m_b) r^2 + I_w )
( M_{12} = M_{21} = m_b r l \cos \theta )
( M_{22} = m_b l^2 + I_b )
( C = -m_b r l \dot{\theta}^2 \sin \theta )
( G = m_b g l \sin \theta )
( B = \begin{bmatrix} 1 \ 0 \end{bmatrix} )

The code then solves for the accelerations and integrates them to obtain the state trajectories.
Conclusion
The Lagrangian approach provides a systematic way to derive the dynamics of the Segway robot. The resulting equations of motion capture the nonlinear coupling between the wheel’s motion (( x )) and the body’s angle (( \theta )), which is essential for designing control strategies, such as the adaptive control implemented in the associated code.

