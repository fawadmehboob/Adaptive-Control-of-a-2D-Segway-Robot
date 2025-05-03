Segway Robot Dynamics via Lagrange Approach
This document provides a detailed derivation of the dynamics of a 2D Segway robot using the Lagrangian mechanics approach. The Segway is modeled as a two-wheeled inverted pendulum with a wheel assembly and a body pivoting above the wheel axle. The equations of motion are derived using the Lagrange method, which involves computing the kinetic and potential energies, forming the Lagrangian, and applying the Euler-Lagrange equations.
System Description
The Segway robot consists of:

Wheels: A combined wheel assembly with mass ( m_w ), radius ( r ), and moment of inertia ( I_w ).
Body (Link): A rigid body with mass ( m_b ), length ( L ), and center of mass at a distance ( l = L/2 ) from the pivot (wheel axle). The moment of inertia of the body about its center of mass is ( I_b ).
Coordinates: Generalized coordinates ( x ) (horizontal position of the wheel axle) and ( \theta ) (angle of the body from the vertical, where ( \theta = 0 ) is upright).
External Torque: A torque ( \tau ) is applied to the wheels.

Parameters

( m_w = 1.0 , \text{kg} ): Mass of the wheel assembly.
( m_b = 5.0 , \text{kg} ): Mass of the body.
( r = 0.1 , \text{m} ): Radius of the wheel.
( L = 1.0 , \text{m} ): Length of the body.
( l = L/2 = 0.5 , \text{m} ): Distance from the pivot to the body’s center of mass.
( I_w = \frac{1}{2} m_w r^2 ): Moment of inertia of the wheel.
( I_b = \frac{1}{12} m_b L^2 ): Moment of inertia of the body about its center of mass.
( g = 9.81 , \text{m/s}^2 ): Gravitational acceleration.
( \tau ): Torque applied to the wheels.

Lagrange Approach
The Lagrange method involves the following steps:

Compute the kinetic energy ( T ) of the system.
Compute the potential energy ( V ) of the system.
Form the Lagrangian ( \mathcal{L} = T - V ).
Apply the Euler-Lagrange equations to derive the equations of motion:[\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}_i} \right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i]where ( q_i ) are the generalized coordinates (( x ), ( \theta )), ( \dot{q}_i ) are their time derivatives, and ( Q_i ) are the generalized forces.

Step 1: Kinetic Energy ( T )
The kinetic energy consists of contributions from the wheel and the body.
Wheel Kinetic Energy
The wheel has translational and rotational motion:

Translational: The wheel’s center moves with velocity ( \dot{x} ).
Rotational: The wheel rotates with angular velocity ( \dot{\phi} = \dot{x}/r ).

Translational kinetic energy of the wheel:[T_{w,\text{trans}} = \frac{1}{2} m_w \dot{x}^2]
Rotational kinetic energy of the wheel:[T_{w,\text{rot}} = \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2]
Total wheel kinetic energy:[T_w = \frac{1}{2} m_w \dot{x}^2 + \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2]
Body Kinetic Energy
The body has translational and rotational motion:

Translational: The body’s center of mass moves due to the wheel’s motion and the body’s rotation.
Rotational: The body rotates about the pivot with angular velocity ( \dot{\theta} ).

The position of the body’s center of mass (relative to a fixed origin) is:[x_{\text{cm}} = x + l \sin \theta, \quad y_{\text{cm}} = r + l \cos \theta]
The velocities are:[\dot{x}{\text{cm}} = \dot{x} + l \dot{\theta} \cos \theta, \quad \dot{y}{\text{cm}} = -l \dot{\theta} \sin \theta]
Translational kinetic energy of the body:[T_{b,\text{trans}} = \frac{1}{2} m_b (\dot{x}{\text{cm}}^2 + \dot{y}{\text{cm}}^2) = \frac{1}{2} m_b \left[ (\dot{x} + l \dot{\theta} \cos \theta)^2 + (-l \dot{\theta} \sin \theta)^2 \right]][= \frac{1}{2} m_b \left[ \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \cos^2 \theta + l^2 \dot{\theta}^2 \sin^2 \theta \right]][= \frac{1}{2} m_b \left[ \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \right]]
Rotational kinetic energy of the body (about its center of mass):[T_{b,\text{rot}} = \frac{1}{2} I_b \dot{\theta}^2]
Total body kinetic energy:[T_b = \frac{1}{2} m_b \left[ \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \right] + \frac{1}{2} I_b \dot{\theta}^2]
Total Kinetic Energy
[T = T_w + T_b = \frac{1}{2} m_w \dot{x}^2 + \frac{1}{2} I_w \left( \frac{\dot{x}}{r} \right)^2 + \frac{1}{2} m_b \left[ \dot{x}^2 + 2 \dot{x} l \dot{\theta} \cos \theta + l^2 \dot{\theta}^2 \right] + \frac{1}{2} I_b \dot{\theta}^2][T = \frac{1}{2} \left( m_w + m_b + \frac{I_w}{r^2} \right) \dot{x}^2 + m_b l \dot{x} \dot{\theta} \cos \theta + \frac{1}{2} \left( m_b l^2 + I_b \right) \dot{\theta}^2]
Step 2: Potential Energy ( V )
The potential energy is due to the body’s center of mass in the gravitational field. The height of the center of mass (taking ( y = 0 ) at the ground):[y_{\text{cm}} = r + l \cos \theta]
Potential energy:[V = m_b g (r + l \cos \theta)]
Since ( r ) is constant, it does not affect the dynamics:[V = m_b g l \cos \theta]
Step 3: Form the Lagrangian
[\mathcal{L} = T - V][\mathcal{L} = \frac{1}{2} \left( m_w + m_b + \frac{I_w}{r^2} \right) \dot{x}^2 + m_b l \dot{x} \dot{\theta} \cos \theta + \frac{1}{2} \left( m_b l^2 + I_b \right) \dot{\theta}^2 - m_b g l \cos \theta]
Step 4: Euler-Lagrange Equations
For ( x ):
[\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{x}} \right) - \frac{\partial \mathcal{L}}{\partial x} = Q_x]

( \frac{\partial \mathcal{L}}{\partial \dot{x}} = \left( m_w + m_b + \frac{I_w}{r^2} \right) \dot{x} + m_b l \dot{\theta} \cos \theta )
( \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{x}} \right) = \left( m_w + m_b + \frac{I_w}{r^2} \right) \ddot{x} + m_b l \ddot{\theta} \cos \theta - m_b l \dot{\theta}^2 \sin \theta )
( \frac{\partial \mathcal{L}}{\partial x} = 0 )
( Q_x = \frac{\tau}{r} ) (generalized force due to torque)

[\left( m_w + m_b + \frac{I_w}{r^2} \right) \ddot{x} + m_b l \ddot{\theta} \cos \theta - m_b l \dot{\theta}^2 \sin \theta = \frac{\tau}{r}]
For ( \theta ):
[\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) - \frac{\partial \mathcal{L}}{\partial \theta} = Q_\theta]

( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} = m_b l \dot{x} \cos \theta + \left( m_b l^2 + I_b \right) \dot{\theta} )
( \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) = m_b l \ddot{x} \cos \theta - m_b l \dot{x} \dot{\theta} \sin \theta + \left( m_b l^2 + I_b \right) \ddot{\theta} )
( \frac{\partial \mathcal{L}}{\partial \theta} = -m_b l \dot{x} \dot{\theta} \sin \theta + m_b g l \sin \theta )
( Q_\theta = -\tau ) (torque acts oppositely on the body)

[m_b l \ddot{x} \cos \theta + \left( m_b l^2 + I_b \right) \ddot{\theta} + m_b g l \sin \theta = -\tau]
Equations of Motion in Matrix Form
Rewrite the EOM in matrix form:[\begin{bmatrix}\left( m_w + m_b + \frac{I_w}{r^2} \right) & m_b l \cos \theta \m_b l \cos \theta & m_b l^2 + I_b\end{bmatrix}\begin{bmatrix}\ddot{x} \\ddot{\theta}\end{bmatrix}+\begin{bmatrix}

m_b l \dot{\theta}^2 \sin \theta \m_b g l \sin \theta\end{bmatrix}
\begin{bmatrix}\frac{\tau}{r} \-\tau\end{bmatrix}]

Adjusting for the code’s form (where ( \ddot{x}/r ) is used):[\begin{bmatrix}\left( m_w + m_b \right) r^2 + I_w & m_b r l \cos \theta \m_b r l \cos \theta & m_b l^2 + I_b\end{bmatrix}\begin{bmatrix}\frac{\ddot{x}}{r} \\ddot{\theta}\end{bmatrix}+\begin{bmatrix}

m_b r l \dot{\theta}^2 \sin \theta \m_b g l \sin \theta\end{bmatrix}
\begin{bmatrix}\tau \0\end{bmatrix}]

Conclusion
The Lagrangian approach provides a systematic method to derive the nonlinear dynamics of the Segway robot, capturing the coupling between the wheel’s motion (( x )) and the body’s angle (( \theta )). These equations form the basis for implementing control strategies, such as the adaptive control approach in the associated code.
