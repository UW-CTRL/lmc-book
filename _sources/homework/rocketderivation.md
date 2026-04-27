# Rocket dynamics derivation


To derive the equations of motion, we consider the Euler-Lagrange equations. 


$$ \frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{q}}\right) - \frac{\partial \mathcal{L}}{\partial q} = \sum_i Q_i $$

where $\mathcal{L} = T - V$. 

$T$ is the kinetic energy, $V$ is the potential energy, $q$ is the generalized coordinates, and $Q_i$ represents non-conservative generalized forces. The RHS is where you include forces that cannot be derived from a potential energy function, such as friction, air resistance, or the active thrust and control torques applied to your rocket. If the system is purely conservative, this side of the equation is set to zero.


The position of the thruster is $\mathbf{x}_\text{t} = [p_x, p_z]^T$, and the position of the center of mass of the rocket body is $\mathbf{x}_\text{b} = [p_x + \frac{\ell}{2}\sin\theta, p_z + \frac{\ell}{2}\cos\theta]^T$. 

Below, we present the derivation of the equations of motion of the system. 

See the [notes](https://underactuated.mit.edu/multibody.html) here for additional references regarding Euler-Lagrange dynamics and deriving equations of motion of multi-body systems.

## Kinetic energy

The *translational* kinetic energy is $T_\text{trans} = \frac{1}{2}m_\text{t}\|\dot{\mathbf{x}}_\text{t}\|^2 + \frac{1}{2}m_\text{b}\|\dot{\mathbf{x}}_\text{b}\|^2$

$$
T_\text{trans} = \frac{1}{2}m_\text{t}(\dot{p}_x^2 + \dot{p}_z^2) + \frac{1}{2}m_\text{b}(\dot{p}_x^2 + \dot{p}_z^2 + \ell\dot{\theta}(\dot{p}_x\cos\theta - \dot{p}_z\sin\theta) + \frac{1}{4}\ell^2\dot{\theta}^2) 
$$

The *rotational* kinetic energy $T_\text{rot}= \frac{1}{2}I\dot{\theta}^2$ where $I$ is the moment of inertia of a rod rotating about its center of mass.

$$
T_\text{rot} = \frac{1}{2} \left(\frac{1}{12}\right)m_\text{b}\ell^2\dot{\theta}^2
$$

Thus the total kinetic energy is:

$$
T = \frac{1}{2}(m_\text{t} + m_\text{b})(\dot{p}_x^2 + \dot{p}_z^2) + \frac{1}{2}m_\text{b}\ell\dot{\theta}(\dot{p}_x\cos\theta - \dot{p}_z\sin\theta) + \frac{1}{6}m_\text{b}\ell^2\dot{\theta}^2
$$


## Potential energy

Only gravitational potential energy is acting on the system as there are no elastic component like springs. 
Let $z=0$ correspond to zero gravitational potential energy. 

The potential energy of the thruster is

$$ V_\text{t} = m_\text{t}gp_z$$

The potential energy of the rocket body is

$$ V_\text{body} = m_\text{b}g(p_z + \frac{\ell}{2}\cos\theta)$$

Thus the total potential energy is
$$
V = (m_\text{t} + m_\text{b})gp_z + \frac{1}{2}m_\text{b}g\ell\cos\theta
$$

## Applying the Euler-Lagrange equations

Given the Lagrangian $\mathcal{L}$, and the Euler-Lagrange equation,

$$
\frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{q}_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i,
$$

where $q = [p_x, p_z, \theta]^\top$ and $Q_i$ are the generalized forces/torques, write explicitly the equations for each coordinate.

The Lagrangian is

$$
\mathcal{L} = \frac{1}{2}(m_\text{t} + m_\text{b})(\dot{p}_x^2 + \dot{p}_z^2) + \frac{1}{2}m_\text{b}\ell\dot{\theta}(\dot{p}_x\cos\theta - \dot{p}_z\sin\theta) + \frac{1}{6}m_\text{b}\ell^2\dot{\theta}^2 - (m_\text{t} + m_\text{b})g p_z - \frac{1}{2}m_\text{b}g\ell\cos\theta
$$

For each generalized coordinate:

1. For $p_x$:

$$
\frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{p}_x} \right) - \frac{\partial \mathcal{L}}{\partial p_x} = Q_{p_x}
$$

Compute the terms:

$$
\begin{align*}
\frac{\partial \mathcal{L}}{\partial \dot{p}_x} &= (m_t + m_b)\dot{p}_x + \frac{1}{2}m_b\ell\dot{\theta}\cos\theta \\
\frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{p}_x} \right)
  &= (m_t + m_b)\ddot{p}_x + \frac{1}{2}m_b\ell\left( \ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta \right) \\
\frac{\partial \mathcal{L}}{\partial p_x} &= 0 \\
\therefore &\quad (m_t + m_b)\ddot{p}_x + \frac{1}{2}m_b\ell\left( \ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta \right) = Q_{p_x}
\end{align*}
$$

2. For $p_z$:

$$
\frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{p}_z} \right) - \frac{\partial \mathcal{L}}{\partial p_z} = Q_{p_z}
$$

Compute the terms:

$$
\begin{align*}
\frac{\partial \mathcal{L}}{\partial \dot{p}_z} &= (m_t + m_b)\dot{p}_z - \frac{1}{2}m_b\ell\dot{\theta}\sin\theta \\
\frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{p}_z} \right)
  &= (m_t + m_b)\ddot{p}_z - \frac{1}{2}m_b\ell \left( \ddot{\theta}\sin\theta + \dot{\theta}^2\cos\theta \right) \\
\frac{\partial \mathcal{L}}{\partial p_z} &= - (m_t + m_b)g \\
\therefore & \quad (m_t + m_b)\ddot{p}_z - \frac{1}{2}m_b\ell\left( \ddot{\theta}\sin\theta + \dot{\theta}^2\cos\theta \right) + (m_t + m_b)g = Q_{p_z}
\end{align*}
$$

3. For $\theta$:

$$
\frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) - \frac{\partial \mathcal{L}}{\partial \theta} = Q_{\theta}
$$

Compute the terms:

$$
\begin{align*}
\frac{\partial \mathcal{L}}{\partial \dot{\theta}} &= \frac{1}{2}m_b\ell(\dot{p}_x\cos\theta - \dot{p}_z\sin\theta) + \frac{1}{3}m_b\ell^2\dot{\theta} \\
\frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) &= \frac{1}{2}m_b\ell\left( \ddot{p}_x\cos\theta - \dot{p}_x\dot{\theta}\sin\theta - \ddot{p}_z\sin\theta - \dot{p}_z\dot{\theta}\cos\theta \right) + \frac{1}{3}m_b\ell^2\ddot{\theta} \\
\frac{\partial \mathcal{L}}{\partial \theta} &= -\frac{1}{2}m_b\ell\dot{\theta}(\dot{p}_x\sin\theta + \dot{p}_z\cos\theta) + \frac{1}{2}m_bg\ell\sin\theta \\
\end{align*}
$$

Therefore:

$$
\frac{1}{2}m_b\ell\left( \ddot{p}_x\cos\theta - \ddot{p}_z\sin\theta - \dot{p}_x\dot{\theta}\sin\theta - \dot{p}_z\dot{\theta}\cos\theta \right)
+ \frac{1}{3}m_b\ell^2\ddot{\theta}
+ \frac{1}{2}m_b\ell\dot{\theta}(\dot{p}_x\sin\theta + \dot{p}_z\cos\theta)
- \frac{1}{2}m_b g\ell\sin\theta
= Q_\theta
$$

Or collected:

$$
\frac{1}{2}m_b\ell\left( \ddot{p}_x\cos\theta - \ddot{p}_z\sin\theta \right)
+ \frac{1}{3}m_b\ell^2\ddot{\theta}
- \frac{1}{2}m_b g\ell\sin\theta
= Q_\theta
$$

(since the terms $-\dot{p}_x\dot{\theta}\sin\theta - \dot{p}_z\dot{\theta}\cos\theta$ cancel with the time-derivative of $\partial\mathcal{L}/\partial\theta$.)

Therefore, the explicit equations of motion for the three generalized coordinates are:

$$
\begin{align*}
(m_t + m_b)\ddot{p}_x + \frac{1}{2}m_b\ell\left( \ddot{\theta}\cos\theta - \dot{\theta}^2 \sin\theta \right) &= Q_{p_x} \\
(m_t + m_b)\ddot{p}_z - \frac{1}{2}m_b\ell\left( \ddot{\theta}\sin\theta + \dot{\theta}^2\cos\theta \right) + (m_t + m_b)g &= Q_{p_z} \\
\frac{1}{2}m_b\ell\left( \ddot{p}_x\cos\theta - \ddot{p}_z\sin\theta \right)
+ \frac{1}{3}m_b\ell^2\ddot{\theta}
- \frac{1}{2}m_b g\ell\sin\theta
&= Q_\theta
\end{align*}
$$


## Rearrange into state-space dynamics form

Now, with these equations of motion, we need to rearrange the terms to obtain state space dynamics, for some definition of a state vector $\mathbf{x} \in \mathbb{R}^n$.

First, notice that the equations of motion just derived can be rewritten in the standard second order form:

$$
\mathbf{M}(q) \ddot{q} + \mathbf{C}(q, \dot{q}) \dot{q} + \mathbf{g}(q) = \mathbf{Q}
$$
where the generalized coordinates are $q = \begin{bmatrix} p_x \\ p_z \\ \theta \end{bmatrix}$.

Rearranging the equations of motion further...
\begin{align*}
(m_t + m_b)\ddot{p}_x + \frac{1}{2} m_b \ell \cos\theta\, \ddot{\theta} - \frac{1}{2} m_b \ell \sin\theta\, \dot{\theta}^2 &= Q_{p_x} \\
(m_t + m_b)\ddot{p}_z - \frac{1}{2} m_b \ell \sin\theta\, \ddot{\theta} - \frac{1}{2} m_b \ell \cos\theta\, \dot{\theta}^2 + (m_t + m_b)g &= Q_{p_z} \\
\frac{1}{2} m_b \ell \cos\theta\, \ddot{p}_x - \frac{1}{2} m_b \ell \sin\theta\, \ddot{p}_z + \frac{1}{3} m_b \ell^2 \ddot{\theta} - \frac{1}{2} m_b g \ell \sin\theta &= Q_\theta
\end{align*}

We see that:

$$
\mathbf{M}(q) = \begin{bmatrix}
m_t + m_b & 0 & \frac{1}{2} m_b \ell \cos \theta \\
0 & m_t + m_b & -\frac{1}{2} m_b \ell \sin \theta \\
\frac{1}{2} m_b \ell \cos\theta & -\frac{1}{2} m_b \ell \sin\theta & \frac{1}{3} m_b \ell^2
\end{bmatrix}
$$

$$
\mathbf{C}(q, \dot{q}) =
\begin{bmatrix}
0 & 0 & -\frac{1}{2} m_b \ell \dot{\theta} \sin\theta \\
0 & 0 & -\frac{1}{2} m_b \ell \dot{\theta} \cos\theta \\
0 & 0 & 0
\end{bmatrix}
$$

$$
\mathbf{g}(q) = \begin{bmatrix}
0 \\
(m_t + m_b)g \\
-\frac{1}{2} m_b g \ell \sin\theta
\end{bmatrix}
$$

and $\mathbf{Q} = \begin{bmatrix} Q_{p_x} \\ Q_{p_z} \\ Q_\theta \end{bmatrix}$ collects the generalized forces.



## Generalized forces

Let us now determine the generalized forces acting on the rocket system.
Suppose now the rocket is equipped with a main thrust at the base providing both $x$ and $z$ thrust, and a single secondary thruster at the rocket nose providing only $x$-directed thrust, $T_{\text{nose},x}$.

The available actuator inputs form the control input vector:

$$
u = \begin{bmatrix}
T_{\text{base},x} \\
T_{\text{base},z} \\
T_{\text{nose},x}
\end{bmatrix}
$$

The generalized forces acting on the coordinates $p_x$, $p_z$, and $\theta$ can be written as:

$$
\mathbf{Q} = 
\begin{bmatrix}
Q_{p_x} \\
Q_{p_z} \\
Q_\theta
\end{bmatrix}
$$

where

$$
\begin{align*}
Q_{p_x} &= T_{\text{base},x} + T_{\text{nose},x} \\
Q_{p_z} &= T_{\text{base},z} \\
Q_\theta &= T_{\text{nose},x} (\ell \cos \theta)
\end{align*}
$$

Therefore, in vector form:

$$
\mathbf{Q} =
\begin{bmatrix}
T_{\text{base},x} + T_{\text{nose},x} \\
T_{\text{base},z} \\
\ell\, T_{\text{nose},x} \cos \theta
\end{bmatrix}
$$

Or, equivalently, using a selection matrix $\mathbf{B}$:

$$
\mathbf{Q} = \mathbf{B} u
$$

with

$$
\mathbf{B} =
\begin{bmatrix}
1 & 0 & 1 \\
0 & 1 & 0 \\
0 & 0 & \ell \cos\theta
\end{bmatrix}
$$

## First-order state space dynamics

To write the second-order equations above in first-order state space form. Let us define $q = [p_x, p_z, \theta]^T$ and $\dot{q} = [\dot{p}_x, \dot{p}_z, \dot{\theta}]^T$.
Then we can define the state vector as

$$
x =
\begin{bmatrix}
p_x \\ p_z \\ \theta \\ \dot{p}_x \\ \dot{p}_z \\ \dot{\theta}
\end{bmatrix} = \begin{bmatrix}
q \\ \dot{q}
\end{bmatrix}
$$

The original equations of motion are:
$$
\mathbf{M}(q)\ddot{q} + \mathbf{C}(q,\dot{q})\dot{q} + \mathbf{g}(q) = \mathbf{B}u
$$


Writing this as a first-order system:

$$
\dot{x} = 
\begin{bmatrix}
\dot{p}_x \\ \dot{p}_z \\ \dot{\theta} \\ \ddot{p}_x \\ \ddot{p}_z \\ \ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\dot{q} \\
\mathbf{M}(q)^{-1} \left[ \mathbf{B}u - \mathbf{C}(q,\dot{q})\dot{q} - \mathbf{g}(q) \right]
\end{bmatrix}
$$

Thus, the first-order state-space equations are:

$$
\boxed{
\dot{x} = f(x, u) =
\begin{bmatrix}
\dot{q} \\
\mathbf{M}(q)^{-1}
\left[ \mathbf{B}u - \mathbf{C}(q, \dot{q})\dot{q} - \mathbf{g}(q) \right]
\end{bmatrix}
}
$$

where $x_1 = p_x$, $x_2 = p_z$, $x_3 = \theta$, $x_4 = \dot{p}_x$, $x_5 = \dot{p}_z$, $x_6 = \dot{\theta}$.


