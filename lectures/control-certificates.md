# Control certificate functions

In this section, we describe control synthesis techniques for ensuring the stability and safety of dynamical systems. In particular, we consider using scalar functions that serve as *certificates* for certain properties of a dynamical system under control.
Namely, we consider:

- **Control Lyapunov Functions** (CLF) to certify stabilizability
- **Control Barrier Functions** (CBF) to certify safety

The term *certificate* is used in the literature, particularly in formal methods and control theory contexts, where such functions are interpreted as evidence that a control policy exists to guarantee a certain property.

First, we recap Lyapunov theory and stability, then extend the ideas to *control* Lyapunov theory, and then show that a natural extension to Lyapunov theory is *barrier* functions and *control* barrier functions (CBFs).
Then we show that for control affine dynamics, synthesizing a controller that satisfies the CLF and CBF constraints results in a quadratic program, which is a convex optimization problem that is straightforward to solve.

## Types of stability

Consider an LTI system $\dot{x} = Ax$. Let $x^\mathrm{eq}$ be an equilibrium state, where $f(x^\mathrm{eq}) = 0$.
Then the equilibrium is:
- **(Lyapunov) stable**: $\forall\epsilon>0, \exists\delta>0$ such that $|x(0)-x^\mathrm{eq}|<\delta \Rightarrow |x(t)-x^\mathrm{eq}|<\epsilon, \forall t\geq0$
- **Asymptotically stable**: $\forall\epsilon>0, \exists\delta>0$, such that $|x(0)-x^\mathrm{eq}|<\delta \Rightarrow \lim_{t\to\infty}|x(t)-x^\mathrm{eq}|=0$
- **Exponentially stable**: $\forall\epsilon>0, \exists\delta>0, K>0$, such that $|x(0)-x^*|<\delta$, $|x(t)-x^*|\leq \alpha e^{-\lambda t}|x(0)-x^*|$, $\forall t\geq 0$

These above types of stability are in regard to an equilibrium state—whether or not the system remains close or converges to the equilibrium state if the initial state were to be close to it.
As such, these stability definitions describe *local* stability properties.


```{margin} Contraction theory
For the interested reader, *contraction theory* is a control certificate technique for determining the global stability of a system. Check out [this paper](https://www.sciencedirect.com/science/article/abs/pii/S0005109898000193), which introduces the idea of contraction analysis, and [this paper](https://arxiv.org/abs/1503.03144) for control feedback design using contraction metrics.
```

For *any* initial conditions, not just ones close to the equilibrium state of interest, then this refers to *global* stability, which is a much stronger condition and often harder to prove.

| **Type**                      | **Definition**                                                                 | **Notes**                             |
|-------------------------------|---------------------------------------------------------------------------------|----------------------------------------|
| **Lyapunov Stability**        | Trajectories stay close to equilibrium if started nearby                       | No guarantee of convergence            |
| **Asymptotic Stability**      | Trajectories stay close **and** converge to equilibrium                        | Includes Lyapunov stability            |
| **Exponential Stability**     | Trajectories converge to equilibrium **at an exponential rate**                | Stronger than asymptotic stability     |
| **Local Stability**           | Stability properties hold **near** the equilibrium                             | Common in nonlinear systems            |
| **Global Stability**          | Stability properties hold **for all** initial conditions                       | Often harder to prove                  |


## Checking for stability

### Linear systems

There are techniques for determining whether a system is stable.
If the system is linear (assuming no control input), that is, we have $\dot{x} = Ax$, then system is (expoentially) stable if the real parts of all the eigenvalues of $A$ are strictly negative.

```{admonition} Pause and think
Why is this the case?
```

```{admonition} Solution to $\dot{x} = Ax$
:class: dropdown
Recall that the solution to $\dot{x} = Ax$ is $x(t) = Ve^{Dt}V^{-1}x(0)$ where $A = VDV^{-1}$ is the diagonalization of $A$ and $x(0)$ is the initial condition. Whether $x(t)$ grows or decays exponentially depends on $D$, the eigenvalues. If the real parts of all eigenvalues are negative, then $e^{Dt}$ will decay.
```


```{admonition} Pause and think
What about for discrete-time linear dynamics $x_{t+1} = A x_t$?
```

[Professor Xu Chen's ME 547 notes](https://faculty.washington.edu/chx/teaching/me547/stability_eigenvalue/) provide more information regarding the stability of LTI systems.

### Lyapunov functions

For nonlinear systems, determining its stability is generally challenging. However, Lyapunov theory provides a general approach to proving the stability of a nonlinear system. At a high-level, if we can find a Lyapunov function such that a set of conditions are true, then the system is stable. The Lyapunov conditions are somewhat straightforward, and avoids the need to explicitly compute solution of system responses. The challenge is finding such a Lyapunov function. But if one can be found, then the existence of it is a certificate for stability.

```{admonition} Theorem (Lyapunov function)
Given a general nonlinear dynamical system $\dot{x} = f(x)$, $x \in \mathcal{X} \subset \mathbb{R}^n$.
Without loss of generality, let $x=0$ be an equilibrium state.
Let $V:\mathbb{R}^n \rightarrow \mathbb{R}$ be a scalar function that is continuous, and has continuous first derivatives.
Then the origin $x=0$ is Lyapunov stable if:
- $V(0) = 0$
- $V(x) > 0 \, \forall x\in\mathcal{X} \setminus \{ 0\}$
- $\dot{V}(x) = \nabla V(x)^Tf(x) \leq 0\, \forall  x\in\mathcal{X}$
```
For the last condition, if the inequality was a strict inequality, then we would have asymptotic stability. If instead the RHS was $-\alpha V(x)$, then we would have exponential stability with a decay rate of $\alpha$.

```{admonition} Exercise
In the case of linear systems $\dot{x} = Ax$, a good Lyapunov candidate is a quadratic function $V(s) = x^TPx$ where $P$ is positive definite, i.e., $P=P^T, P\succ 0$.
Show that for $V(x)$ to be a valid Lyapunov function, we must have $A^TP + PA \preceq 0$.
```

```{admonition} Theorem (Lyapunov stability theorem for linear systems)
For a linear system $\dot{x} = Ax$, the origin $x=0$ is asymptotically stable if and only if for any symmetric positive definite matrix $Q$, the Lyapunov equation

$$ A^TP + PA = -Q$$

has a unique positive definite solution $P\succ 0$, $P^T = P$.
```

Intuitively, the Lyapunov function can be interpreted as as an energy-like function of the dynamics. If such an energy-like function can be found and shown to decay over time, then the system would "run out of energy" and settle to the equilibrium point. Of course, not all dynamical systems are rooted in mechanical systems where energy has a physical meaning. But for mechanical systems, deriving the energy of the system is often a good place to start when finding a Lyapunov function candidate.



## Control Lyapunov functions (CLF)
So far, we have been considering stability for dynamical systems without input.
But what if a system is naturally unstable (i.e., zero control input), but for certain choices of control input $u$, the system can be stable. In other words, there could be values of $u$ such that the Lyapunov conditions are met.
As such, a natural extension to Lyapunov theory is *Control* Lyapunov theory. The following definition looks very similar to the definition of a Lyapunov function.


```{admonition} Theorem (Control Lyapunov function)
Given a general nonlinear dynamical system $\dot{x} = f(x, u)$, $x \in \mathcal{X} \subset \mathbb{R}^n$, $u\in\mathcal{U} \subset\mathbb{R}^m$.
Without loss of generality, let $x=0$ be an equilibrium state.
Let $V:\mathbb{R}^n \rightarrow \mathbb{R}$ be a scalar function that is continuous, and has continuous first derivatives.
Then the origin $x=0$ is Lyapunov stable if:
- $V(0) = 0$
- $V(x) > 0 \, \forall x\in\mathcal{X} \setminus \{ 0\}$
- $\min_{u\in \mathcal{U}}\nabla V(x)^Tf(x,u) \leq 0\, \forall  x\in\mathcal{X}$
```

In the last condition, the interpretation is that there exists a control $u\in\mathcal{U}$ that satisfies the inequality $\nabla V(x)^Tf(x,u) \leq 0$ for all $x$ in the domain.
Likewise as before, we can also have asumptotic and exponential stability by slightly modifying the last condition.


So if we can find a valid CLF, then we know that there is always a control $u$ that ensures that our system will be stable.



### CLF controller

Suppose you have a nominal controller for your system. It does not matter how this nominal controller is synthesized, but you know that at any time $t$ and state $x$, the nominal controller spits out $u_\mathrm{nom}$.
However, to ensure the system remains stable about the equilibrium state, or ensure that the system converges to an equilibrium state, we can adjust $u_\mathrm{nom}$ slightly to make sure that the CLF conditions are satisfies. In particular, we want to adjust $u_\mathrm{nom}$ as little as possible since $u_\mathrm{nom}$ is computed from some nominal controller that presumable encodes important objectives relevant to the task.
As such, we can frame this as an optimization problem:


```{math}
:label: eq-clf-optimization
u^\star = &\underset{u}{\text{argmin}} \: \| u - u_\mathrm{nom}\|_2^2\\
&\text{subj. to} \:\: \nabla V(x)^Tf(x,u) \leq 0
```

Note, we can change the inequality constraint slightly to reflect asymptotic and exponential stability.

For general nonlinear dynamics, the constraint would be nonlinear, potentially non-convex.
Thus making the constraint in {eq}`eq-clf-optimization` a challenging optimization problem to solve. However, for control affine systems, where the dynamics can be expressed as:

```{math}
\dot{x} = f(x) + g(x)u
```

the constraint in {eq}`eq-clf-optimization` becomes linear in $u$, making the optimization problem a quadratic program (QP). Quadratic programs are convex optimization problems and can be solved efficiently using standard solvers.

```{admonition} Example: Control Lyapunov Function for a Nonlinear System

Consider the nonlinear dynamical system:

$$
\dot{x}_1 = x_2, \quad \dot{x}_2 = -x_1 + u
$$


This system represents a simple pendulum with control input $ u $. We aim to stabilize the system at the origin $ (x_1, x_2) = (0, 0) $.

A natural choice for a Lyapunov function is the total energy of the system:

$$
V(x) = \frac{1}{2}x_1^2 + \frac{1}{2}x_2^2
$$

This function is positive definite and radially unbounded. Its derivative along the system's trajectories is:

$$
\dot{V}(x) = \nabla V(x)^T f(x, u) = x_1 \dot{x}_1 + x_2 \dot{x}_2 = x_1 x_2 + x_2(-x_1 + u)
$$

Simplifying:

$$
\dot{V}(x) = x_2(-x_1 + x_1 + u) = x_2 u
$$

To ensure $ \dot{V}(x) \leq 0 $, we choose a control input $ u $ such that:

$$
u = -kx_2, \quad k > 0
$$

Substituting this into $ \dot{V}(x) $:

$$
\dot{V}(x) = x_2(-kx_2) = -k x_2^2
$$

Since $ k > 0 $, $ \dot{V}(x) \leq 0 $, and the system is Lyapunov stable. This demonstrates that $ V(x) $ is a valid control Lyapunov function for the given system.

```



## Control Invariant Set

It turns out that the sub-level sets of a (control) Lyapunov function are forward *(control) invariant sets*, which are sets where if the system starts inside that set, it will remain inside that set for all future times.
For *control* invariant sets, there is an additional "there exists a control" clause: A set is forward control invariant if the system starts inside that set, there exists a control that will allow the system to remain inside that set for all future times.

Mathematically, $\mathcal{C} = \{ x \mid V(x) \leq c\}$ is the sub-level set thresholded by $c$, and we are claiming that $\mathcal{C}$ is forward (control) invariant.
Intuitively, since $\dot{V}(x) \leq 0$, the largest value $V$ attains is $V(x(0))$ and any future $x$, we must have $V(x) \leq V(x(0))$. Thus if we set $c=V(x(0))$, then we must have $x\in \mathcal{C}$ for all future times.

## Barrier functions

The previous discussion about stability and Lyapunov functions was centered about the idea that the system would (hopefully) eventually converge to an equilibrium state.
What if instead, rather than converge an equilibrium state, we desired the system remained inside a set. It does not matter where inside the set the system is as long as it is inside it. Roughly speaking, you can view this as stability to not a state but to a set.

We can make some small modification to the ideas surrounding Lyapunov theory to consider containment within a desired set. A natural and common choice of the desired set is the *safe set*, the set of states that is deemed safe for the system to operate in. For example, the set of states that are not in collision to obstacles.

Let $b:\mathbb{R}^n \rightarrow \mathbb{R}$ be a scalar function, and suppose we have dynamics $\dot{x} = f(x)$. Define $\mathcal{S} = \{ x \mid b(x) \geq 0\}$ as the "safe set". Then if $b(x(0)) \geq 0$ (i.e., the system is initially safe), then if $\nabla b(x)^T f(x) \geq 0$ whenever $b(x) = 0$, then we call $b$ a *barrier function*. This condition will ensure that if the system initially starts inside $\mathcal{S}$, then it will remain inside $\mathcal{S}$. Intuitively, whenever the system reaches the boundary of $\mathcal{S}$ where $b(x) = 0$, since $\nabla b(x)^T f(x) \geq 0$, then the barrier value must either increase (i.e., move away from the boundary and further back into $\mathcal{S}$) or remain at the boundary (i.e., stay there or move along the boundary of $\mathcal{S}$.) As such, $b$ acts like "barrier" that prevents the system from exiting $\mathcal{S}$.



## Control Barrier Functions (CBFs)

```{margin} Safe Autonomy with Control Barrier Functions

The textbook [Safe Autonomy with Control Barrier Functions
](https://link.springer.com/book/10.1007/978-3-031-27576-0) by Wei Xiao, Christos G. Cassandras, Calin Belta offers an introduction and overview of control barrier functions and their use in safe autonomy.
```
We can extend the idea of barrier functions to *control* barrier functions (CBFs), in a similar way we saw with Lyapunov and control Lyapunov functions.

We change the barrier condition described above slightly to account for the control input.
Let $b:\mathbb{R}^n \rightarrow \mathbb{R}$ be a scalar function, and suppose we have dynamics $\dot{x} = f(x,u)$. Define $\mathcal{S} = \{ x \mid b(x) \geq 0\}$ as the "safe set".
We see that if $\max_{u \in \mathcal{U}} \nabla b(x)^T f(x,u) \geq 0$ whenever $b(x) = 0$, then, if $b(x(0)) \geq 0$, then the system will always remain inside $\mathcal{S}$. Note that having the inequality on the right be 0 imposes the least restriction on the system---it can so absolutely anything inside $\mathcal{S}$, but at the boundary, we must have $\max_{u \in \mathcal{U}} \nabla b(x)^T f(x,u) \geq 0$.

One may want a more gradual constraint on the system as it approaches the boundary. Intuitively, the closer the system approaches the boundary, the more restrictions there should be on the motion such that when the system is at the boundary, we must have $\max_{u \in \mathcal{U}} \nabla b(x)^T f(x,u) \geq 0$, which is the condition above. This makes practical sense---when the system is far away from the boundary of $\mathcal{S}$, it should have a lot of freedom on its motion, including moving towards the boundary. But since the system is far away from the boundary, moving towards the boundary does not pose much risk. But as the system gets closer and closer to the boundary, the freedom in its motion should be reduced accordingly, for instance, the system should begin to slow down and/or steer away from the boundary.
To achieve this behavior, we introduce a different term on the RHS of the inequality. First we introduce the following definition.


```{admonition} Class $\mathcal{K}$ and extended class $\mathcal{K}$ functions
A Lipschitz continuous function $\alpha : [0, a) \rightarrow [0,\infty)$, $a > 0$ is said to belong to class $\mathcal{K}$ if it is strictly increasing and $\alpha(0) = 0$. Moreover, $\alpha : [−b, a) \rightarrow [−\infty,\infty)$, $a > 0$, $b > 0$ belongs to extended class $\mathcal{K}$ if it is strictly increasing and $\alpha(0) = 0$.
```

Intuitively, a class $\mathcal{K}$ or extended class $\mathcal{K}$ functions are monotonically increasing functions that are 0 at 0. A common example include $\alpha(x) = ax$.

Now we formally define a control barrier function

```{admonition} Definition (Control barrier function)
Given a set $\mathcal{S} = \{ x \mid b(x) \geq 0\}$ where $b:\mathbb{R}^n \rightarrow \mathbb{R}$. Then $b$ is a control barrier function for a system $\dot{x} = f(x,u)$ if there exists a class $\mathcal{K}$ function $\alpha$ such that

```{math}
:label: eq-cbf
\max_{u\in\mathcal{U}} \nabla b(x)^Tf(x,u)  \geq -\alpha(b(x)) , \quad \forall \: x\in\mathcal{S}
```
This definition is similar CLFs introduced beforehand, but instead of 0 on the RHS, we have $-\alpha(b(x))$ which provides the behavior we want where we gradually restrict the motion of the system as it approaches the boundary of $\mathcal{S}$.
The $\max$ operation there is to ensure that there exists at least one control $u\in\mathcal{U}$ that satisfies the inequality.

If we can find such a function $b$ and $\alpha$ that satisfy the CBF inequality over the domain, then we can guarantee that if the system starts inside $\mathcal{S}$, then it will remain inside $\mathcal{S}$ for all future time.

### Finding valid CBFs
In general, finding a valid CBF (and CLF for that matter) is challenging. One may construct one based on deep understanding of the system, or develop techniques to synthesize one. Later we will learn about HJ reachability and HJ reachability value functions are valid CBFs.


## CBF safety filter

The CBF discussion so far is centered about the definition, but we have yet to compute a controller that would ensure the system remains inside $\mathcal{S}$.
To ensure the system remains inside $\mathcal{S}$, we need to guarantee that the control input satisfies the inequality in {eq}`eq-cbf` at all times. Since a valid CBF ensures the existence of at least one control input that meets this condition, we can leverage this property. Simultaneously, we often have a nominal controller designed to achieve specific objectives or tasks. Therefore, we aim to balance following the nominal controller's guidance while ensuring the system's safety by staying within $\mathcal{S}$.

This trade-off can be formulated as an optimization problem:

```{math}
:label: eq-cbf-optimization
u^\star = &\underset{u}{\text{argmin}} \: \| u - u_\mathrm{nom}\|_2^2\\
&\text{subj. to} \:\: \nabla b(x)^Tf(x,u) \geq -\alpha(b(x))
```

For systems with *control affine* dynamics, this optimization problem becomes a quadratic program (QP), which is a convex optimization problem that can be solved efficiently using standard solvers.


Like what we saw with CLFs, we can pose this as an optimization problem,


```{math}
:label: eq-cbf-optimization
u^\star = &\underset{u}{\text{argmin}} \: \| u - u_\mathrm{nom}\|_2^2\\
&\text{subj. to} \:\: \nabla b(x)^Tf(x,u) \geq -\alpha(b(x))
```

If the dynamics are *control affine*, then {eq}`eq-cbf-optimization` is a quadratic program and can be solved very efficiently.


```{admonition} Example: Control Barrier Function for a Nonlinear System
Consider the nonlinear system:

$$
\dot{x}_1 = x_2, \quad \dot{x}_2 = -x_1 + u
$$

This system represents a simple pendulum with control input $u$. We aim to ensure that the system remains within a safe set $\mathcal{S}$ defined as:

$$
\mathcal{S} = \{ x \mid b(x) \geq 0 \}, \quad b(x) = 1 - x_1^2 - x_2^2
$$

Here, $\mathcal{S}$ is a circular region centered at the origin with radius 1. The goal is to ensure the system stays within this region.

The gradient of $b(x)$ is:

$$
\nabla b(x) = \begin{bmatrix} -2x_1 \\ -2x_2 \end{bmatrix}
$$

The system dynamics are:

$$
\dot{x} = \begin{bmatrix} x_2 \\ -x_1 + u \end{bmatrix}
$$

Substituting the dynamics into $\nabla b(x)^T f(x, u)$:

$$
\nabla b(x)^T f(x, u) = \begin{bmatrix} -2x_1 & -2x_2 \end{bmatrix} \begin{bmatrix} x_2 \\ -x_1 + u \end{bmatrix}
$$

Simplifying:

$$
\nabla b(x)^T f(x, u) = -2x_1 x_2 - 2x_2(-x_1 + u) = -2x_1 x_2 + 2x_1 x_2 - 2x_2 u = -2x_2 u
$$

To satisfy the CBF condition:

$$
-2x_2 u \geq -\alpha(b(x))
$$

where $\alpha(b(x))$ is a class $\mathcal{K}$ function, e.g., $\alpha(b(x)) = kb(x)$ with $k > 0$. Substituting $\alpha(b(x))$:

$$
-2x_2 u \geq -k(1 - x_1^2 - x_2^2)
$$

Rearranging:

$$
u \leq \frac{k(1 - x_1^2 - x_2^2)}{2x_2}, \quad x_2 \neq 0
$$

To ensure safety, we solve the following quadratic program (QP):

$$
u^\star = &\underset{u}{\text{argmin}} \: \| u - u_\mathrm{nom}\|_2^2\\
&\text{subj. to} \:\: -2x_2 u \geq -k(1 - x_1^2 - x_2^2)
$$

Here, $u_\mathrm{nom}$ is a nominal control input designed for other objectives (e.g., stabilization). The QP adjusts $u_\mathrm{nom}$ minimally to ensure the system remains within the safe set $\mathcal{S}$.
```


```{admonition} Pause and think
What are other choices of $\alpha$ that we could use? What kind of behaviors would they induce?
```

## CBF-CLF controller
We can combine both the CLF and CBF constraints into a single optimization, where we seek to achieve both stability (or convergence to a goal state) and safety simultaneously.

```{math}
:label: eq-cbf-clf-optimization
u^\star = &\underset{u}{\text{argmin}} \: \| u - u_\mathrm{nom}\|_2^2\\
&\text{subj. to} \:\: \nabla b(x)^Tf(x,u) \geq -\alpha(b(x))\\
& \qquad \quad \:\:\: \nabla V(x)^Tf(x,u) \leq 0
```


```{admonition} Pause and think
Are there any potential issues we may encounter when solving {eq}`eq-cbf-clf-optimization`? What could we do to address these issues?
```

## Other topics and extra readings

- Depending on the choice of your CLF/CBF for your given control affine dynamics $\dot{x} = f(x) + g(x)u$, you may find that for  $\nabla b(x)^Tg(x) = 0$ for all $x\in\mathcal{X}$. This occurs when the relative degree of the CLF/CBF for yhr dynamics is greater than 1. In that case, you will need to either change your CLF/CBF to make the relatively gree 1, or apply [high-order CBFs](https://ieeexplore.ieee.org/document/9516971).

- Finding a value CLF/CBF is not always straightforward and is challenging for general nonlinear systems. Recent work looks into parameterizing CLF/CBFs as deep neural networks and optimizing the network weights to ensure the CLF/CBF conditions are satisfied. Survey paper: [Neural CBFs and CLFs](https://ieeexplore.ieee.org/document/10015199)

- Another way to come up with a CBF (and perhaps also a CLF) is to use human demonstrations as example trajectories satisfying a CBF. Then we can parameterize a CBF and optimize a set of parameters that best explains the data. Paper: [Learning CBFs from demonstrations](https://arxiv.org/abs/2004.03315)

- Related to the above two points on learning CBFs data, this is a nice survey paper from IEEE Control Systems Magazine on [Data-Driven Safety Filters](https://ieeexplore.ieee.org/document/10266799).
