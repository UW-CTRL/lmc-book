# Control certificate functions

In this section, we introduce techniques for synthesizing controllers that guarantee the stability and safety of dynamical systems. Central to these techniques are certain scalar functions, known as *certificate functions*, which provide formal guarantees about specific properties of a controlled system.

Specifically, we focus on two important types of certificate functions:

- **Control Lyapunov Functions (CLFs):** Used to certify that a system can be stabilized through appropriate control.
- **Control Barrier Functions (CBFs):** Used to certify that safety constraints can be maintained throughout the system's evolution.

The term *certificate* comes from control theory and formal methods, where such functions serve as mathematical evidence that a suitable control policy exists to ensure a desired property, such as stability or safety.

We begin by revisiting Lyapunov theory and the concept of stability, then extend these ideas to the notion of *control* Lyapunov functions. Building on this, we introduce *barrier functions* and their controlled counterparts, *control barrier functions (CBFs)*. Finally, we demonstrate that for control-affine systems, it is possible to synthesize controllers that satisfy CLF and CBF constraints by solving a quadratic program—a class of convex optimization problems that can be efficiently solved in practice.

## Types of Stability

Consider a linear time-invariant (LTI) system $\dot{x} = Ax$. Let $x^\mathrm{eq}$ denote an equilibrium point, that is, a state for which $f(x^\mathrm{eq}) = 0$.

We classify the stability of this equilibrium as follows:

- **(Lyapunov) Stability:** For every $\epsilon > 0$, there exists a $\delta > 0$ such that if $|x(0)-x^\mathrm{eq}| < \delta$, then $|x(t)-x^\mathrm{eq}| < \epsilon$ for all $t \geq 0$.
- **Asymptotic Stability:** For every $\epsilon > 0$, there exists a $\delta > 0$ such that if $|x(0)-x^\mathrm{eq}| < \delta$, then $\lim_{t\to\infty} |x(t) - x^\mathrm{eq}| = 0$.
- **Exponential Stability:** There exist constants $\delta > 0$, $\alpha > 0$, and $\lambda > 0$ such that if $|x(0)-x^\mathrm{eq}| < \delta$, then $|x(t)-x^\mathrm{eq}| \leq \alpha e^{-\lambda t} |x(0) - x^\mathrm{eq}|$ for all $t \geq 0$.

These stability definitions refer to an equilibrium point and characterize whether solutions that start close to the equilibrium remain close or converge to it over time. Thus, they capture *local* stability properties of the system.

```{margin} Contraction theory
For the interested reader, *contraction theory* is a control certificate technique for determining the global stability of a system. Check out [this paper](https://www.sciencedirect.com/science/article/abs/pii/S0005109898000193), which introduces the idea of contraction analysis, and [this paper](https://arxiv.org/abs/1503.03144) for control feedback design using contraction metrics.
```

When stability holds for *all* initial conditions—not just those close to the equilibrium—this is referred to as *global* stability. Global stability is a much stronger property than local stability and can be significantly more challenging to establish.

| **Type**                  | **Definition**                                                    | **Notes**                                 |
|---------------------------|--------------------------------------------------------------------|--------------------------------------------|
| **Lyapunov Stability**    | Trajectories remain close to equilibrium if they start nearby      | Does not guarantee convergence             |
| **Asymptotic Stability**  | Trajectories remain close and eventually converge to equilibrium   | Implies Lyapunov stability                 |
| **Exponential Stability** | Trajectories converge to equilibrium at an exponential rate        | Stronger than asymptotic stability         |
| **Local Stability**       | Stability properties hold for initial conditions *near* equilibrium| Typical in nonlinear systems               |
| **Global Stability**      | Stability properties hold for *all* initial conditions             | Strongest, but often difficult to prove for nonlinear systems   |


## Checking for Stability

### Linear Systems

To assess the stability of a linear system (without control input), that is, $\dot{x} = Ax$, we examine the eigenvalues of $A$. The system is (exponentially) stable if **all** eigenvalues of $A$ have strictly negative real parts.

```{admonition} Pause and Think
Why does the sign of the real part of the eigenvalues determine stability?
```

```{admonition} Solution to $\dot{x} = Ax$
:class: dropdown
Recall that the general solution to $\dot{x} = Ax$ is $x(t) = V e^{D t} V^{-1} x(0)$, where $A = V D V^{-1}$ is the diagonalization of $A$, and $D$ is a diagonal matrix of eigenvalues. The behavior of $x(t)$ is governed by $e^{D t}$: if any eigenvalue has a positive real part, $e^{D t}$ grows exponentially. If all have negative real parts, $e^{D t}$ decays exponentially—ensuring all trajectories approach the origin.
```

```{admonition} Pause and Think
How does this result generalize to discrete-time linear dynamics $x_{t+1} = A x_t$?
```

For more on the stability of linear time-invariant systems, see [Professor Xu Chen's ME 547 notes](https://faculty.washington.edu/chx/teaching/me547/stability_eigenvalue/).


### Lyapunov functions

Assessing the stability of nonlinear systems is generally a challenging problem. Lyapunov theory, however, offers a powerful and widely applicable method for establishing stability without requiring explicit solutions to the system dynamics. The core idea is that if we can find a suitable Lyapunov function satisfying certain conditions, then we can certify the stability of a given equilibrium point. While the conditions are relatively straightforward to check, the main challenge lies in finding an appropriate Lyapunov function itself. If one can be found, its existence serves as a mathematical certificate of stability.

```{admonition} Theorem (Lyapunov Function)
Consider a general nonlinear dynamical system $\dot{x} = f(x)$, where $x \in \mathcal{X} \subset \mathbb{R}^n$.
Assume without loss of generality that $x=0$ is an equilibrium point.
Let $V:\mathbb{R}^n \rightarrow \mathbb{R}$ be a scalar, continuously differentiable function (a Lyapunov candidate).
Then the equilibrium $x=0$ is Lyapunov stable if:
- $V(0) = 0$,
- $V(x) > 0$ for all $x\in\mathcal{X} \setminus \{ 0\}$,
- $\dot{V}(x) = \nabla V(x)^Tf(x) \leq 0$ for all $x\in\mathcal{X}$.
```
If the last condition is satisfied with strict inequality ($\dot{V}(x) < 0$ for all $x \neq 0$), the equilibrium is asymptotically stable. If instead $\dot{V}(x) \leq -\alpha V(x)$ for some $\alpha > 0$, the system exhibits exponential stability at a rate $\alpha$.

```{admonition} Exercise
For linear systems $\dot{x} = Ax$, a common Lyapunov candidate is the quadratic function $V(x) = x^T P x$ where $P$ is symmetric and positive definite ($P = P^T, P \succ 0$).
Show that $V(x)$ satisfies the Lyapunov conditions if and only if $A^T P + P A \preceq 0$.
```

```{admonition} Theorem (Lyapunov Stability for Linear Systems)
For a linear system $\dot{x} = Ax$, the origin $x=0$ is asymptotically stable if and only if, for any symmetric positive definite matrix $Q$, the Lyapunov equation

$$A^T P + P A = -Q$$

admits a unique symmetric positive definite solution $P \succ 0$.
```

Intuitively, a Lyapunov function plays a role similar to that of an "energy" for the system: if we can construct a function that decreases along system trajectories, this function demonstrates that the system dissipates energy over time and ultimately settles to equilibrium. While not all dynamical systems have a physical energy interpretation, for mechanical systems, physical energy is often a natural starting point for constructing a Lyapunov candidate.



## Control Lyapunov Functions (CLF)

Up to this point, we have focused on assessing the stability of dynamical systems **without** control input. But what if a system is naturally unstable with zero control input? Sometimes, even if the open-loop (uncontrolled) dynamics are unstable, it is possible to stabilize the system by carefully choosing a control input $u$. In other words, for certain choices of $u$, the Lyapunov conditions can be satisfied. 

This motivates a natural extension of Lyapunov theory—*Control* Lyapunov theory—where we explicitly exploit the ability to influence the system through control. Conceptually, the definition mirrors that of an ordinary Lyapunov function, but adapts the stability condition to account for the control input.

```{admonition} Theorem (Control Lyapunov Function)
Consider a general nonlinear system $\dot{x} = f(x, u)$, where $x \in \mathcal{X} \subset \mathbb{R}^n$ and $u \in \mathcal{U} \subset \mathbb{R}^m$.
Assume without loss of generality that $x = 0$ is an equilibrium point.
Let $V:\mathbb{R}^n \rightarrow \mathbb{R}$ be a scalar function that is continuous and continuously differentiable.
Then the origin $x = 0$ is Lyapunov stable if:
- $V(0) = 0$
- $V(x) > 0$ for all $x \in \mathcal{X} \setminus \{0\}$
- For all $x \in \mathcal{X}$, $\min_{u \in \mathcal{U}} \nabla V(x)^T f(x, u) \leq 0$
```

The last condition means that *for every state $x$ in the domain*, there exists a control $u \in \mathcal{U}$ which ensures that the time derivative of the Lyapunov function is nonpositive (i.e., the "energy" does not increase). As before, by strengthening this condition—for example, requiring strict inequality ($< 0$) or exponential decay (i.e., $\leq -\alpha V(x)$ for some $\alpha > 0$)—we can certify asymptotic or exponential stability, respectively.

The key insight is that if we can find a valid Control Lyapunov Function (CLF), this guarantees that, *from any state in our domain*, there always exists a control $u$ that can stabilize the system.


### CLF Controller

Suppose you have a *nominal controller* for your system. Regardless of how this nominal controller is designed, at any time $t$ and state $x$, it outputs a control input $u_\mathrm{nom}$. However, $u_\mathrm{nom}$ may not inherently guarantee stability or convergence to the desired equilibrium. To address this, we can *modify* $u_\mathrm{nom}$ slightly, aiming to minimally adjust it so that the control Lyapunov function (CLF) conditions are satisfied. The goal is to stay as close as possible to $u_\mathrm{nom}$—which is usually chosen because it achieves specific task objectives—while ensuring stability.

We can formalize this idea as the following optimization problem:

```{math}
:label: eq-clf-optimization
u^\star = \underset{u\in \mathcal{U}}{\text{argmin}} \;\; \| u - u_\mathrm{nom}\|_2^2 \\
\text{subject to} \;\; \nabla V(x)^T f(x, u) \leq 0
```

Note that the constraint can be strengthened to enforce asymptotic or exponential stability by tightening the inequality appropriately.

For general nonlinear dynamics, the constraint above is typically nonlinear and may be non-convex, making the optimization problem challenging to solve directly. However, for **control-affine systems**, where the dynamics can be written as

```{math}
\dot{x} = f(x) + g(x)u,
```

the constraint in {eq}`eq-clf-optimization` becomes *linear* with respect to $u$. This structure turns the problem into a quadratic program (QP), which is convex and can be solved efficiently with standard optimization tools.

```{admonition} Example: Control Lyapunov Function for a Nonlinear System

Consider the nonlinear dynamical system:

$$
\dot{x}_1 = x_2, \quad \dot{x}_2 = -x_1 + u
$$

This system describes a simple pendulum actuated by a control input $u$. Our goal is to stabilize the state to the origin $(x_1, x_2) = (0, 0)$.

A natural choice for a Lyapunov function is the total energy:

$$
V(x) = \frac{1}{2}x_1^2 + \frac{1}{2}x_2^2,
$$

which is positive definite and radially unbounded. Computing its time derivative along the system trajectories gives:

$$
\dot{V}(x) = \nabla V(x)^T f(x, u) = x_1 \dot{x}_1 + x_2 \dot{x}_2 = x_1 x_2 + x_2(-x_1 + u).
$$

By simplifying,

$$
\dot{V}(x) = x_2(-x_1 + x_1 + u) = x_2 u.
$$

To enforce $\dot{V}(x) \leq 0$, we need to select a control $u$ such that $x_2 u \leq 0$ for all $x$. One simple choice is

$$
u = -k x_2, \quad k > 0.
$$

Substituting this into $\dot{V}(x)$:

$$
\dot{V}(x) = x_2(-k x_2) = -k x_2^2.
$$

Since $k > 0$, $\dot{V}(x) \leq 0$ always holds, ensuring stability. Thus, $V(x)$ is a valid control Lyapunov function for this system.

```



## Control Invariant Sets

The sub-level sets of a (control) Lyapunov function, $\mathcal{C} = \{ x \mid V(x) \leq c \}$, are not just mathematical constructions—they are *forward (control) invariant sets*. This means that if the system begins within such a set, it will remain there for all future times. For *control* invariant sets, there is an important addition: there must exist a control input such that, starting in the set, there is always a way to keep the system inside the set as time progresses.

In other words, suppose the system begins at $x(0)$ with $V(x(0)) \leq c$. Because $\dot{V}(x) \leq 0$, the value of $V(x)$ cannot increase: for all future times, $V(x(t)) \leq V(x(0)) \leq c$. Therefore, by choosing $c = V(x(0))$, we guarantee that $x(t)$ stays in $\mathcal{C}$ for all $t \geq 0$. This is how Lyapunov functions naturally define invariant "funnels" around desired states.

## Barrier Functions

Until now, our focus has been on using Lyapunov functions to argue stability—that is, that the system will eventually converge to a particular equilibrium state. But what if our goal is less about convergence to a point, and more about ensuring the state simply remains within a certain desirable region? Instead of converging to a point, we may want the system to stay inside a *set* for all time, regardless of *where* in the set.

To generalize Lyapunov ideas toward set invariance, we introduce *barrier functions*. Typically, we define the *safe set* $\mathcal{S} = \{ x \mid b(x) \geq 0 \}$ for some scalar function $b : \mathbb{R}^n \rightarrow \mathbb{R}$. The goal is to ensure that, if the system starts in $\mathcal{S}$, it stays there.

Consider the dynamics $\dot{x} = f(x)$, and suppose $b(x(0)) \geq 0$, i.e., the initial condition is safe. If, at any point on the boundary ($b(x) = 0$), we have $\nabla b(x)^T f(x) \geq 0$, then $b$ is called a *barrier function*. This condition means that when the system approaches the boundary of the safe set, its trajectory does not point outward (away from safety); instead, the dynamics ensure that $b(x)$ does not decrease below zero. Thus, $b(x)$ acts as a "barrier," preventing the state from leaving $\mathcal{S}$. When the system is at the boundary, it will either move back into the safe region ($b(x)$ increases) or remain on the boundary ($b(x)$ stays constant).

This guarantees that as long as the system starts in the safe set $\mathcal{S}$, it cannot escape—thus maintaining safety over time.




## Control Barrier Functions (CBFs)

```{margin} Safe Autonomy with Control Barrier Functions
The textbook [Safe Autonomy with Control Barrier Functions](https://link.springer.com/book/10.1007/978-3-031-27576-0) by Wei Xiao, Christos G. Cassandras, and Calin Belta provides an excellent introduction and comprehensive overview of control barrier functions and their application in the context of safe autonomy.
```

The concept of barrier functions can be naturally generalized to *control barrier functions* (CBFs), just as Lyapunov functions were extended to control Lyapunov functions (CLFs).

To accommodate the effect of control inputs, we slightly modify the classic barrier condition. Let $b : \mathbb{R}^n \rightarrow \mathbb{R}$ be a scalar function, and consider a system with dynamics $\dot{x} = f(x, u)$. We define the "safe set" as $\mathcal{S} = \{ x \mid b(x) \geq 0 \}$. 

If, for every $x$ on the boundary ($b(x) = 0$), there exists at least one control action $u$ such that
$$
\max_{u \in \mathcal{U}} \nabla b(x)^T f(x, u) \geq 0,
$$
then, as long as $b(x(0)) \geq 0$, the state will remain inside $\mathcal{S}$ for all future times. Setting the right-hand side to $0$ provides the most flexibility – inside $\mathcal{S}$ the system can move freely, but at the boundary, the system must be able to avoid leaving $\mathcal{S}$.

However, in many practical scenarios, we prefer a more progressive constraint as the state approaches the boundary. Intuitively, the closer the state gets to the boundary of $\mathcal{S}$, the tighter the restrictions on its motion should become – for example, the system should begin to slow down or divert to remain safe. Conversely, deep inside the safe region, we want to preserve maximum flexibility. To obtain this behavior, we generalize the condition by introducing a function on the right-hand side, leading to the following definition.

```{admonition} Class $\mathcal{K}$ and Extended Class $\mathcal{K}$ Functions
A Lipschitz continuous function $\alpha : [0, a) \to [0, \infty)$, $a > 0$, is said to belong to class $\mathcal{K}$ if it is strictly increasing and $\alpha(0) = 0$. Moreover, $\alpha : [-b, a) \to [-\infty, \infty)$, $a > 0$, $b > 0$, is said to belong to the *extended* class $\mathcal{K}$ if it is strictly increasing and $\alpha(0) = 0$.
```

Intuitively, functions in class $\mathcal{K}$ (and extended class $\mathcal{K}$) are monotonically increasing and equal to zero at the origin. A typical example is a linear function $\alpha(x) = ax$.

Now, we formally define a control barrier function:

```{admonition} Definition (Control Barrier Function)
Let $\mathcal{S} = \{ x \mid b(x) \geq 0\}$, where $b : \mathbb{R}^n \to \mathbb{R}$. The function $b$ is called a *control barrier function* for the system $\dot{x} = f(x, u)$ if there exists a class $\mathcal{K}$ function $\alpha$ such that
```{math}
:label: eq-cbf
\max_{u\in\mathcal{U}} \nabla b(x)^T f(x, u) \geq -\alpha(b(x)), \quad \forall x \in \mathcal{S}
```
This formulation is directly analogous to the CLF definition, but now the right-hand side is $-\alpha(b(x))$ rather than $0$. This ensures a gradual restriction on the system’s motion as it nears the boundary of $\mathcal{S}$. The $\max$ ensures the existence of at least one control input that can maintain safety.

If we can find a function $b$ and a class $\mathcal{K}$ function $\alpha$ so that the CBF condition holds everywhere in $\mathcal{S}$, then we are guaranteed that any trajectory starting inside $\mathcal{S}$ will remain in $\mathcal{S}$ for all time.

### Finding valid CBFs

Constructing a valid CBF (or CLF) is, in general, a challenging task. This may rely on a deep understanding of the system under consideration, or on using systematic techniques to synthesize such functions. Later in the course, we will see that certain value functions arising from Hamilton-Jacobi (HJ) reachability analysis provide valid CBFs.


## CBF Safety Filter

Thus far, our discussion of Control Barrier Functions (CBFs) has focused on their theoretical definition. Now, let’s turn to the practical question: how do we design a controller that actually keeps the system inside the safe set $\mathcal{S}$?

Recall that, for safety, the control input $u$ must satisfy the CBF condition in {eq}`eq-cbf` at all times. By definition, a valid CBF ensures that at every state within $\mathcal{S}$, there exists *some* control input that keeps the system safe. In real applications, however, we typically have a *nominal* controller (e.g., for following a trajectory or achieving regulatory objectives), and we want to stay as close as possible to this nominal behavior—so long as it remains safe.

This safety-versus-performance trade-off can be formulated as an optimization problem:

```{math}
:label: eq-cbf-optimization
u^\star = \underset{u}{\text{argmin}} \; \| u - u_\mathrm{nom}\|_2^2 \\
\text{subj. to} \;\; \nabla b(x)^\top f(x,u) \geq -\alpha(b(x))
```

For *control affine* systems (i.e., dynamics of the form $f(x,u) = f_0(x) + g(x)u$), this is a quadratic program (QP): a convex optimization problem that can be solved efficiently using standard solvers.

Just as with Control Lyapunov Functions, we pose a quadratic program to compute a minimally modifying, safety-preserving input at every step.

```{math}
:label: eq-cbf-optimization
u^\star = \underset{u}{\text{argmin}} \; \| u - u_\mathrm{nom}\|_2^2 \\
\text{subj. to} \;\; \nabla b(x)^\top f(x,u) \geq -\alpha(b(x))
```

Again, under *control affine* dynamics, {eq}`eq-cbf-optimization` is a QP and can be solved very efficiently.

---

```{admonition} Example: Control Barrier Function for a Nonlinear System
Consider the following nonlinear system:

$$
\dot{x}_1 = x_2, \quad \dot{x}_2 = -x_1 + u
$$

This is the equation of a simple pendulum with control input $u$. We want to keep the state inside the safe set

$$
\mathcal{S} = \{ x \mid b(x) \geq 0 \}, \qquad b(x) = 1 - x_1^2 - x_2^2
$$

Thus, $\mathcal{S}$ is the unit disk centered at the origin: our goal is to keep the system within this region.

The gradient of $b(x)$ is:

$$
\nabla b(x) = \begin{bmatrix} -2x_1 \\ -2x_2 \end{bmatrix}
$$

and the dynamics vector field is

$$
f(x, u) = \begin{bmatrix} x_2 \\ -x_1 + u \end{bmatrix}
$$

The CBF term computes to:

$$
\nabla b(x)^\top f(x, u) = [-2x_1, -2x_2] \begin{bmatrix} x_2 \\ -x_1 + u \end{bmatrix}
$$

Carrying out this multiplication:

$$
= -2x_1 x_2 - 2x_2 (-x_1 + u) \\
= -2x_1 x_2 + 2x_1 x_2 - 2x_2 u \\
= -2x_2 u
$$

So, the CBF condition is:

$$
-2x_2 u \geq -\alpha(b(x))
$$

If we use a linear class $\mathcal{K}$ function, e.g., $\alpha(b(x)) = k b(x)$ with $k > 0$, this reads:

$$
-2x_2 u \geq -k(1 - x_1^2 - x_2^2)
$$

or equivalently (when $x_2 \neq 0$):

$$
u \leq \frac{k(1 - x_1^2 - x_2^2)}{2x_2}
$$

To enforce safety, we solve the following QP:
$$
u^\star = \underset{u}{\text{argmin}} \; \| u - u_\mathrm{nom}\|_2^2 \\
\text{subj. to} \;\; -2x_2 u \geq -k(1 - x_1^2 - x_2^2)
$$

Here, $u_\mathrm{nom}$ is the nominal control input (for instance, designed for stabilization). The QP makes the smallest possible adjustment to $u_\mathrm{nom}$ so that the system stays within the safe set $\mathcal{S}$.
```

```{admonition} Pause and think
What are other possible choices of $\alpha$? How would they affect the safety filter’s behavior?
```

---

## CBF-CLF Controller

In practice, we often want to guarantee both stability (using a Control Lyapunov Function, or CLF) and safety (using a Control Barrier Function, or CBF) for our system. These two objectives can be elegantly combined within a single optimization framework:

```{math}
:label: eq-cbf-clf-optimization
u^\star = \underset{u}{\mathrm{argmin}} \; \| u - u_\mathrm{nom}\|_2^2 \\
\text{subject to} \;\; \nabla b(x)^\top f(x, u) \geq -\alpha(b(x)) \\
\qquad\qquad\qquad\,\, \nabla V(x)^\top f(x, u) \leq 0
```

Here, we seek a control input $u$ that stays as close as possible to a nominal input $u_\mathrm{nom}$ (for example, a controller designed without safety constraints), while simultaneously ensuring safety (through the CBF constraint) and stability (through the CLF constraint). The resulting controller finds the minimal modification to the nominal input needed to keep the system both safe and stable.

```{admonition} Pause and think
When formulating {eq}`eq-cbf-clf-optimization`, what practical challenges might you encounter? How might these be addressed in implementation?
```


## Other topics and extra readings

- Depending on the choice of your CLF/CBF for your given control affine dynamics $\dot{x} = f(x) + g(x)u$, you may find that for  $\nabla b(x)^Tg(x) = 0$ for all $x\in\mathcal{X}$. This occurs when the relative degree of the CLF/CBF for yhr dynamics is greater than 1. In that case, you will need to either change your CLF/CBF to make the relatively gree 1, or apply [high-order CBFs](https://ieeexplore.ieee.org/document/9516971).

- Finding a value CLF/CBF is not always straightforward and is challenging for general nonlinear systems. Recent work looks into parameterizing CLF/CBFs as deep neural networks and optimizing the network weights to ensure the CLF/CBF conditions are satisfied. Survey paper: [Neural CBFs and CLFs](https://ieeexplore.ieee.org/document/10015199)

- Another way to come up with a CBF (and perhaps also a CLF) is to use human demonstrations as example trajectories satisfying a CBF. Then we can parameterize a CBF and optimize a set of parameters that best explains the data. Paper: [Learning CBFs from demonstrations](https://arxiv.org/abs/2004.03315)

- Related to the above two points on learning CBFs data, this is a nice survey paper from IEEE Control Systems Magazine on [Data-Driven Safety Filters](https://ieeexplore.ieee.org/document/10266799).
