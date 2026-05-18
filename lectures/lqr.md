# Linear Quadratic Regulator

In the previous chapter, we explored sequential decision-making and learned that, by applying the principle of optimality, we can decompose an optimal control problem into a series of smaller decisions that can be solved recursively. This led us to derive the **Bellman Equation** (for discrete time) and the **Hamilton-Jacobi-Bellman (HJB) Equation** (for continuous time).

While these equations elegantly characterize the solution to the optimal control problem in theory, they are often extremely challenging, or even impossible, to solve exactly in practice. Often times, the value function is approximated instead, such as with a neural network, or some form of local search is applied. Several factors contribute to this intractability. For instance, when the state or control spaces are large (in the discrete, finite case) or when the minimization step in the Bellman or HJB equation is itself difficult to solve (e.g., due to non-convexity), finding a solution becomes prohibitive. The complexity of this minimization depends on the structure of the cost function, the dynamics, and the current form of the value function. If the ingredients of this optimization (such as cost functions being nonlinear or dynamics being complicated) are not particularly friendly, then solving these equations becomes extremely hard.

A more subtle challenge is that even if we can solve the minimization at a single time step, we need assurance that this tractability will persist at every subsequent time step as we iterate backward through time using dynamic programming. In other words, we need to ensure that the solution at time $t$ yields a value function form that keeps the next backward step manageable. Otherwise, the recursive process could quickly become intractable even if it may be tractable for the first step.

*So under what conditions is solving these equations tractable?*

## Key assumption of LQR

We make the following simplifying assumptions on the optimal control problem, and show that with these assumptions, solving the Bellman and HJB equations is very tractable and exhibits very nice structure.
For now, let's first consider a discrete-time setting.


```{admonition} Key Assumptions of LQR
:label: eq-lqr-assumptions
- **Linear dynamics:** The system dynamics are assumed to be linear: $x_{k+1} = A_k x_k + B_k u_k$. The matrices $A_k$ and $B_k$ may be time-varying (hence the subscript $k$).
- **Quadratic cost:** The cost function is quadratic in both the states and the controls. Specifically, the stage cost is given by $g(x_k, u_k, t_k) = x_k^{\top} Q_k x_k + u_k^{\top} R_k u_k$, and the terminal cost is $g_N(x_N) = x_N^{\top} Q_N x_N$, where $Q_k^{\top} = Q_k \succeq 0$, $Q_N^{\top} = Q_N \succeq 0$, and $R_k^{\top} = R_k \succ 0$.
- **No additional constraints:** The only constraints are those imposed by the system dynamics and the initial state. There are no explicit constraints on the controls (such as actuator limits) or the states (such as obstacle avoidance).
```

Given these assumptions, the resulting optimal control problem becomes,


```{admonition} LQR optimal control problem
```{math}
:label: eq-lqr-ocp
\min_{u_0,\ldots u_{N-1}} & \: x_N^{\top}Q_Nx_N + \sum_{t=0}^{N-1} x_k^{\top}Q_kx_k + u_k^{\top}R_ku_k\\
\text{subject to} & \: x_{k+1} = A_kx_k + B_ku_k, \quad k=0,\ldots, N-1\\
& \: x_0 = x_\mathrm{current}
```

As a preview of what’s to come: under these assumptions, the value function $V(x, t)$ is guaranteed to take a quadratic form. Specifically, for each time step $k = 0, \ldots, N$, the value function can be written as $V(x_k, t_k) = x_k^\top P_k x_k$. Although this result may not be obvious at first glance, we will demonstrate it in the following sections. The intuition is that, by starting with a quadratic terminal cost and initializing the value function as $V(x_N, t_N) = g_N(x_N) = x_N^\top Q_T x_N$, the preservation of quadratic structure through dynamic programming ensures $V(x, t)$ remains quadratic at each step.


### Why is it called a Linear Quadratic Regulator?

Given these assumptions, the meaning behind each part of the name "Linear Quadratic Regulator" becomes clear:

- **Linear:** The dynamics of the system are linear in both the state and control variables. Interestingly, under the LQR framework, the *optimal* control law that results is also linear in the state.
- **Quadratic:** The cost we aim to minimize is quadratic in both the states and the controls. As a result, the value function—representing the optimal total cost-to-go—also takes a quadratic form.
- **Regulator:** The particular structure of the cost is designed to penalize deviations from the zero state (and from zero control effort), so the optimal controller works to regulate (drive) the system’s state toward zero over time.

In summary, LQR refers to the fact that we are using an optimal *regulator* (feedback controller) designed for *linear* systems with *quadratic* performance criteria.


## Discrete-time LQR

Consider the discrete-time and finite horizon setting, and for simplicity, let's also assume a time-invariant setting (drop the subscripts in $A, B, Q, R$).
Recall the Bellman equation {eq}`eq-bellman`. Let's update the equation with the LQR assumptions.

$$
V^*(x_{t},t) = \min_{u_{t}} \biggl( x_t^TQx_t + u_t^TRu_t + V^*(Ax_t + Bu_t, t+1) \biggl)
$$

To start off the recursion at the end of the horizon, we initialize the value function with $V^*(x,T) = x^TP_Tx$ where $P_T = Q_T$.

$$
V^*(x_{T-1},T-1) &= \min_{u_{T-1}} \biggl( x_{T-1}^TQx_{T-1} + u_{T-1}^TRu_{T-1} + V^*(Ax_{T-1} + Bu_{T-1}, T) \biggl)\\
&= \min_{u_{T-1}} \biggl( x_{T-1}^TQx_{T-1} + u_{T-1}^TRu_{T-1} + (Ax_{T-1} + Bu_{T-1})^TP_T(Ax_{T-1} + Bu_{T-1}) \biggl)\\
$$

Now, we expand this expression and rearrange some terms.

$$
V^*(x_{T-1},T-1) &= \min_{u_{T-1}} \biggl( x_{T-1}^TQx_{T-1} + u_{T-1}^TRu_{T-1} + (Ax_{T-1} + Bu_{T-1})^TP_T(Ax_{T-1} + Bu_{T-1}) \biggl)\\
&= \min_{u_{T-1}} \biggl( x_{T-1}^TQx_{T-1} + u_{T-1}^TRu_{T-1} + x_{T-1}^TA^TP_TAx_{T-1} + 2u_{T-1}^TB^TP_TAx_{T-1} + u_{T-1}^TB^TP_TBu_{T-1} \biggl)\\
&= \min_{u_{T-1}} \biggl( x_{T-1}^T(Q + A^TP_TA)x_{T-1} + u_{T-1}^T(B^TP_TB+R)u_{T-1} + 2u_{T-1}^TB^TP_TAx_{T-1} \biggl)\\
$$

Note that the expression that we want to minimize with respect to $u_{T-1}$ is *quadratic in $u_{T-1}$! It consists of a constant term (depends on $x_{T-1}$), a linear term, and a quadratic term.
This is very convenient since it is straightforward to take the derivative and set it to zero. We also use the fact that $B^TP_TB+R \succeq 0$ so that first order conditions for optimality are also sufficient.

Recall from vector calculus: $\frac{d}{dx} x^TMx = 2Mx$ and $\frac{d}{dx} x^T M = M$. We have

$$
0 &= \frac{d}{du_{T-1}}  \biggl( x_{T-1}^T(Q + A^TP_TA)x_{T-1} + u_{T-1}^T(B^TP_TB+R)u_{T-1} + 2u_{T-1}^TB^TP_TAx_{T-1} \biggl)\\
&= 2(B^TP_TB+R)u_{T-1} + 2B^TP_TAx_{T-1}\\
\Rightarrow u_{T-1}^* &= - \underbrace{(B^TP_TB+R)^{-1}B^TP_TA}_{K_{T-1}}x_{T-1}\\
& u^*_{T-1} = - K_{T-1}x_{T-1}, \quad K_{T-1} = (B^TP_TB+R)^{-1}B^TP_TA
$$

This turns out to be a rather interesting results. It tells us that the *optimal control* at time step $T-1$ is *linear in state*! Despite the seemingly complex set up and optimal control framework, the optimal controller is simply a linear controller. While we could have, in theory, spent a lot of time tuning a linear controller, we can now find an *optimal* gain with respect to our cost function.

Now, let's plug our optimal controller back into the value function and see what $V^*(x_{T-1},T-1)$ ends up being. Note that $Q_T = P_T = P_T^T$, $(A^T)^{-1} = (A^{-1})^T$, and $(AB)^T = B^TA^T$.

$$
V^*(x_{T-1},T-1) &= \min_{u_{T-1}} \biggl( x_{T-1}^T(Q + A^TP_TA)x_{T-1} + u_{T-1}^T(B^TP_TB+R)u_{T-1} + 2u_{T-1}^TB^TP_TAx_{T-1} \biggl)\\
& = x_{T-1}^T(Q + A^TP_TA)x_{T-1} + x_{T-1}^TK_{T-1}^T(B^TP_TB+R)K_{T-1}x_{T-1} - 2x_{T-1}^TK_{T-1}^TB^TP_TAx_{T-1}\\
& = x_{T-1}^T\biggl( Q + A^TP_TA +  K_{T-1}^T(B^TP_TB+R)K_{T-1} - 2K_{T-1}^TB^TP_TA  \biggl)x_{T-1}\\
& = x_{T-1}^T\biggl( Q + A^TP_TA +  A^TP_TB(B^TP_TB+R)^{-1}(B^TP_TB+R)(B^TP_TB+R)^{-1}B^TP_TA - 2A^TP_TB(B^TP_TB+R)^{-1}B^TP_TA  \biggl)x_{T-1}\\
& = x_{T-1}^T\biggl( Q + A^TP_TA +  A^TP_TB(B^TP_TB+R)^{-1}B^TP_TA - 2A^TP_TB(B^TP_TB+R)^{-1}B^TP_TA  \biggl)x_{T-1}\\
& = x_{T-1}^T\underbrace{\biggl( Q + A^TP_TA -  A^TP_TB(B^TP_TB+R)^{-1}B^TP_TA  \biggl)}_{\succeq 0}x_{T-1}\\
$$

In the final expression, we find that we get another quadratic expression! Given the assumptions on $P_T$, that quadratic term is PSD.
Also note that it only depends on $P_T$, the dynamics $(A,B)$ and the cost matrices $(Q,R)$.

If we let $P_{T-1} =  Q + A^TP_TA -  A^TP_TB(B^TP_TB+R)^{-1}B^TP_TA $, then we will have $V(x, T-1) = x^TP_{T-1}x$, another quadratic expression!
But this was just *one* dynamic programming step. We would need to continue and find an expression for all other time step $T-2,\ldots, 1, 0$.
Notice that although the above derivation was for $P_T = Q_T$, it only used the fact that $Q_T =Q_T^T \succeq 0$. Since we showed that $P_{T-1} \succeq 0$, then carrying out the next dynamic programming step would yield the same results but with the time index reduced by one.
Essentially, we have derived an *update* equation for $P_t$ where $V(x,t) = x^TP_tx$.

```{admonition} Discrete-time finite horizon LQR
```{math}
:label: eq-dt-finite-lqr
P_{t-1} &= Q + A^TP_tA -  A^TP_tB(B^TP_tB+R)^{-1}B^TP_tA, \quad P_T = Q_T\\
K_{t} &= (B^TP_{t+1}B+R)^{-1}B^TP_{t+1}A\\
u^*_{t} &= -K_{t}x_{t}
```

The update equation for this discrete-time finite horizon setting is called the **dynamic Riccati equation**.
We can find the values for $P_T, P_{T-1},\ldots, P_1, P_0$ offline, simply by using the dynamic Riccati equation, starting from $T$ and iterating *backward* in time.
Once we find all the values for $P_t$, we can also compute all the gains $K_0, K_1, \ldots, K_{T-1}$, also offline.
The online, to find the optimal control at time step $t$, we simply look up our value for $K_t$ and compute $u^*_t = -K_tx_t$.

### Infinite horizon case
The above derivation was for a finite horizon setting. What if now $T\rightarrow \infty$? We cannot iterate on $P_t$ forever!
It turns out that after some time, the value for $P_t$ converges. That is $P_t = P_{t+1}$ for some sufficiently large $t$.
Assuming $P_t$ has converged, and setting $P_\infty =P_t = P_{t+1}$, the dynamic Riccati equation becomes,

```{admonition} Discrete-time infinite horizon LQR (DARE)
```{math}
:label: eq-dare
P_\infty = Q + A^TP_\infty A -  A^TP_\infty B(B^TP_\infty B+R)^{-1}B^TP_\infty A.
```

This equation is referred to as the **discrete time algebraic Riccati equation (DARE)** for which there are well-established ways to solve that linear matrix equation. Practically, there are built-in functions in control system packages that we can call to find $P_\infty$. But don't forget to read the documentation to know how to use it properly!



## Continuous-time LQR
Now that we understand the derivation for the discrete-time LQR, it becomes relatively straightforward to derive the continuous-time LQR.
We leave this as an exercise, but provide a high-level sketch of the steps (which essentially mirrors that for the discrete-time setting).

1. Instead of using the Bellman equation, we use the HJB equation: $\frac{\partial V}{\partial t}(x,t) + \min_u J(x,u,t) + \nabla V(x,t)^Tf(x,u,t) = 0$
2. The value function has the form $V(x,t) = x^TP(t)x$ and $P(T) = Q_T$
3. When we have $x^T M x$ where $M$ is any square matrix which can be written as $M = \frac{1}{2}(M + M^T) + \frac{1}{2}(M - M^T)$, only the symmetric part of $M$ remains, whereas the skew-symmetric part will be cancelled out. So $x^TMx = \frac{1}{2}x^T(M+M^T)x$.

After working out the details, you should find,

```{admonition} Continuous-time finite horizon LQR
```{math}
:label: eq-ct-finite-lqr
0 & = \dot{P}(t) + Q + A^TP(t) + P(t)A - P(t)BR^{-1}B^TP(t), \quad P(T) = Q_T\\
K(t) &= R^{-1}B^TP(t)\\
u^*(t) &= -K(t)x(t)
```
Then for the infinite horizon case, with $P(t)$ converging to a constant value, we have $\dot{P} = 0$.

```{admonition} Continuous-time infinite horizon LQR (CARE)
```{math}
:label: eq-care
0 =  Q + A^TP_\infty  + P_\infty A - P_\infty BR^{-1}B^TP_\infty
```

This equation is referred to as the **continuous time algebraic Riccati equation (CARE)** for which there are well-established ways to solve that linear matrix equation. Like DARE, there are built-in functions in control system packages that we can use.


## Connection with Lyapunov theory
Notice that for both the discrete- and continuous-time setting, the resulting Riccati equations look similar to the Lyapunov equations, but there are some extra terms in the LQR equations.
Considering the discrete-time setting for now (the continuous-time setting follows similarly), it turns out that for our *closed-loop* dynamics $x_{t+1} = (A - BK_t)x_t$ where $K_t = (B^TP_{t+1}B+R)^{-1}B^TP_{t+1}A$, then $V(x,t) = x^TP_tx$ is a valid Lyapunov function for the closed-loop system.
Differently put, LQR provides a constructive method for finding a valid control Lyapunov function for linear dynamics.

```{admonition} Pause and think
Can you prove that the value function is a valid Lyapunov function for the closed-loop system?
```


This means that with LQR, we are guaranteed stability to the origin (assuming some basic assumptions are met).


## Discussion
(TODO)
- Conditions for LQR to work (controllability, stabilizability, detectability, etc)
- Time-varying LQR



## Tracking LQR

The above LQR formulation assumed that we wanted the state to regulate to zero. But that's not always the case. In fact, we often would like come up with a control that enables our system to track a nominal trajectory given by $(\bar{x}_0, \bar{u}_0), (\bar{x}_1,\bar{u}_1), \ldots$.
What can we do?
Well actually, we would like the difference between the nominal state/control and the state/control to regulate to zero instead. As such, we should consider the *error* states and controls like so.

$$
\delta x_t = x_t - \bar{x}_t, \quad \delta u_t = u_t - \bar{u}_t
$$

Given this definition of error states, it follows that $ \delta x_{t+1} = A \delta x_t + B \delta u_t$.
We can also define the cost to be quadratic in $\delta x_t$ and $\delta u_t$ which essentially penalizes the controller for deviations away from the nominal trajectory.
As such, we can solve the same LQR problem described above but with the error states, controls, dynamics, and cost.
As a result, we will get the optimal control

$$
\delta u_t^* = -K_t \delta x_t
$$

But recall that $\delta u_t = u_t - \bar{u}_t$ and $u_t$ is actually the control we execute on the system, and $\bar{u}_t$ is the nominal control (known *a priori*). Thus the control we should feed to the system is

$$
u_t = \bar{u}_t + \delta u_t^* = \bar{u}_t - K_t\delta x_t
$$




## Iterative LQR
(TODO)
