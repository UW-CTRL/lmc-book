# HJ reachability

In this chapter, we will introduce reachability analysis, and show that we can frame reachability as an optimal control problem and utilize the same machinery discussed in the previous chapter.


## Reachability analysis
Reachability analysis, as the name suggests, is the study of the set of states a system can (or can't) reach within some fixed time. Generally, we refer to this set of states as a *reachable set*. The reachable set depends on a number of factors including
- the dynamics of the system
- initial conditions
- set of controls
- disturbances affecting the system

Reachability analysis is useful for a number of applications such as
- safety-critical control: knowing whether it is possible for the system to hit an obstacle is important, and if so, evasive control should be executed.
- robust control: if there is uncertainty or disturbances present, it is important to know where the system could possible reach and making sure that it does not intersect with obstacles.
- verification: computing the reachable sets based on the dynamics and assumptions about the system can help provide a certificate or guarantees on whether a system can reach a goal or can avoid obstacles.

### Types of reachability
There are two types of reachability problem: Forward reachability and backward reachability.

#### Forward reachability
Given system dynamics, and the set of all allowable controls it can take. Then the *forward reachable set* is the set of the states the system could be in at some future time.
Or differently phrased, starting from here, under any allowed input, where could the system end up?
Let's define this mathematically.
Let $\mathbb{U}[0,t]:=\lbrace u \mid [0,t] \rightarrow \mathcal{U} \rbrace$ be the set of all possible control signals over time interval $[0,t]$. Given dynamics $\dot{x} = f(x,u,t)$, let $\xi_{x_0, 0}^{u(\cdot)}(\tau) = x(\tau)$ be the state the system will be in at time $t=\tau$ if starting at $x_0$ and $t=0$ and executes some control signal $u(\cdot)\in\mathbb{U}[0,\tau]$.

Then we define the forward reachable set $\mathcal{F}(x_0, \tau)$ as follows,

$$\mathcal{F}(x_0, \tau) = \lbrace x \in \mathcal{X} \mid  \exists u(\cdot) \in \mathbb{U}[0, t], \: x = \xi_{x_0,0}^{u(\cdot)}(\tau) \rbrace$$

which translates to, all the states in the domain where there exists a control signal that can make the system be in that state within time $\tau$.


#### Backward reachability
Given a *target set*, the backrward reachable set* is the set of states where it is possible reach that set at some future time.
Or differently phrased, where must the system be now so that it can reach a target set in the future?
Let's define this mathematically.
Let $\mathcal{T}$ be a target set that the system is interested in reaching. Then for a system starting at $t=0$ and would like to reach $\mathcal{T}$ at $t=\tau$, then the *backward reachable set* is


```{admonition} Backward Reachable Set (Reach Case)
```{math}
:label: eq-BRS-reach
\mathcal{R}(\mathcal{T}, \tau) = \lbrace x_0\in\mathcal{X} \mid \exists u(\cdot) \in \mathbb{U}[0,\tau] \: \text{s.t.} \: \xi_{x_0, 0}^{u(\cdot)}(\tau) \in \mathcal{T} \rbrace.

```


This translates to, for all initial states in the domain, there exists a control signal where it will lead the system to be inside $\mathcal{T}$ at time $t=\tau$.

The above assume that reaching the target set $\mathcal{T}$ is a good thing. That is, we are seeking whether there exists *at least* one control signal that would get us there in $\tau$ time. As such, we use $\mathcal{R}$ to denote the intention to **r**each $\mathcal{T}$.



However, we can also treat $\mathcal{T}$ as an undesirable set (e.g., collision set), and ask the question whether it is possible to *avoid* $\mathcal{T}$ in the future. To check whether it is possible to avoid $\mathcal{T}$, we can equivalently determine whether *all* control signals would lead to the system being inside $\mathcal{T}$. Obviously, being inside this set would be undesirable because it would mean that all control signals would always lead the system to end up in $\mathcal{T}$, i.e., there is nothing that can be done to avoid entering $\mathcal{T}$.
We can compute this set in a similar way as above. We refer to this as the *avoid* set and use $\mathcal{A}$ to denote it. This avoid set is also often referred to as the *Inevitable Collision Set (ICS)* if $\mathcal{T}$ represents a collision set.

```{admonition} Backward Reachable Set (Avoid Case)
```{math}
:label: eq-BRS-avoid
\mathcal{A}(\mathcal{T}, \tau) = \lbrace x_0\in\mathcal{X} \mid \forall u(\cdot) \in \mathbb{U}[0,\tau] \: \text{s.t.} \: \xi_{x_0, 0}^{u(\cdot)}(\tau) \in \mathcal{T} \rbrace

```

##### Backward reachable tubes
Great! We just defined these sets that determine whether is it possible reach a target set (reach case), and whether it is impossible to avoid the target set (avoid case).
However, the above definitions only considers whether the system is inside the target set only at the final time $\tau$. It is possible that the system enters the target set before $t<\tau$ but exits it and is outside of $\mathcal{T}$ at $t=\tau$.

So we should also define the sets to consider entry into the target set $\mathcal{T}$ *anytime* between $t=0$ and $t=\tau$.
These sets are referred to as *Backward Reachable Tubes (BRT)* and had a very similar definition as above, but considers additional entry into $\mathcal{T}$ for any time $s\in[0,\tau]$.



```{admonition} Backward Reachable Tube (Reach Case)
```{math}
:label: eq-BRS-reach
\widetilde{\mathcal{R}}(\mathcal{T}, \tau) = \lbrace x_0\in\mathcal{X} \mid  \exists u(\cdot) \in \mathbb{U}[0,\tau],\: \exists s\in[0,\tau], \: \text{s.t.} \: \xi_{x_0, 0}^{u(\cdot)}(s)) \in \mathcal{T} \rbrace.

```

```{admonition} Backward Reachable Tube (Avoid Case)
```{math}
:label: eq-BRS-avoid
\widetilde{\mathcal{A}}(\mathcal{T}, \tau) = \lbrace x_0\in\mathcal{X} \mid  \forall u(\cdot) \in \mathbb{U}[0,\tau], \: \exists s\in[0,\tau], \: \text{s.t.} \: \xi_{x_0, 0}^{u(\cdot)}(s) \in \mathcal{T} \rbrace

```


### Computing BRS
Great! We just defined what backward reachable sets and backward reachable tubes are. But how do we go about actually computing them?
Let's focus our attention to BRS for the moment. We are concerned with whether we end up in $\mathcal{T}$ at the end of the time horizon, and we must reason *backward* in time to determine what is the set of states that would lead to entry into $\mathcal{T}$ in the future.
This aspect of reasoning *backward* in time strongly hints that we need to perform dynamic programming to determine the BRS. In fact, we can pose this reachability problem as an *optimal control problem* and solve it using the HJB equation.

Note that in the BRS definition, we only care if we enter $\mathcal{T}$ at the end of the horizon and nothing else. We don't care about the total control effort used. With this idea, we can define the *terminal cost* based on the $\mathcal{T}$.
Let us define a terminal cost function $g:\mathbb{R}^n \rightarrow \mathbb{R}$ such that,

$$
\mathcal{T} = \lbrace x\in\mathcal{X} \mid g(x) \leq 0 \rbrace
$$

which means that the $g(x)$ is negative if $x\in\mathcal{T}$, or $g(x)=0$ if $x\in\partial\mathcal{T}$ (i.e., on the boundary of $\mathcal{T}$).

And we also set the running cost (or stage cost) to be zero---again, we are just concerned about whether the system ends up in $\mathcal{T}$ at the end of the horizon, and nothing about the control effort or state error during the horizon. Below is an optimal control problem with the described cost structure, and no additonal constraints aside from ones constraining the dynamics and allowable state/control sets.


$$
\min_{u(\cdot) \in \mathbb{U}[0,\tau]} &\: J_T(x(T))\\
\text{subject to} &\: \dot{x} = f(x,u,t)\\
                  &\: u(t) \in \mathcal{U}, x(t) \in \mathcal{X}
$$

With this cost structure, note that finding an optimal control policy that *minimizes* the cost is equivalent to finding an optimal control policy that drives the system into $\mathcal{T}$.
In fact, if $J_T$ is defined as the *signed distance function* of $\mathcal{T}$, then the more negative $J_T(x)$, the further *inside* $x$ is of $\mathcal{T}$ and the more positive $J_T(x)$ is, the further *outside* $x$ is from $\mathcal{T}$. Then the optimal policy would driven the system as far inside of $\mathcal{T}$ as possible.



Recall that the (optimal) value function measures the future accumulated cost associated with state $x$ if the system follows the (optimal) policy. With this particular choice of cost structure described above (zero running cost and terminal cost to have a zero sub-level set be $\mathcal{T}$), the value function becomes:

$$
V^*(x,t) = \min_{u(\cdot) \in \mathbb{U}[0,\tau]}J_T(\xi_{x,t}^{u(\cdot)}(\tau)), \qquad \pi^*(\cdot) = \mathrm{arg}\min_{u(\cdot)  \in \mathbb{U}[0,\tau]}J_T(\xi_{x,t}^{u(\cdot)}(\tau))
$$

which is simply the terminal cost evaluated at the state at time $\tau$ if starting from state $x$, at time $t$, and following policy $\pi^*$. Since $J_T(x) \leq 0$ implies $x\in \mathcal{T}$, this means that if $V(x,t) \leq 0$, then following the optimal policy from state $x$ and time $t$ will lead the system to be inside $\mathcal{T}$ at time $\tau$ (i.e., at the end of the horizon).
As such, this means that the zero sub-level set of $V(\cdot, t)$ is precisely the BRS for a horizon of $\tau - t$! Concretely, we have,

```{admonition} Backward Reachable Set as a Value Function (Reach Case)
```{math}
:label: eq-BRS-reach-value
\mathcal{R}(\mathcal{T}, \tau - t) = \lbrace x\in \mathcal{X} \mid V^*(x,t) \leq 0 \rbrace, \qquad \text{where the planning horizon is }\tau

```

It may be easier to set the end of the time horizon to be $t=0$ and since we are stepping backwards in time, we use $t=-\tau$ to indicate $\tau$ time units back in time. So we can also write $V(x,-\tau)$ to indicate the BRS of a time horizon of $\tau$.

```{admonition} Backward Reachable Set as a Value Function (reparameterizing time) (Reach Case)
```{math}
:label: eq-BRS-reach-value
\mathcal{R}(\mathcal{T}, \tau) = \lbrace x\in \mathcal{X} \mid V^*(x,-\tau) \leq 0 \rbrace

```

What about the avoid case? Recall that the BRS for the avoid case is to determine the set of states where it is impossible to avoid $\mathcal{T}$ despite the system's best effort.
With the same cost described above, if the system instead aimed to *maximize* the terminal cost, then it is trying its best to avoid $\mathcal{T}$, but if $V^*(x,t) \leq 0$ it means that the system will end up inside $\mathcal{T}$ (despite the system's best effort).

As such, the BRS for the avoid case is the same as the reach case, except that we are performing a *maximization* problem instead of a minimization one.


Great! Now we have now posed the backward reachability problem as an optimal control problem and discussed how to interpret the value function. So how do we go about solving for the value function? We solve the Hamilton-Jacboi-Bellman equation! But the stage cost is set to zero.

```{admonition} HJB equation for BRS
```{math}
:label: eq-BRS-reach-avoid-value

\frac{\partial V}{\partial t}(x,t) + \min_{u\in\mathcal{U}} \nabla V(x,t)^T f(x,u,t) = 0 \qquad \text{Reach case}\\
\frac{\partial V}{\partial t}(x,t) + \max_{u\in\mathcal{U}} \nabla V(x,t)^T f(x,u,t) = 0 \qquad \text{Avoid case}\\
V(x,0) = J_T(x) \qquad t\in[-\tau, 0]\\

```

### Computing BRT
What about the backward reachable tube? What want to check whether the system enters $\mathcal{T}$ *any* time during the horizon. As such, we don't want to compute the terminal cost at the end of the horizon, but rather, determine the lowest value anytime over the horizon. Mathematically, the value function becomes,

$$
V^*(x,t) = \min_{u(\cdot) \in \mathbb{U}[0,\tau]} \min_{s\in[0,\tau]} J_T(\xi_{x,t}^{u(\cdot)}(s)), \qquad \pi^*(\cdot) = \mathrm{arg}\min_{u(\cdot)  \in \mathbb{U}[0,\tau]} \min_{s\in[0,\tau]} J_T(\xi_{x,t}^{u(\cdot)}(s)).
$$

We change the HJB PDE slightly to reflect the minimization over time,

```{admonition} HJB equation for BRT
```{math}
:label: eq-BRT-reach-avoid-value

\frac{\partial V}{\partial t}(x,t) + \min\biggl(0, \min_{u\in\mathcal{U}} \nabla V(x,t)^T f(x,u,t)\biggl) = 0 \qquad \text{Reach case}\\
\frac{\partial V}{\partial t}(x,t) + \min\biggl(0, \max_{u\in\mathcal{U}} \nabla V(x,t)^T f(x,u,t)\biggl) \qquad \text{Avoid case}\\
V(x,0) = J_T(x) \qquad t\in[-\tau, 0],\\

```

which looks very similar to the standard HJB equation, except there is a $\min$ with zero. Intuitively, if $\min_{u\in\mathcal{U}} \nabla V(x,t)^T f(x,u,t) > 0 $ (or $\max$), it means the system is moving in a direction where $V$ increases (i.e., moving *away* from $\mathcal{T}$). Since we want to find the *minimum* value of $V$ over the horizon, whenever $V$ is increasing, we want to "freeze" the value of $V$, essentially forcing $\frac{\partial V}{\partial t}=0$.
Alternatively, if $\min_{u\in\mathcal{U}} \nabla V(x,t)^T f(x,u,t) < 0 $, it means the system is moving towards $\mathcal{T}$ and we want to keep track of this and not "freeze" the value of $V$.

### Presence of disturbances
What if now there are disturbances added to the system? Assuming our dynamics are now $\dot{x} = f(x,u,d)$ and there is an external disturbance input $d$ that affects the system. Then we want to ensure that it is still possible to reach the target set $\mathcal{T}$, or whether it can still be avoided.

For the *reach* case, we want to consider whether the system can still reach $\mathcal{T}$ *regardless of any* disturbances possible. This means that the BRS/BRT should be defined such that the system can reach $\mathcal{T}$ *for all* disturbances.


```{admonition} Backward Reachable Set/Tube with Disturbances (Reach Case)
```{math}
:label: eq-BRS/BRT-disturbance-reach
\mathcal{R}_d(\mathcal{T}, \tau) = \lbrace x_0\in\mathcal{X} \mid \forall d(\cdot) \in \mathbb{D}[0, \tau],\: \exists u(\cdot) \in \mathbb{U}[0,\tau] \: \text{s.t.} \: \xi_{x_0, 0}^{u(\cdot)}(\tau) \in \mathcal{T} \rbrace.

\widetilde{\mathcal{R}}_d(\mathcal{T}, \tau) = \lbrace x_0\in\mathcal{X} \mid \forall d(\cdot) \in \mathbb{D}[0, \tau],\: \exists u(\cdot) \in \mathbb{U}[0,\tau],\: \exists s\in[0,\tau], \: \text{s.t.} \: \xi_{x_0, 0}^{u(\cdot)}(s)) \in \mathcal{T} \rbrace.

```

For the *avoid* case, we want to consider whether there is at least one disturbance that would always lead the system into $\mathcal{T}$ regardless of any possible control signal. In other words, the BRS/BRT is considering whether there is a disturbance signal that makes it impossible for the system to about $\mathcal{T}$.
This means that the BRS/BRT should be defined such that there exists a disturbance signal such that the system will reach $\mathcal{T}$ regardless of any control signals.


```{admonition} Backward Reachable Set/Tube with Disturbances (Avoid Case)
```{math}
:label: eq-BRS/BRT-disturbance-avoid
\mathcal{A}_d(\mathcal{T}, \tau) = \lbrace x_0\in\mathcal{X} \mid \exists d(\cdot) \in \mathbb{D}[0, \tau],\: \forall u(\cdot) \in \mathbb{U}[0,\tau] \: \text{s.t.} \: \xi_{x_0, 0}^{u(\cdot)}(\tau) \in \mathcal{T} \rbrace.

\widetilde{\mathcal{A}}_d(\mathcal{T}, \tau) = \lbrace x_0\in\mathcal{X} \mid \exists d(\cdot) \in \mathbb{D}[0, \tau],\: \forall u(\cdot) \in \mathbb{U}[0,\tau],\: \exists s\in[0,\tau], \: \text{s.t.} \: \xi_{x_0, 0}^{u(\cdot)}(s) \in \mathcal{T} \rbrace.

```

So now, we have updated the definition of BRS/BRT with disturbances present.
We can similarly update the HJB PDE to include the disturbance term.
Since the disturbance is acting in a manner that tries to oppose the system's best effort to reach/avoid $\mathcal{T}$, if the system is trying perform a $\max$, then the disturbance will perform a $\min$, and vice versa.
But there is a question as to which order we should compute this sequence of operations.
Or differently put, what kind of information pattern we should assume. Does the system get to see what the disturbance is first when deciding the optimal control, or the other way around?
Generally, we are using reachability analysis to be robust against disturbances/uncertainties, and therefore would like to take on a more conservative stance. That is, we would like to choose the ordering that would assume that the information pattern would benefit the disturbance.


There is a max-min inequality that describes that the player acting second has the advantage.
```{margin} Ordering
To interpret $\max_{z\in Z} \min_{w\in W} g(z,w)$, we can consider it as $\max_{z\in Z} \biggl(\min_{w\in W} g(z,w)\biggl)$. So when $\max_{z\in Z}$ is performed (i.e., going first), the $\min_{w\in W}$ will be applied after (i.e., going second).
```

```{admonition} Max-min inequality
```{math}
\sup_{z\in Z} \inf_{w\in W} f(z,w) \leq \inf_{w\in W} \sup_{z \ in Z} f(z,w)
```

With this max-min inequality, we assume the system chooses a control *first*, and then the disturbance will act second. With the addition of the disturbance term, the HJB equation becomes the *Hamilton-Jacboi-Isaacs* PDE.


```{admonition} HJI equation for BRS
```{math}
:label: eq-BRS-reach-avoid-value-HJI

\frac{\partial V}{\partial t}(x,t) + \min_{u\in\mathcal{U}}\max_{d\in\mathcal{D}} \nabla V(x,t)^T f(x,u,d,t) = 0 \qquad \text{Reach case}\\
\frac{\partial V}{\partial t}(x,t) + \max_{u\in\mathcal{U}}\min_{d\in\mathcal{D}} \nabla V(x,t)^T f(x,u,d,t)\qquad \text{Avoid case}\\
V(x,0) = J_T(x) \qquad t\in[-\tau, 0],\\

```

```{admonition} HJI equation for BRT
```{math}
:label: eq-BRT-reach-avoid-value-HJI

\frac{\partial V}{\partial t}(x,t) + \min\biggl(0, \min_{u\in\mathcal{U}}\max_{d\in\mathcal{D}} \nabla V(x,t)^T f(x,u,d,t)\biggl) = 0 \qquad \text{Reach case}\\
\frac{\partial V}{\partial t}(x,t) + \min\biggl(0, \max_{u\in\mathcal{U}}\min_{d\in\mathcal{D}} \nabla V(x,t)^T f(x,u,d,t)\biggl) \qquad \text{Avoid case}\\
V(x,0) = J_T(x) \qquad t\in[-\tau, 0],\\

```

## HJ reachability toolbox

This [`hj_reachability`](https://github.com/StanfordASL/hj_reachability) toolbox can be used to solve the corresponding HJI equation. The repo has a nice `quickstart` notebook. Also check out [this demo notebook](../examples/hj_reachability_basics.ipynb)

## A few remarks and comments

- It is assumed that the control and disturbance sets are *bounded*. The formulation considers a *worst-case* analysis on the disturbance input. As such, given a bounded disturbance set, the HJ reachability formulation will consider the worst disturbance in that set, even if it may be unlikely that disturbance will occur. This resulting computing would be conservative, but perhaps *too* conservative. The size of the sets could be reduced, but then that would compromise on the robustness of the computed BRS/BRT.

- If the dynamics are control and disturbance affine $\dot{x} = f_0(x) + B_u(x)u + B_d(x)d$, and the control and disturbances sets are bounded, then computing the $\min$/$\max$ is relatively straight forward. Simply determine the sign of $\nabla V(x,t)^TB_u(x)$ and $\nabla V(x,t)^TB_d(x)$ and consider the largest/smallest values of $u$ and $d$ depending on whether you want $\nabla V(x,t)^TB_u(x)u$ and $\nabla V(x,t)^TB_d(x)d$ to be maximized or minimized. The resulting optimal control is *bang-bang*. This is not surprising since we have zero running cost, there is not penalty on control effort. So the optimal thing to do to reach/avoid $\mathcal{T}$ would be to take the most extreme action, always.

- If the dynamics and control/disturbance sets are not nice (e.g., not affine, non-convex etc), then solving the HJB/HJI PDE involves solving a difficult optimization problem.

- The `hj_reachability` toolbox solves the PDEs explicity. That is, it discretizes the state space (i.e., creates a grid over the state space) and solves the PDE by considering each grid point. This explicit approach suffers from the curse of dimensionality---as the state dimension of the problem grows, the more grid points the solver must consider.

## TODOs
- add figures
- add papers
- add further readings
- add reach-avoid


<!-- Open loop because:
Assume all possible control inputs are applied without correction along the way. I.e., no feedback.



So far, we have formulated a sequential decision-making problem (e.g., a trajectory optimization problem), and studied how we can solve for the optimal policy by introducing a notion of a *value function* and solving for the value and policy for all states and time ahead of time. Online, we can simply use the value function to evaluate "how good" a state is at any point in time, and also "look up" the optimal control/action to take.

We saw how we can frame solving for the value function reduces to a dynamic programming problem, and derived two key equations corresponding to a discrete and continuous time setting, the Bellman equation and Hamilton-Jacobi-Bellman equation, respectively.

The value function carries an important interpretation---the further accumulated cost from the current time $t$ until the end of the horizon, potentially an infinite horizon.


 -->



<!-- Consider now, we are interested in *reaching* a target set $\mathcal{T}$. We are only concerned about whether we reach $\mathcal{T}$ at the end of the time horizon (assuming finite horizon), and -->


<!-- ```{admonition} Bellman Equation (discrete time, finite horizon)
```{math}
:label: eq-bellman
&V^*(x_{t},t) = \min_{u_{t}} \biggl( J(x_{t}, u_{t}, t) + V^*(x_{t+1}, t+1) \biggl), \qquad \text{where} \quad x_{t+1} = f(x_t, u_t, t)\\
&\pi^*(x_{t},t) = \mathrm{arg}\min_{u_{t}} \biggl( J(x_{t}, u_{t}, t) + V^*(x_{t+1}, t+1) \biggl)
```

```{admonition} Hamilton-Jacobi-Bellman Equation (continuous time, finite horizon)
```{math}
:label: eq-bellman
&V^*(x_{t},t) = \min_{u_{t}} \biggl( J(x_{t}, u_{t}, t) + V^*(x_{t+1}, t+1) \biggl), \qquad \text{where} \quad x_{t+1} = f(x_t, u_t, t)\\
&\pi^*(x_{t},t) = \mathrm{arg}\min_{u_{t}} \biggl( J(x_{t}, u_{t}, t) + V^*(x_{t+1}, t+1) \biggl)
```

R -->
