# Introduction to optimization

```{margin} Algorithms for Optimization
The textbook [Algorithms for Optimization](https://mykel.kochenderfer.com/textbooks/) by Mykel Kochenderfer and Tim A. Wheeler offers a comprehensive introduction to optimization with a focus on practical algorithms.
```

In this chapter, we give a brief overview of mathematical optimization and highlight ways we can go about solving them.

At a high-level, optimization is the process of finding the best solution to a problem by minimizing (or maximizing) an objective function while satisfying constraints.
Humans are constantly solving some form of optimization problem in everyday activities. For example, to get to class on time, you may want to minimize the distance traveled while avoiding high-traffic areas and avoid stairs.
In aerospace engineering, you want to minimize the weight and cost of an aircraft design (e.g., wing shape, fuselage size, engine placement) while ensuring the aircraft is stable, exhibits desired aerodynamics properties, and can withstand the forces during operations.
In the context of controls, you want to pick the best sequence of control inputs that minimized fuel usage over the time while reaching the goal, avoiding obstacles, and obeying the system dynamics.

While it may be simple to describe the optimization problem in words, ultimately we need to represent the problem mathematically and in such a way that can be tractably solved.

NOTE: We won't be diving deep into optimization theory and solvers in this course. Instead, we will learn how to frame control synthesis problems as optimization problems and in such as way that it is tractable (i.e., convex) to solve using off-the-shelf solvers/packages (e.g., `cvxpy`).





## Unconstrained optimization

### Mathematical description
First let's consider an **unconstrained optimization problem**.
An (unconstrained) optimization problem consists of two main elements: $x\in\mathbb{R}^n$ is the *decision variable*, $n$ number of variables whose values are to be determined, and $f: \mathbb{R}^n \rightarrow \mathbb{R}$ an *objective function* that determines how desirable $x$ is. Supposing that $f$ describes the *cost*, or how expensive $x$ is, then naturally we want to choose $x$ that *minimizes* $f$.
As such, the mathematical optimization problem becomes:

$$\min_x \: f(x), \qquad x^\star = \underset{x}{\mathrm{argmin}} \: f(x)$$

where the first equation describes the minimum value of $f$ over the variable $x$, while the second equation is describing the value of the *argument* that minimizes $f$. Typically the optimal decision variable, i.e., the *minimizer* is denoted by a superscript $\star$.

### Solving unconstrained optimization problems
Depending on the properties of $f$, finding the optimal solution can either be very straightforward, or very hard or expensive.
If $f$ is "nice" and differentiable, then we can simply apply gradient descent to find the stationary points.
If $f$ is not differentiable, or evaluating $f$ is an expensive process (e.g., running expensive simulations such as computation fluid dynamics), then we would likely need to use *derivative-free* techniques and computing the optimal solution can be very difficult.



### Gradient descent
One of the most popular and simplest approach is gradient descent.
If $f$ is nice smooth differentiable function where it is possible, and hopefully cheap, to compute $\nabla f(x)$, the *gradient* of $f$, then we can simply update our value of $x$ by moving moving in the direction of steepest descent.
Mathematically, if $\alpha$ is a step size, and $x_k$ is the current guess for the optimal solution, then we can update our guess by applying the update rule,

$$
x_{k+1} = x_k - \alpha \nabla f(x)
$$


```{admonition} Pause and think
What are some practical considerations when performing gradient descent?
```

```{margin} Convex Optimization
The textbook [Convex Optimization](https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf) by Lieven Vandenberghe and Stephen P. Boyd is the go-to text for learning about convex optimization. Lectures of 2023 Stanford EE 364a [course offering are on Youtube](https://youtu.be/kV1ru-Inzl4?si=S2Nf1e1gu_M6H2zq).
```

```{admonition} Practical considerations
:class: dropdown


We need to be careful about how to pick $\alpha$; if it is too small, then you may need to take many steps, but if it's too big, you may struggle to converge to a (local) optimum.
We can perform a [line search](https://optimization.cbe.cornell.edu/index.php?title=Line_search_methods) where you search along the steepest descent direction to find a suitable step size.
There are more advanced techniques that consider the *momentum** that can adaptively change $\alpha$ based on the geometry of $f$. We won't discuss further here, but [Algorithms for Optimization](https://mykel.kochenderfer.com/textbooks/) touches on this and has some useful references.



Just because the objective is differentiable does not mean you will always find the *globally* optimal solution. The objective could have many local optima, meaning even if you converge to a value $x_1$ where $\nabla f(x_1) = 0$, there could be another value $x_2$ where $f(x_2) \leq f(x_1)$.




However, if $f$ is *convex*, loosely speaking, "bowl-shaped", then that implies that any minimum found is the global minimum. We won't go into too much detail into convex optimization, but we will use the fact later on that *quadratic* and *linear* functions are convex.
But if your problem is convex, then generally (in most cases), this is tractable to solve, and there are many well-supported tools and solvers for solving convex optimization problems. In this course, we will use [`cvxpy`](https://www.cvxpy.org/).
```



### Derivative free
What if you can't take gradients, or that you don't want to because it's very expensive or difficult to?! Well, there are many other methods that don't rely on gradient information at all.
A common approach is a sampling-based, or population-based approach where you sample your search space to get a sense of the objective landscape, and from that, you can resample or refine your search towards more promising areas.
Popular methods include cross-entropy method, simulated annealing, and genetic algorithms.



## Constrained optimization
Suppose now that there are some values of $x$ that are not allowed. For example, a negative value may be physically impossible or undesirable, such as negative weight. Or that some values of $x$ must be a certain value. Or more generally, there is a function $g:\mathbb{R}^n \rightarrow \mathbb{R}^{m_\leq}$ and another function $h:\mathbb{R}^n \rightarrow \mathbb{R}^{m_=}$ such that we require $g(x) \leq 0$ and $h(x) = 0$. Here, $m_\leq$ and $m_=$ denotes the number of inequality and equality constraints respectively.

With these constraints, we can also consider a **constrained optimization problem**,

$$
\min_x &  \quad f(x)\\
\text{subject to}&  \quad g(x) \leq 0\\
&   \quad h(x) = 0
$$

This becomes a hard problem to solve, and applying regular gradient descent is not going to work because you may descend towards a region where the constraints are violated.

```{admonition} Pause and think
What are some ways you could go about handling these constraints?
```

```{admonition} Handling constraints
:class: dropdown

There are a few ways to go about handling these constraints. Again, we won't be going into depth here, but we are briefly mentioning these approaches in case you want to read more about them.

- **Projected gradient descent**: Take a gradient descent step as normal, and then project the solution back to the closest feasible point in the feasible set. This projection may involve solving another optimization problem(!), but for certain geometries of the feasible set, the projection could be found in closed form.

- **Use the Lagrangian**: A special way to convert a constrained optimization problem into an unconstrained one. But this is performed in a specific way where *Lagrangian multipliers* are introduced, and the necessary conditions for optimality are given by the *Karush–Kuhn–Tucker (KKT) conditions*. There are duality connections between the Lagrangian (dual problem) and the original problem (primal problem), where solving the dual problem gives you insight about the primal problem.

- **Treat constraints as (big) penalties in the objective**: We can add the constraint functions as part of the objective function so that there is a high cost then the constraints are violated. This turns the constrained problem into an unconstrained one, but then you have to carefully tune the weightings on the constraints, and there is no guarantee that the constraints will be perfectly satisfied.

- **Log-barrier**: A version of the approach above except that a log function is applied on the constraint so that the cost approaches infinity as $x$ approaches the infeasible region. See homework 1.
```



A reminder again, in this course, we will focus on framing control problems as optimization problems and ensure that the optimization problem is formulated such that we directly use off-the-shelf optimizers.


## Optimization solvers

There are many optimization solvers out there. Note there there are some packages that are *modeling languages*, that is, a nice intuitive wrapper for users to define an optimization problem in a natural way that follows the math, rather than in the restrictive standard form required by solvers. Then the user can select different backend solvers. While other packages are the actualy solvers and they come with their own interface.
Others are simply a function you can use as part of a toolbox.
In this class, we will mainly be using `cvxpy` as it is designed to be very intuitive and simple to use. Though for your project, you are welcome to use whatever solver/language/function you wish.



Here’s a list of **optimization solvers** and **modeling languages**, including links and short descriptions for each.
### **🔹 Optimization solvers**

#### **Free & open-source solvers**
- **[CVXOPT](https://cvxopt.org/)** – Convex optimization library in Python, supports linear and quadratic programming.
- **[SciPy Optimize](https://docs.scipy.org/doc/scipy/reference/optimize.html)** – General-purpose optimization module in SciPy (Python).
- **[OSQP](https://osqp.org/)** – Quadratic programming solver designed for fast embedded optimization.
- **[GLPK (GNU Linear Programming Kit)](https://www.gnu.org/software/glpk/)** – Solves large-scale linear programming (LP) and mixed-integer programming (MILP) problems.
- **[Ipopt](https://coin-or.github.io/Ipopt/)** – Interior-point solver for nonlinear programming (NLP).
- **[CBC (Coin-or Branch and Cut)](https://github.com/coin-or/Cbc)** – Open-source MILP solver.
- **[Clp (Coin-or Linear Programming)](https://github.com/coin-or/Clp)** – Open-source LP solver.
- **[CasADi](https://web.casadi.org/)** – Symbolic framework for nonlinear optimization and optimal control.
- **[GEKKO](https://gekko.readthedocs.io/en/latest/)** – Python package for solving LP, NLP, and MILP problems.

---

#### **Commercial solvers (often more Powerful, but some have free academic licenses)**
- **[Gurobi](https://www.gurobi.com/)** – Industry-leading solver for LP, QP, and MILP. Free academic licenses available.
- **[CPLEX (IBM)](https://www.ibm.com/products/ilog-cplex-optimization-studio)** – Powerful solver for LP, MILP, and NLP. Academic licenses available.
- **[MOSEK](https://www.mosek.com/)** – Optimized for large-scale conic and linear problems.
- **[Knitro](https://www.artelys.com/knitro/)** – High-performance nonlinear optimization solver.
- **[MATLAB Optimization Toolbox](https://www.mathworks.com/products/optimization.html)** – Various solvers for LP, QP, and NLP problems.

---

### **🔹 Optimization modeling languages**

- **[CVXPY](https://www.cvxpy.org/)** – Python-based modeling framework for convex optimization.
- **[PuLP](https://coin-or.github.io/pulp/)** – Python library for linear programming modeling.
- **[Pyomo](http://www.pyomo.org/)** – Python-based modeling for LP, NLP, and MILP.
- **[JuMP](https://jump.dev/)** – High-performance optimization modeling in Julia.
- **[AMPL](https://ampl.com/)** – A powerful modeling language for mathematical programming.
- **[GAMS](https://www.gams.com/)** – General Algebraic Modeling System for LP, NLP, and MILP.
- **[YALMIP](https://yalmip.github.io/)** – Optimization modeling toolbox for MATLAB.
- **[CVX (MATLAB)](http://cvxr.com/cvx/)** – Convex optimization modeling in MATLAB.



### **🔹 Optimization functions within toolboxes**

Here’s a list of **optimization functions within toolboxes** for popular programming environments like MATLAB, SciPy, and Julia.

---

#### **🔹 Optimization functions in MATLAB** *(from Optimization Toolbox and other toolboxes)*

- **[fmincon](https://www.mathworks.com/help/optim/ug/fmincon.html)** – Solves constrained nonlinear optimization problems.
- **[linprog](https://www.mathworks.com/help/optim/ug/linprog.html)** – Linear programming solver for minimizing linear objectives subject to linear constraints.
- **[quadprog](https://www.mathworks.com/help/optim/ug/quadprog.html)** – Solves quadratic programming (QP) problems.
- **[fminunc](https://www.mathworks.com/help/optim/ug/fminunc.html)** – Unconstrained nonlinear optimization.
- **[lsqnonlin](https://www.mathworks.com/help/optim/ug/lsqnonlin.html)** – Solves nonlinear least-squares problems.
- **[ga](https://www.mathworks.com/help/gads/ga.html)** – Genetic algorithm solver for global optimization.
- **[particleswarm](https://www.mathworks.com/help/gads/particleswarm.html)** – Particle swarm optimization (PSO) for global optimization.
- **[simulannealbnd](https://www.mathworks.com/help/gads/simulannealbnd.html)** – Simulated annealing for constrained or unconstrained problems.

---

#### **🔹 Optimization functions in SciPy (Python)** *(from `scipy.optimize` module)*

- **[scipy.optimize.minimize](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html)** – General function for unconstrained/constrained minimization with multiple algorithms (BFGS, Nelder-Mead, SLSQP, etc.).
- **[scipy.optimize.linprog](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linprog.html)** – Linear programming solver.
- **[scipy.optimize.least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)** – Nonlinear least-squares solver.
- **[scipy.optimize.curve_fit](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.curve_fit.html)** – Curve fitting using nonlinear least squares.
- **[scipy.optimize.differential_evolution](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html)** – Global optimization using evolutionary algorithms.

---

### **🔹 Optimization unctions in Julia** *(from `JuMP` and `Optim.jl` packages)*

- **[`JuMP.optimize!`](https://jump.dev/JuMP.jl/stable/)** – Defines and solves optimization models.
- **[`Optim.optimize`](https://julianlsolvers.github.io/Optim.jl/stable/)** – General-purpose unconstrained optimization.
- **[`NLopt.optimize`](https://github.com/JuliaOpt/NLopt.jl)** – Interfaces nonlinear optimization methods in Julia.
- **[`BlackBoxOptim.optimize`](https://github.com/robertfeldt/BlackBoxOptim.jl)** – Black-box optimization algorithms like genetic algorithms.
