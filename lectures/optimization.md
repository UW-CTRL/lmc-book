# Introduction to optimization

```{margin} Algorithms for Optimization
The textbook [Algorithms for Optimization](https://mykel.kochenderfer.com/textbooks/) by Mykel Kochenderfer and Tim A. Wheeler offers a comprehensive introduction to optimization with a focus on practical algorithms.
```

In this chapter, we present a concise overview of mathematical optimization and introduce several strategies for approaching and solving optimization problems.

At its core, mathematical optimization involves finding the best solution to a problem by minimizing (or maximizing) an objective function, subject to a set of constraints. We frequently solve optimization problems in our daily lives, often without realizing it. For instance, when trying to get to class on time, you might seek the shortest path while avoiding traffic and stairways. In aerospace engineering, designers aim to minimize the weight and cost of an aircraft—adjusting factors like wing shape, fuselage size, and engine placement—while ensuring stability, aerodynamic efficiency, and structural integrity. Similarly, in control systems, we strive to select the sequence of control inputs that minimizes fuel consumption, reaches a desired goal, avoids obstacles, and respects the dynamics of the system.

Although optimization problems are often easy to describe in words, translating them into precise mathematical formulations—especially in a way that enables efficient computation—can be challenging.

**Note:** This course will not explore the full depth of optimization theory or solver algorithms. Our primary focus will be on expressing control synthesis problems as mathematical optimization problems, and in particular, reformulating them in ways (e.g., to make them convex) that permit efficient solution with modern, off-the-shelf solvers and packages such as `cvxpy`.



## Unconstrained Optimization

### Mathematical Formulation

Let us begin by considering an **unconstrained optimization problem**. Such a problem involves two principal components: 
- The *decision variable* $x \in \mathbb{R}^n$, representing the $n$ unknowns whose values we seek, and 
- The *objective function* $f: \mathbb{R}^n \rightarrow \mathbb{R}$, which assigns a numerical value (often representing *cost*) to each possible $x$.

If $f(x)$ represents the cost associated with a particular $x$, our aim is to select $x$ so as to *minimize* the cost. The problem is typically written as:

$$
\min_x \; f(x), \qquad x^\star = \arg\min_x f(x)
$$

The first expression seeks the minimum value of $f$ over all $x$, while the second denotes the *minimizer*: the particular value $x^\star$ attaining that minimum. Conventions often use the superscript $\star$ to indicate the optimal variable.

### Approaches to Solving Unconstrained Optimization Problems

The difficulty of solving an unconstrained optimization problem depends heavily on the properties of the objective function $f$. 

- If $f$ is *smooth* (differentiable) and has a structure amenable to analysis, techniques like gradient descent can efficiently find stationary points.
- However, if $f$ is non-differentiable, highly non-convex, or expensive to evaluate (e.g., each evaluation requires a costly simulation), then specialized *derivative-free* methods may be required, and obtaining an optimal solution can become substantially more challenging.



### Gradient Descent

One of the most popular and straightforward methods for solving unconstrained optimization problems is **gradient descent**. The key idea behind gradient descent is to iteratively improve your estimate of the optimal solution by moving in the direction of *steepest descent*—that is, the direction in which the function decreases most rapidly.

Suppose $f$ is a smooth (differentiable) function and it is feasible—ideally inexpensive—to compute its gradient, $\nabla f(x)$. Then, at each iteration, we can update our current guess $x_k$ by taking a small step $\alpha$ (the *step size*, sometimes called the *learning rate*) in the negative gradient direction:

$$
x_{k+1} = x_k - \alpha \nabla f(x_k)
$$

Here, $x_{k+1}$ becomes the new guess, and the process is repeated until convergence, or a maximum number of steps is reached.


```{admonition} Pause and think
What are some practical considerations when performing gradient descent?
```

```{margin} Convex Optimization
The textbook [Convex Optimization](https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf) by Lieven Vandenberghe and Stephen P. Boyd is the go-to text for learning about convex optimization. Lectures of 2023 Stanford EE 364a [course offering are on YouTube](https://youtu.be/kV1ru-Inzl4?si=S2Nf1e1gu_M6H2zq).
```

```{admonition} Practical considerations
:class: dropdown


Choosing the appropriate value for the step size $\alpha$ in gradient descent is crucial. If $\alpha$ is too small, the algorithm may require many iterations to make progress; if it is too large, you risk overshooting and may fail to converge to (even a local) minimum. 

A common strategy is to use a [line search](https://optimization.cbe.cornell.edu/index.php?title=Line_search_methods), which determines a suitable step size by searching along the descent direction to find an appropriate value of $\alpha$ that sufficiently reduces the objective. For further efficiency, more advanced methods—such as those incorporating *momentum*—can adaptively adjust $\alpha$ based on the geometry of $f$. While we won't delve into these techniques here, [Algorithms for Optimization](https://mykel.kochenderfer.com/textbooks/) provides an excellent introduction and additional references.

It is important to note that even if the objective function $f$ is differentiable, gradient descent may not always find the *global* optimum. The presence of multiple local minima means the algorithm might converge to a point $x_1$ with $\nabla f(x_1) = 0$, even though there could exist another point $x_2$ where $f(x_2) \leq f(x_1)$.

However, if $f$ is *convex*—that is, roughly "bowl-shaped"—any local minimum is guaranteed to be the global minimum. Convex optimization is a rich and well-developed area; in particular, quadratic and linear functions are convex, and we will take advantage of this fact later in the course. For convex problems, efficient and reliable algorithms exist, and a variety of well-supported tools and solvers are available. In this course, we will make use of [`cvxpy`](https://www.cvxpy.org/) for formulating and solving convex optimization problems.
```



### Derivative-free
What if you can't compute gradients, or doing so would be too expensive or difficult? In such cases, there are many alternative methods that do not require gradient information. 

A common strategy is to use sampling-based or population-based methods, where you explore the search space by evaluating the objective function at a set of sampled points. Based on these samples, you can iteratively refine your search—focusing more on promising regions of the space.

Popular derivative-free optimization methods include the cross-entropy method, simulated annealing, and genetic algorithms.


## Constrained optimization
Now, let’s consider situations where certain values of $x$ are inadmissible. For instance, negative values might be physically impossible or undesirable (such as negative weights), or some variables may need to be fixed at specific values. More generally, we can represent such requirements using two constraint functions: $g:\mathbb{R}^n \rightarrow \mathbb{R}^{m_\leq}$ for inequality constraints and $h:\mathbb{R}^n \rightarrow \mathbb{R}^{m_=}$ for equality constraints, where we require that $g(x) \leq 0$ and $h(x) = 0$. Here, $m_\leq$ and $m_=$ denote the number of inequality and equality constraints, respectively.

With these constraints in place, we arrive at the general **constrained optimization problem**:

$$
\min_x \quad f(x) \\\\
\text{subject to} \quad g(x) \leq 0 \\\\
\phantom{\text{subject to}} \quad h(x) = 0
$$

Solving constrained optimization problems is generally much more challenging than the unconstrained case. Standard gradient descent is typically insufficient, since the algorithm may step into infeasible regions that violate the constraints.

```{admonition} Pause and think
What are some effective strategies for handling constraints in optimization problems?
```

```{admonition} Handling constraints
:class: dropdown

There are several well-established methods for managing constraints in optimization. While we won’t dive deeply into the technical details here, it’s useful to be aware of the main approaches, especially if you wish to explore them further:

- **Projected Gradient Descent**: After each standard gradient descent step, project the updated solution back onto the feasible set defined by the constraints. This may require solving an additional optimization problem, though for certain constraint sets, the projection can often be computed explicitly and efficiently.

- **Lagrangian Methods**: Reformulate constrained problems as unconstrained ones by introducing *Lagrange multipliers*. The resulting optimality conditions are known as the *Karush–Kuhn–Tucker (KKT) conditions*. Solving the so-called dual problem may provide valuable insight into the structure of the original (primal) problem.

- **Penalty Methods**: Incorporate the constraint functions directly into the objective, adding a large penalty when constraints are violated. This effectively transforms the constrained problem into an unconstrained one, but requires careful tuning of the penalty terms, and feasibility of the resulting solution is not always guaranteed.

- **Log-Barrier Methods**: An alternative penalty approach where a logarithmic barrier is added for the constraints. As the solution approaches the boundary of the feasible set, the cost increases sharply (approaching infinity), thereby discouraging constraint violations. (See homework 1 for a practical example.)

```

As a final note, throughout this course we will emphasize formulating control problems as optimization problems, ensuring they are structured so that off-the-shelf optimization solvers can be applied directly and efficiently.



## Optimization solvers

There is a wide variety of optimization solvers available, each with its own strengths and interfaces. Some packages are *modeling languages*—user-friendly frameworks that allow you to define optimization problems in mathematical terms, freeing you from the rigid standard forms required by many solvers and enabling you to easily switch between different solver backends. Other packages are dedicated *solvers* with their own specific interfaces, while some environments offer optimization functionality through toolbox functions.

In this course, we will primarily use `cvxpy` because of its intuitive, math-like syntax and ease of use. However, for your project work, you are welcome to use any solver, modeling language, or library that best fits your needs or interests.




Below is a curated list of commonly used **optimization solvers** and **modeling languages**, with links and brief descriptions for each. Please note: while this list draws on various sources (including AI-assisted compilation), the teaching team has not comprehensively tested every tool listed.


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

### **🔹 Optimization functions in Julia** *(from `JuMP` and `Optim.jl` packages)*

- **[`JuMP.optimize!`](https://jump.dev/JuMP.jl/stable/)** – Defines and solves optimization models.
- **[`Optim.optimize`](https://julianlsolvers.github.io/Optim.jl/stable/)** – General-purpose unconstrained optimization.
- **[`NLopt.optimize`](https://github.com/JuliaOpt/NLopt.jl)** – Interfaces nonlinear optimization methods in Julia.
- **[`BlackBoxOptim.optimize`](https://github.com/robertfeldt/BlackBoxOptim.jl)** – Black-box optimization algorithms like genetic algorithms.
