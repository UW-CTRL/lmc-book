# State estimation

So far, we have assumed we had perfect state information. That is, we could observe the state perfectly. In practice, this isn't always true. For instance, we don't typically know the *true* position of a robot (relative to some frame). Instead, we take measurements, such as GPS signals, LiDAR range measurements, or measurements from a motion capture system. Using these measurements, we can back out, or *estimate* what the state could be. But again, this is only an *estimate* since the measurements are not perfect either, but some can be more accurate than others. That is to say, our measurement model is subject to *measurement noise*.


Not only is there uncertainty introduced through the measurements, but there is also uncertainty in the dynamics of the system. While we can derive a dynamics model that, in theory, describes the motion of the system, there are typically sources of uncertainty that make this dynamics model not perfect. For instance, there may be things that are not captured in the model (e.g., wind affecting an aircraft, ice on the road that affects a vehicle's tire friction). We refer to the uncertainty in the dynamics model as *process noise*.

Moreover, the measurements may only provide a signal to part of the state (e.g., GPS readings provide information on position but not velocity). So we need to also estimate part of the state that we may not be able to directly measure.

## Problem set up

Let us set up this state estimation problem more formally. To start off, let us consider a discrete-time setting.
Suppose we have a dynamical system with state vector $x_t\in \mathbb{R}^n$, control input $u_t\in\mathbb{R}^m$ and measurement $y_t\in \mathbb{R}^p$ at time step $t$. Let $u_0,u_1,\ldots,u_{t-1}$ denote a sequence of control inputs the system executes right before timestep $t$. This is assumed to be known (since we get to choose the controls to execute). And let $y_1, y_2,\ldots, y_t$ denote a sequence of measurements up until timestep $t$.

Then, the goal is to develop an algorithm that takes in $u_0,u_1,\ldots,u_{t-1}$ and $y_1, y_2,\ldots, y_t$ to estimate the state $x_t$ at timestep $t$ with some estimate $\hat{x}_t \sim p(\hat{x}_t \mid \text{past controls}, \text{past and current obsevations})$ such that the error between the true and estimated state $\| x_t - \hat{x}_t\|$ is "as small as possible".

In general, this estimation problem is a challenging optimization problem for a number of reasons. For example, the uncertainty in the measurement and dynamics model present itself in a very complicated way, such as being state-dependent, a distribution that isn't well described by standard distributions (e.g., Gaussian), the measurement and dynamics model are nonlinear, and the measurements are high-dimensional (e.g., an image).

There's been a lot of progress made, especially with deep learning techniques, to estimate the state (and also a map of the environment---this is referred to as Simultaneous Localization and Mapping, or SLAM).
But in this chapter, we will study a particular state estimation algorithm that is very tractable, simple, and powerful, despite certain simplifying assumptions it makes.

## Intuition
In the process of estimating the state, we assume we have (i) a dynamics model, (ii) a measurement model, and (iii) a description of of the process and measurement noise.
With the dynamics model, we could propagate the current state estimate through the dynamics and get a prediction of the next state. While with the measurement model, we could analyze a measurement and predict where the state could be that way. But which one should be trust more? The prediction from the dynamics model or the prediction from the measurement model?

Well, ideally both! And this depends on how much we should trust each of the models. The level of "trust" would depend on how much noise they each encompass. That is, we should consider the "relative magnitudes" of uncertainty in the process and measurement noise models. We have put "relative magnitudes" in quotes since it's not always straightforward to compare uncertainties given two arbitrary probability distributions.

At a high-level, a state estimation algorithm that we will dive into next has the following steps. Given the previous state estimate $\hat{x}_{t-1}$, we then perform:

1. **Prediction step**. Using the dynamics, *predict* where the next state should be according to ther dynamics model. But with the understanding that this is only a *prediction* since the true system is subject to process noise, and the exact noise quantity is not something we know or can measure.
2. **Receive a measurement $y_t$.** Compare what the predicted measurement would have been using the predicted next state and measurement model. This difference is a *measurement residual*.
3. **Update step**. Obtain an updated prediction of the next state by reasoning about the predicted state and the measurement residual, and figuring out an optimal way to fuse these two pieces of information together.

[TODO: add figure]

## The Kalman Filter

### Key assumptions
To make the state estimation procedure tractable, we need to make some simplying assumption. Not too surprisingly (given the name of this course), we will assume the **dynamics and measurement models are linear**.
In terms of simplifying assumptions on the noise, we make the following assumptions:
- The process and measurement noise is *additive*.
- The noise is *Gaussian white noise*, meaning the noise is uncorrelated across different time steps, and is drawn from a normal distribution with mean 0 and with some covariance to be set by user.

Mathematically, we assume the dynamics and measurement model have the following form:



```{admonition} Gaussian Linear Dynamics
```{math}
:label: eq-gaussian-linear-dynamics

&x_{t+1} = Ax_t + Bu_t + w_t, \qquad &\mathbb{E}[w_tw_t^T] = Q, \qquad &w_t \sim \mathcal{N}(0, Q)\\
&y_t = Cx_t + v_t, \qquad &\mathbb{E}[v_tv_t^T] = R, \qquad &v_t \sim \mathcal{N}(0, R)

```
Note: For further simplicity, we have assumed time-invariant dynamics, measurement, and noise. We can consider a time-variant setting by simply adding a subscript $t$ on the $A, B, C, Q, R$ matrices.



### Set up
Our estimate is not exactly a specific value, but rather is a *proability distribution* over $\hat{x}_t$. That is, we want to estimate the probability distribution $p$ where $\hat{x}_t \sim  p(\hat{x}_t \mid u_{0:t-1}, y_{1:t}, x_0)$.
But what does this probability distribution look like?
```{margin} Compare with LQR
This is analogous to LQR. By assuming the value function is quadratic, after appling the Bellman update, the value function at the next time step is also quadratic!
```
Well, it turns out that there is a reason why we chose Gaussian white noise and linear dynamics. When we apply an affine transformation to a Gaussian, the output is another Gaussian but with a different mean and covariance. So if we assume that our state estimate is represented by a Gaussian distribution. Then "passing" that estimate through the dynamics and measurement model will result in the updated estimate being Gaussian as well!

So let's assume that the initial state estimate is represented by a normal distribution with mean $\mu_0$ and covariance $P_0$, $\hat{x}_0 \sim \mathcal{N}(\mu_0, P_0)$. Here, $\mu_0$ and $P_0$ are chosen by the user. This represents an estimate of where the system's starting state.

<!-- ```{margin} Some properties of Gaussians
If $x\sim\mathcal{N}(\mu, \Sigma)$, and $z = c - x$ where $c$ is a constant, then $\mathbb{E}[z] = \mathbb{E}[c - x] = \mathbb{E}[c] - \mathbb{E}[x] = c - \mu$, since expectations is a linear operation. Also, $\mathrm{Cov}(aX + bY) = a^2 \mathrm{Cov}(X) + b^2 \mathrm{Cov}(Y)$.
``` -->
Let's also define the error at time $t$ to be $e_t = x_t - \hat{x}_t$ where $x_t$ is the *true* state which is an actual value but unknown to us. Since $\hat{x}_t\sim\mathcal{N}(\mu_t, P_t)$, then $e_t \sim \mathcal{N}(x_t - \mu_t, P_t)$. This is because shifting a Gaussian by a constant will only change the mean, but keep the covariance the same.
While knowing the mean of the error $x_t - \mu_t$ is not possible since $x_t$ is not known, knowing $P_t$ is useful (and later we show possible to compute) since this measures the *uncertainty in the error of our estimate*.
Specifically, we have $P_t = \mathbb{E}[e_te_t^T] = \mathbb{E}[(x_t - \mu_t)(x_t - \mu_t)^T]$ and so we have the diagonal entries describing the mean squared error for each entry of the state.
As such, we would like to minimize the uncertainty in our error estimate by *minimizing the trace of $P_t$*, a fact that we will use later.


Summarizing the ideas above:
- Initial state estimate $\hat{x}_0 \sim \mathcal{N}(\mu_0, P_0)$ chosen by user.
- Error estimate $e_t = x_t - \hat{x}_t$ where $\mathbb{E}[e_te_t^T] = P_t$.
- Want to minimizing the diagonal entries of $P_t$ at every time step.

So let's begin the estimation process where we first *predict* the next state, receive a measurement, and then fuse the measurement information with the prediction.

### The Kalman Filter steps

#### **1. Prediction step**
Given the previous estimate $\hat{x}_{t-1}$, we would like to predict what $\hat{x}_t$ is by considering only the dynamics without process noise. Since this is only a prediction and not our final estimate, we denote this prediction with a superscript $p$. So let's "pass" our estimate $\hat{x}_{t-1}$ through the linear dynamics.

$$
\hat{x}_t^p = A\hat{x}_{t-1} + Bu_{t-1}
$$

Since $\hat{x}_{t-1}$ is a random variable and is passed through an affine transformation, then $\hat{x}_t^p$ is also a random variable. If $\hat{x}_{t-1}\sim\mathcal{N}(\mu_{t-1}, P_{t-1})$, then $\hat{x}_t^p \sim\mathcal{N}(\mu_t^p, P_t^p)$, but with a different mean and covariance to be determined. Since we assume that $\hat{x}_0 \sim \mathcal{N}(\mu_0, P_0)$, and Gaussians remain Gaussians under affine transformations, then it follows that all $\hat{x}_t^p$ (and \hat{x}_t$) will also be normally distributed.

So we have $\hat{x}_{t-1}\sim\mathcal{N}(\mu_{t-1}, P_{t-1})$. Recalling that the expectation is a linear operator, then we have,

$$
\mathbb{E}[\hat{x}_t] &= \mathbb{E}[A\hat{x}_{t-1} + Bu_{t-1}]\\
&= \mathbb{E}[A\hat{x}_{t-1}] + \mathbb{E}[Bu_{t-1}] \\
&= A\mathbb{E}[\hat{x}_{t-1}] + Bu_{t-1}\\
\mu_t^p & = A\mu_{t-1} + Bu_{t-1}
$$

Next, we need to predict the covariance of the estimate $P_t^p$. Let the predicted error estimate be $e_t^p = x_t - \hat{x}_t^p$ and $P_t^p = \mathbb{E}[e_t^p {e_t^p}^T]$. Noting that the true state evolves according to $x_t = Ax_{t-1} + Bu_{t-1} + w_t$, we then have

$$
\mathbb{E}[e_t^p {e_t^p}^T] & = \mathbb{E}[(x_t - \hat{x}_t^p)(x_t - \hat{x}_t^p)^T]\\
&= \mathbb{E}[(Ax_{t-1} + Bu_{t-1} + w_t - A\hat{x}_{t-1} + Bu_{t-1})((Ax_{t-1} + Bu_{t-1} + w_t - A\hat{x}_{t-1} + Bu_{t-1}))^T]\\
&= \mathbb{E}[(Ax_{t-1}  + w_t - A\hat{x}_{t-1} )(Ax_{t-1}  + w_t - A\hat{x}_{t-1} )^T]\\
&= \mathbb{E}[(A(x_{t-1} - \hat{x}_{t-1})  + w_t )(A(x_{t-1} - \hat{x}_{t-1})  + w_t )^T]\\
&= \mathbb{E}[(Ae_{t-1}  + w_t )(Ae_{t-1}  + w_t )^T]\\
&= \mathbb{E}[Ae_{t-1}e_{t-1}^TA^T + Ae_{t-1}w_t^T + w_t e_{t-1}^TA^T + w_tw_t^T]\\
&= \mathbb{E}[Ae_{t-1}e_{t-1}^TA^T] + \mathbb{E}[Ae_{t-1}w_t^T] + \mathbb{E}[w_t e_{t-1}^TA^T] + \mathbb{E}[w_tw_t^T]\\
&= A\mathbb{E}[e_{t-1}e_{t-1}^T]A^T + A\mathbb{E}[e_{t-1}w_t^T] + \mathbb{E}[w_t e_{t-1}^T]A^T + \mathbb{E}[w_tw_t^T]\\
P_t^p&= AP_{t-1}A^T + Q
$$

Note that $\mathbb{E}[e_{t-1}w_t^T] = \mathbb{E}[w_t e_{t-1}^T]=0$ because the error at $t-1$ is independent of the noise that is happening at $t$.

Summarizing the prediction step, given the estimate from the previous time step $\hat{x}_{t-1} \sim \mathcal{N}(\mu_{t-1}, P_{t-1})$, we have:


```{admonition} Prediction step
```{math}
:label: eq-kf-prediction

\textbf{Predicted mean:}& \quad \mu_t^p = A\mu_{t-1} + Bu_{t-1}\\
\textbf{Predicted error covariance:}& \quad P_t^T = AP_{t-1}A^T + Q\\
\textbf{Predicted estimate:}& \quad \hat{x}_t^p \sim \mathcal{N}(\mu _t^p, P_t^p)\\
```


#### **2. Receive measurement**

We now receive a measurement $y_t$. We can compute the *measurement residual*, also referred to as the *innovation vector*, which is the error between the measurement and the predicted measurement using the predicted estimate $\hat{x}_t^p$.

$$h_t = y_t - C\hat{x}_t^p$$

#### **3. Update step**
We now *update* our predicted estimate and get our final state estimate $\hat{x}_t$.
Recall that we would like to *fuse* information from our prediction and measurement in some optimal manner.
The way we fuse these two pieces of information is as follows:

$$ \hat{x}_t = \hat{x}_t^p + K_t (y_t - C\hat{x}_t^p), \qquad \hat{x}_t \sim \mathcal{N}(\mu_t, P_t)$$

where $K_t$ is the **Kalman gain** which is a gain matrix which tradeoff our "trust" in the dynamics model and the measurement model. Intuitively, $K_t$ should consider the relative sizes of $Q$ and $R$, the uncertainty i the dynamics and measurement model. If there is more uncertainty in the dynamics then we should trust our measurement model more, and vice versa.


Notice that the mean $\mu_t$ and $P_t$ are updated since the superscript $p$ is dropped.
So we need to determine what $\mu_t$ and $P_t$ are.

Like in the prediction step, we have

$$\mu_t = \mu_t^p + K_t(y_t - Cu_t^p)$$

but $K_t$ is to be chosen such that $P_t$, which describes the uncertainty in our estimate, is made to be "as small as possible". This hints that some kind of optimization problem to be solved.
So let's first compute an expression for $P_t$.

From our definition of $P_t$, we have,

$$
P_t &= \mathbb{E}[e_te_t^T]\\
&= \mathbb{E}[(x_t - \hat{x}_t)(x_t - \hat{x}_t)^T]\\
&= \mathbb{E}[(x_t - \hat{x}_t^p - K_t (y_t - C\hat{x}_t^p))(x_t - \hat{x}_t^p - K_t (y_t - C\hat{x}_t^p))^T]\\
&= \mathbb{E}[(x_t - \hat{x}_t^p - K_t (Cx_t + v_t - C\hat{x}_t^p))(x_t - \hat{x}_t^p - K_t (Cx_t + v_t - C\hat{x}_t^p))^T]\\
&= \mathbb{E}[(I-K_tC)(x_t - \hat{x}_t^p)(x_t - \hat{x}_t^p)^T(I-K_tC)^T + K_tv_tv_t^TK_t^T]\\
&= (I-K_tC)\mathbb{E}[(x_t - \hat{x}_t^p)(x_t - \hat{x}_t^p)^T](I-K_tC)^T + K_t\mathbb{E}[v_tv_t^T]K_t^T\\
&= (I-K_tC)P_t^p(I-K_tC)^T + K_tRK_t^T\\
$$

We need to select $K_t$ such that the uncertainty of our estimate is minimized.
Note that the *diagonal entries* of $P_t$ represents the mean squared error of the estimate. As such, we want find $K_t$ that minimizes the *trace of $P_t$*.

$$
\min_{K_t} \mathrm{Tr}(P_t)
$$

After some algebra, and taking the derivative with respect to $K_t$ and setting it zero, we have,

$$
\mathrm{Tr}(P_t) &= \mathrm{Tr}(P_t^p) - 2\mathrm{Tr}(K_tCP_t^p) + \mathrm{Tr}(K_tCP_t^pC^TK_t^T) + \mathrm{Tr}(K_tRK_t)\\
\text{Given the following identities: } &\frac{d}{dA}\mathrm{Tr}(ABA^T) = 2AB, \: B=B^T, \qquad \frac{d}{dA}\mathrm{Tr}(AC) = C^T\\
\frac{d}{dK_t}\mathrm{Tr}(P_t) &= -2P_t^pC^T + 2K_tCP_t^pC^T + 2K_tR\\
0 &= -2P_t^pC^T + 2K_tCP_t^pC^T + 2K_tR\\
\Rightarrow & \quad K_t^* = P_t^pC^T(CP_t^pC^T + R)^{-1}
$$

Plugging $K_t^*$ back into the $P_t$ expression, we have:

$$
P_t = (I - K_t^*C)P_t^p, \: \text{where} \: K_t^* = P_t^pC^T(CP_t^pC^T + R)^{-1}
$$

Summarizing the update step, we have,

```{admonition} Update step
```{math}
:label: eq-kf-update
\textbf{Updated mean:}& \quad \mu_t = \mu_t^p + K_t(y_t - Cu_t^p)\\
\textbf{Kalman gain:}& \quad K_t = P_t^pC^T(CP_t^pC^T + R)^{-1}\\
\textbf{Updated error covariance:}& \quad P_t = (I - K_tC)P_t^p\\
\textbf{Updated estimate:}& \quad \hat{x}_t \sim \mathcal{N}(\mu _t, P_t)\\
```

### The Kalman Filter algorithm

Notice that the prediction and update equation depend only on the previous state estimate.
Given an initial state estimate, and as we receive measurements (and control inputs), we can compute the next state estimate.

The pseudocode for running a Kalman filter is as follows.
Starting with $\hat{x}_0 \sim \mathcal{N}(\mu_0, P_0)$,

```
mu_previous = mu0
P_previous = P0
for t = 0,...,N
    mu_predict, P_predict = predict_step(mu_previous, P_previous, A, B, Q)
    y = receive measurement from sensor
    mu_update, P_update = update_step(mu_predict, P_predict, y, C, R)
    mu_previous = mu_previous
    P_previous = P_update
end
```

### Connection with LQR
The Kalman filter may have reminded you of the LQR algorithm, especially with the using the same letters to define matrices. But there are also some notable differences.
For starters, the Kalman filter runs *forward* in time, while LQR requires solving for $P_t$ *backward* in time.

It turns out, the Kalman Filter prediction and update steps is actually the Riccati equation!

Given that $P_t = (I - K_tC)P_t^p$, and $P_{t+1}^p = AP_tA^T + Q$, substituting the first equation into the second, we get:

$$
P_{t+1}^p = Q + AP_t^pA^T - AP_t^pC^T(CP_t^pC^T + R)^{-1} CP_t^pA^T
$$

Does this look familiar...?
If we refer back to your LQR notes (for discrete time), the equation we derived was,

$$
P_{t-1} = Q + A^TP_tA - A^TP_tB(B^TP_tB + R)^{-1}B^TP_tA
$$

Doesn't it look very similar, except for a few things:
- Kalman filter iterates *forward* in time, while LQR iterates *backward* in time.
- The $A$ matrices differ by a transpose
- The $B$ in LQR is $C^T$ in the Kalman filter

This connection between LQR (control) and Kalman Filter (estimation) is known as **duality**. Essentially, performing control and estimation ends up being the same algorithm (solving the Riccati equation), but with different variables and direction in the iteration.


| **LQR description** | **LQR variables** | **KF variables** | **KF description**|
|---------------------|-------------------|------------------|-------------------|
| Dynamics            | $A$               | $A^T$            | Dynamics transposed|
| Control matrix      | $B$               | $C^T$            | Observation transposed|
| State cost          | $Q$               | $Q$              | Process noise covariance|
| Control cost        | $R$               | $R$              | Measurement noise covariance|
| Terminal cost       | $Q_T$             | $P_0$            | Initial state covariance|
