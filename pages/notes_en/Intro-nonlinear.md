---
layout: page
title: Introduction to nonlinear systems
collection: notes_en
---

## General Idea

Nonlinear control is a field of control that deals with nonlinear dynamical systems without making the system linear by the Taylor series approximation, as it is commonly done in linear control theory. Instead, by using model-aware user-crafted nonlinear control signals we can transform a nonlinear objective into a linear system and apply the linear control tools we know to stabilize it.  

## Control techniques

In nonlinear control theory we can find  feedback linearization, sliding mode control, adaptive control, control lyapunov function and many other techniques. More or less  they heavily rely on the knowledge of the dynamics of the non-linear system,  or at least its dynamic structure. I will mostly focus here on feedback linearization because it is quite simple and yet  a very powerful tool for control and verification. 

## Feedback linearization

Let’s start with a very simple example. Consider the following scalar ($x$ is not a vector) nonlinear system with unbounded control $u$:

$$
\dot{x}=3x^2+\sin(x)+(1+x^2)u
$$

Can you guess a stabilizing controller that drives $x \rightarrow 0$? Remember that you have free choosing of the control signal $u$, you know the dynamics of the system and the state $x$ is known at every instant of time.

You might have guessed the control signal by now, even if you didn’t, it is actually quite simple. We want to remove the nonlinear terms, and it makes sense since they are the ones that make the system harder to understand. The nonlinear feedback is:

$$
u=\frac{1}{1+x^2}[-3x^2-\sin(x)+\nu]
$$

We just applied the inverse of the nonlinear terms. But we also applied an auxiliary control $\nu \in \mathbb{R}$, this control term will be designed with linear control tools. When we apply the control to the non-linear system we end up with:

$$
\dot{x}=\nu
$$

This is a linear system, a very simple one in fact. For a first-order system we can apply a proportional control $\nu=-Kx, K>0$. This leaves us with  $\dot{x}=-Kx$ which is an stable system. 

<aside>
💡 Note that the the control signal is well-defined because $(1+x^2)$  is never zero. In general it is important to check that the transformation is singularity-free in the domain of operation.

</aside>

You see, this is a very simple idea but it powers a lot of nonlinear control techniques. We will explore a second example before we generalize it. 

Consider the following scalar nonlinear system:

$$
\dot{x}_1=x_2 \\
\dot{x}_2= \sin(x_1)+x_3\\
\dot{x}_3=x_1+u
$$

Now, assume our objective is to drive $y=x_1-x_2 \rightarrow  0$.  This might seem trickier, in one hand we have a third order  system and on the other, we want to control a function of our states $y(x_1,x_2)$. As we will see, the trick is to generate the dynamics of $y(x)$  and generate a linearizing feedback that leads again to a linear system. To generate the dynamics of $y$ we will apply  a derivative operator over it until the control signal $u$ appears on the dynamics. Let’s see how it goes:

$$
y=x_1-x_2
$$

We apply the first derivative:

$$
\dot{y}=\dot{x}_1-\dot{x}_2=x_2-\sin(x_1)-x_3 
$$

We don’t see the control signal $u$ yet, so we apply another derivative:

$$
\ddot{y}=\dot{x}_2-\cos(x_1)\dot{x}_1-\dot{x}_3\\
\ddot{y}=\sin(x_1)+x_3-\cos(x_1)x_2-x_1-u
$$

In  the last expression we found the control $u$. Since we find the control on the second derivative of the output $y$, we say that the output has relative degree two ($r=2$). Now we apply  the same idea as the first example. We will make all the other terms cancel with an appropriate control:

$$
u=-(x_1+\cos(x_1)x_2-x_3-\sin(x_1))-\nu
$$

We end up with the following linear dynamics on the output:

$$
\ddot{y}=\nu
$$

For that second order system a PD controller suffice to drive $y \rightarrow 0$. In that case $\nu=-K_py-K_d\ddot{y}$, and the output $y$ is stabilized.

<aside>
💡 Our original system is of order 3, while our output dynamics is a second order system. It means that there is a residual dynamics called Zero Dynamics of order 1. It is important to check the stability of the Zero Dynamics.

</aside>

## Zero Dynamics

The Zero Dynamics is the dynamics that is left after controlling the output system. There is a couple of common ways to formalize it but we will not do it yet. Besides more detail and formal definitions can be found in nonlinear systems books.

Consider the following system:

$$
\dot{x}_1=x_1^2+x_2\\
\dot{x}_2=x_2+u
$$

Let’s assume we are interested on driving $y=x_2-x_2^{ref}$ to zero. In that case:

$$
\dot{y}=\dot{x}_2=x_2+u
$$

For the relative degree one system, the stabilizing control has the form:

$$
u=-x_2-Ky
$$

The controller drives $y$ to zero, meaning that $x_2=x_2^{ref}$. For the remaining system described by $\dot{x}_1=x_1^2+x_2$ we can analyze it when $x_2$ takes its desired value, i.e.

 

$$
\dot{x}_1=x_1^2+x_2^{ref}
$$

We note that this system is dependent on the initial condition of $x_1$ and the value of $x_2^{ref}$. For instance, for $x_2^{ref}=0$ we observe that $x_1$ can explode unstably ($\dot{x}_1=x_1^2$).

## Example: Cart-Pole System

Let’s borrow the definition of the cart-pole from this [MIT lecture note](http://underactuated.mit.edu/acrobot.html#cart_pole). Specially the structure given in  robot dynamics:

$$
M(q)\ddot{q}+C(q,\dot{q})\dot{q}=\tau_g(q)+Bu
$$

where, $q=[x, \theta]$.

We usually are interested on controlling $y = \theta - \pi$. The output dynamics are:

$$
\ddot{y}=\ddot{\theta}=S\ddot{q}
$$

Where $S = \begin{bmatrix} 0 &  1\end{bmatrix}$.

$$
\ddot{y}=SM^{-1}(q)[-C(q,\dot{q}) \dot{q}+\tau_g(q)]+SM^{-1}(q)Bu
$$

Following the Feedback linearization methodology,  we stabilize the system by providing the following feed-forward term:

$$
u=-(SM^{-1}B)^{-1}SM^{-1}(q)[-C(q,\dot{q})\dot{q}+\tau_g(q)] + (SM^{-1}B)^{-1}\nu
$$

where the auxiliar control can be chosen as: $\nu=-K_p(\theta-\pi)-K_d \dot{\theta}$

### What happens with the zero dynamics?

When we control uniquely the second order system $\ddot{y}=\ddot{\theta}$, the dynamics of $x$ are  left to evolve on its own. 

$$
\ddot{x}=\bar{S}M^{-1}(q)[-C(q,\dot{q}) \dot{q}+\tau_g(q)]+\bar{S}M^{-1}(q)Bu
$$

where, $\bar{S}=\begin{bmatrix} 1 & 0 \end{bmatrix}$ and $u$ is the controller designed above to drive $y \rightarrow 0$. When the control output is driven to zero we note that the controller reduces slightly to 

$$
u=-(SM^{-1}B)^{-1}SM^{-1}(q)[-C(q,\dot{q})\dot{q}+\tau_g(q)] \\
\ddot{\theta}=\dot{\theta}=0, \theta=\pi
$$

In that case, the dynamics of $x$ becomes:

(I will drop all the dependencies to reduce visual mess)

$$
\ddot{x}=\bar{S}M^{-1}[-C \dot{q}+\tau_g]-\bar{S}M^{-1}B(SM^{-1}B)^{-1}SM^{-1}[-C\dot{q}+\tau_g]
$$

That equation is independent of u and depends only on the variable $x$, so we can “easily” investigate its stability. I used the symbolic toolbox of MATLAB to simplify that  equation, and it gave me the following  expression: