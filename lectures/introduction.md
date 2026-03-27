# Introduction

Welcome to these notes on multivariable control. Unlike many traditional texts, these materials and accompanying exercises take a hands-on, practical approach, emphasizing how theory translates to implementation. This course is grounded in *optimization-based* control—the process of selecting the "best" possible controls by optimizing a set of criteria within given constraints. The key distinction between optimization-based control and conventional optimization is context: here, we are concerned with the control of *dynamical systems*—systems whose (often multidimensional) state evolves over time according to specific equations. The objectives and constraints arise from both the underlying physics and the goals of the task at hand. As a result, controllers must not only make decisions relevant for the present moment, but also take into account their effects on the future. This naturally leads to the concept of *sequential decision making*, where the challenge lies in crafting a sequence of actions to achieve the desired result.


Advances in computation, data availability, and hardware have significantly influenced how we approach sequential decision making. However, the fundamental challenge—finding the optimal sequence of controls to guide a system toward its objectives—remains unchanged. These notes aim to introduce the foundational principles of multivariable control, establish the connection to adjacent fields, and provide context for understanding research papers and further resources across these domains.

## Notes in Progress

These materials are a work in progress and directly reflect lectures from UW AA/EE/ME 548 and AE 513, taught by Karen Leung (Spring 2023–present), as well as scribe notes contributed by students in Spring 2023. Since these notes are new, you may encounter typos or errors. If you spot any, please email `aa548-spr26-staff@uw.edu`, or better yet, submit a pull request to the GitHub repository.
