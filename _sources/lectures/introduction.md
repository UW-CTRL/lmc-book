# Introduction

These notes provide an introduction to the basics of multivariable control. Perhaps unlike other texts, these notes and accompanying exercises takes on a more practical approach to the topic. These notes are primarily centered around *optimization-based* control, the notion of selecting the "best" controls according to some criteria and possibly constraints. Perhaps the main difference between optimization-based control and just regular optimization is that former is in the context of controlling a dynamical system. That is, a system whose (multi-dimensional) properties changes over time according to a set of equations. So the selected controls should not only consider the impact at the current time, but also the future. Thus the problem of controlling a dynamical system requires making a *sequence* of decisions, or also referred to as *sequential decision making*. How these dynamics are represented, the properties of them, how certain we are about them, and what elements of the system can be observed will influence the types of techniques to use to solve the, broadly speaking, "control problem".

How we perform sequential decision making has changed over the years due to advances in computation, data, and machinery. Though arguably, the fundamental problem of finding the "best" sequence of control to enable the system to accomplish a desired tasks has not.
The goal of these notes is to expose the reader to the basics and fundamentals, and connections to adjacent topics, with the hope that it will make reading research papers and other resources in those connected fields more accessible.




## Note are work-in-progress
These notes are under construction and are based on lectures from UW AA/EE/ME 548 and AE 513, taught by Karen Leung Spring 2023--now. These notes are based on scribe notes by students from Spring 2023.
Since these notes are brand new, you may find typos and errors. If you do, please send an email to `ae513-aut25-staff@uw.edu` or submit a pull request with the fixes to the Github repository.
