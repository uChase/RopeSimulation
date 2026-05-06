# Real-Time Rope Simulation in Unity

This repository contains the Unity project for my real-time rope simulation. The project implements a custom particle-based rope solver with numerical integration, Lagrange multiplier constraint forces, bending correction, wind forces, and procedural mesh generation.

## Where to Find the Code

The main source code is located in:

```text
Assets/Scripts/
```
This directory contains the scripts for the rope simulation, including the rope controller, node data structures, solver implementations, wind box behavior, and mesh generation logic.

The main code of interest is:
```text
Assets/Scripts/Rope.cs
```
This file contains most of the rope simulation logic, including force computation, the Lagrange multiplier constraint solve, bending correction, and procedural mesh generation.

The solver implementations are also important. These are located in the same scripts directory and include the Verlet and RK4 solver files.

## Project Overview

The rope is modeled as a chain of particles connected by distance constraints. Instead of using Unity rigid bodies and joints for each rope segment, the simulation computes the rope motion directly. The implementation compares two numerical solvers:

* Verlet integration
* Fourth-order Runge–Kutta integration

The simulation also includes a Lagrange multiplier constraint solve to keep rope segments close to their target length, using a tridiagonal system solved with the Thomas algorithm.

## Running the Project

Open the project in Unity, then open the main scene included in the project. The rope behavior can be adjusted through the inspector by changing parameters such as node count, segment length, solver type, damping, wind strength, rope radius, and mesh resolution.

## Notes

This project was created for a scientific computing final project (Harvey Mudd MATH164). The focus is on real-time performance, numerical methods, and procedural rope rendering rather than a fully physically complete rope model.
