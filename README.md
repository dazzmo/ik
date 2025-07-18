<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->

<!-- PROJECT LOGO -->
<br>
<div>
  <h1>Cink - The C++ Adaptation of the Pink Differentiable Inverse Kinematics Library </h1>
  <p>
    A work-in-progress implementation of a basic inverse kinematics engine
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About The Project
<p align="left">
This project is a C++ adaptation of Stephane Caron's pink python package, utilising the Pinocchio library in its native C++ for efficient derivation and computation of inverse kinematics. 

For solving the quadratic programs that arise from this problem formulation, we use CasADi's solver wrappers for quadratic programs, allowing users to specify a supported QP solver similarly to qpsolvers in Python.

## Installation
<a name="installation"></a>

### Prerequisites
Cink requires the following third-party libraries in order to be built and installed.
* [pinocchio]()
* [casadi]() (Installed with any desired QP solvers for `qpsol()` - see here)

### Building

1. Clone the repo
   ```sh
   git clone https://github.com/dazzmo/ik
   ```
2. Build the library
    ```sh
    cd ik
    mkdir build && cd build
    cmake ..
    make
   ```
3. Installation of the library can then be performed by
    ```sh
    make install
    ```

<!-- CONTACT -->
## Contact

Damian Abood - damian.abood@sydney.edu.au

## Acknowledgements
This package was inspired by Stephane Caron's pink python package
<p align="right">(<a href="#readme-top">back to top</a>)</p>
