<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->

<!-- PROJECT LOGO -->
<br>
<div align="center">

  <h1 align="center">Inverse Kinematics Engine</h1>

  <p align="left">
    A work-in-progress implementation of a basic inverse kinematics engine
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About The Project
<p align="left">
</p>
<p align="right">(<a href="#readme-top">back to top</a>)
</p>

## Getting Started
<a name="getting-started"></a>

### Prerequisites
ik requires the following third-party libraries in order to be built and installed.
* Pinocchio - for kinodynamic calculations such as frame placements and inverse kinematics
* CasADi - for representing and solving optimisation problems with freely available solvers
<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installing Prerequisites Through Robotpkg
If packages were previously installed via other methods, Simply remove the pinocchio package from ROS (sudo apt remove ros-humble-pinocchio). I would also recommend removing CasADi from the system if downloaded, which can be done by deleting the /usr/local/lib & /usr/local/include files.
#### Install
1. Install the robotpkg packages using the following commands: 
    ```sh
    sudo apt install -qqy lsb-release curl && \
    sudo mkdir -p /etc/apt/keyrings && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc >/dev/null && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list >/dev/null && \
    sudo apt update && \
    sudo apt install -qqy robotpkg-py3*-pinocchio robotpkg-py3*-casadi robotpkg-py3*-example-robot-data
    ```
 
2. Update your .bashrc with the following lines:
    ```sh
    export PATH=/opt/openrobots/bin:$PATH
    export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
    export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY
    export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
    export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
    ```
#### Uninstall
  1. Remove the ```/opt/openrobots``` folder from the system files.
  2. Remove the recently added lines from .bashrc.
  3. Reinstall Pinocchio via ROS and CasADi via the traditional method.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation
<a name="installation"></a>

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
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the GNU LESSER GENERAL PUBLIC LICENSE License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Damian Abood - damian.abood@sydney.edu.au

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Acknowledgements

<p align="right">(<a href="#readme-top">back to top</a>)</p>
