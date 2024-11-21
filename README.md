<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->

<!-- PROJECT LOGO -->
<br>
<div align="center">

  <h1 align="center">Operational Space Controller</h1>

  <p align="left">
    A work-in-progress implementation of a basic low-level operational space control used for whole body task-space control of articulated robotic systems subject to contact.
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
* [boost](https://www.boost.org/) (Version 1.70 or higher)
* [bopt]()
* [pinocchio]()
* [casadi]()

For testing purposes we have (this is later be a toggle-able option):
* [googletest](https://github.com/google/googletest)
* [glog](https://github.com/google/glog)


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
