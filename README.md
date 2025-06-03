operational space whole-body control library.

based on,

Yisoo Lee, Junewhee Ahn, et al. "Computationally Efficient HQP-based Whole-body Control Exploiting the Operational-space Formulation." 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2021.
Junewhee Ahn, et al. "Time-Efficient Contact Consistent Whole-Body Control Framework via Reduced-Dimension Dynamics Construction" 2024 IEEE ACCESS
Junewhee Ahn, et al. "Efficient Computation of Whole-Body Control Utilizing Simplified Whole-Body Dynamics via Centroidal Dynamics" 2024 arXiv
	

## How to ...
### prerequisites
 * Eigen3
 * [RBDL](https://github.com/saga0619/rbdl-orb)
 * [qpOASES](https://github.com/saga0619/qpOASES)
 
### install RBDL
```sh
git clone --recursive https://github.com/saga0619/rbdl-orb
cd rbdl-orb
mkdir build
cd build
cmake ..
make all
sudo make install
```

### install qpOASES
```sh
git clone https://github.com/saga0619/qpoases
cd qpoases
mkdir build
cd build
cmake ..
make all
sudo make install
```

### How to install
```sh
mkdir build
cd build 
cmake ..
make
sudo make install
```

### Unit Test and Benchmark
```sh
cmake -DRUN_TEST=ON -DRUN_BENCHMARK=ON -DCMAKE_BUILD_TYPE=Debug ..
```

### How to play with
see [example](https://github.com/saga0619/dyros_hqp_lib/tree/main/example)

