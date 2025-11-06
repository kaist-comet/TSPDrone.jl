# TSPDrone.jl

[![Build Status](https://github.com/chkwon/TSPDrone.jl/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/chkwon/TSPDrone.jl/actions?query=workflow%3ACI)
[![codecov](https://codecov.io/gh/chkwon/TSPDrone.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/chkwon/TSPDrone.jl)

This package solves the Traveling Salesman Problem with Drone (TSP-D) with 1 truck and 1 drone. This implements the Divide-Partition-and-Search (DPS) algorithm and the Deep Reinforcement Learning (DRL) method as proposed in the following paper:

* [Bogyrbayeva, A., Yoon, T., Ko, H., Lim, S., Yun, H. and Kwon, C., 2023. A deep reinforcement learning approach for solving the traveling salesman problem with drone. *Transportation Research Part C: Emerging Technologies*, 148, p.103981.](https://arxiv.org/abs/2112.12545) [DOI: 10.1016/j.trc.2022.103981](https://doi.org/10.1016/j.trc.2022.103981)

The DPS algorithm is based on the TSP-ep-all algorithm of [Agatz et al. (2018)](https://doi.org/10.1287/trsc.2017.0791) and the divide-and-conquer heuristic of [Poikonen et al. (2019)](https://doi.org/10.1287/ijoc.2018.0826).

If you use either the DPS algorithm or the DRL method, please cite:
```
@article{bogyrbayeva2023deep,
  title={A deep reinforcement learning approach for solving the traveling salesman problem with drone},
  author={Bogyrbayeva, Aigerim and Yoon, Taehyun and Ko, Hanbum and Lim, Sungbin and Yun, Hyokun and Kwon, Changhyun},
  journal={Transportation Research Part C: Emerging Technologies},
  volume={148},
  pages={103981},
  year={2023},
  publisher={Elsevier}
}
```

# Install

```julia
] add https://github.com/chkwon/TSPDrone.jl
```

If you want just the DPS algorithm, the above will be sufficient. Skip the rest.

If you also want to use the DRL method, you need to set up your Python and PyTorch installations. 
Suppose your Python installation is located at `/usr/local/bin/python3`.
First, make sure that this particular Python has `torch`, `numpy`, and `scipy` packages; if not, please install them. 
For example:
```console
python3 -m pip install torch numpy scipy
```

In Julia:
```julia
julia> ENV["PYTHON"] = "/usr/local/bin/python3"
```
Then
```julia
julia> import Pkg; Pkg.build("PyCall")
```
Test if everything works fine:
```julia
julia> using TSPDrone
julia> TSPDrone.test_RL()
```
which should not generate errors.
If it does not work properly, check if your Julia is connected with a proper Python installation. 
For example:
```julia
julia> using PyCall
julia> PyCall.python
"/usr/local/bin/python3"
julia> PyCall.pyversion
v"3.9.7"
```
If it does not use the Python installation you like, try the above process again.


# Using the Divide-Partition-and-Search (DPS) Algorithm

You can provide `x` and `y` coordinates of customers. 
The depot should be the first element in `x` and `y`.

Two parameters `truck_cost_factor` and `drone_cost_factor` will be multiplied to the Euclidean distance calculated from the coordinates. 
```julia 
using TSPDrone
n = 10 
x = rand(n); y = rand(n);
truck_cost_factor = 1.0 
drone_cost_factor = 0.5
result = solve_tspd(x, y, truck_cost_factor, drone_cost_factor)
@show result.total_cost;
@show result.truck_route;
@show result.drone_route;
```
returns
```
result.total_cost = 1.640497825599171
result.truck_route = [1, 2, 6, 7, 10, 8, 4, 11]
result.drone_route = [1, 9, 6, 5, 7, 3, 11]
```
where node `11` represents the depot as the final destination. 

You can also provide the cost matrices of truck and drone directly.
Again, the depot is labeled as `1`.
```julia
using TSPDrone
n = 10 
dist_mtx = rand(n, n)
dist_mtx = dist_mtx + dist_mtx' # symmetric distance only
truck_cost_mtx = dist_mtx .* 1.0
drone_cost_mtx = truck_cost_mtx .* 0.5 
result = solve_tspd(truck_cost_mtx, drone_cost_mtx)
@assert size(truck_cost_mtx) == size(drone_cost_mtx) == (n, n)
```
returns
```
result.total_cost = 1.640497825599171
result.truck_route = [1, 2, 6, 7, 10, 8, 4, 11]
result.drone_route = [1, 9, 6, 5, 7, 3, 11]
```
where again node `11` represets the depot as the final destination.

## Summary of the Result
Use `print_summary(result)`:
```julia
julia> print_summary(result)
Operation #1:
  - Truck        = 0.17988883875173492 : [1, 3]
  - Drone        = 0.11900891950265155 : [1, 4, 3]
  - Length       = 0.17988883875173492
Operation #2:
  - Truck        = 0.4784476248243221 : [3, 9]
  - Drone        = 0.27587675362585756 : [3, 7, 9]
  - Length       = 0.4784476248243221
Operation #3:
  - Truck        = 0.445749847855226 : [9, 6]
  - Drone        = 0.48831605249544785 : [9, 10, 6]
  - Length       = 0.48831605249544785
Operation #4:
  - Truck        = 0.9269158918021541 : [6, 5, 8, 11]
  - Drone        = 0.8714473929102112 : [6, 2, 11]
  - Length       = 0.9269158918021541
Total Cost = 2.073568407873659
```
## Options for DPS 
Optional keyword arguments for `solve_tspd`:
```julia
n_groups::Int = 1, 
method::String = "TSP-ep-all", 
flying_range::Float64 = MAX_DRONE_RANGE, 
time_limit::Float64 = MAX_TIME_LIMIT,
initial_tour::Union{Vector{Int}, Nothing} = nothing,
drone_eligible::Union{Nothing, Set{Int}} = nothing
```
* `n_groups`: The number of subgroups for divide and conquer. For example, if `n=100` and `n_groups=4`, then each group will have `25` nodes. Then, `method` is applied for each group. 
* `method`: can be one of the following (See [Agatz et al. 2018](https://doi.org/10.1287/trsc.2017.0791)):
    - `"TSP-ep"`: The exact partitioning of the initial TSP tour is performend. Then, no search is performed.
    - `"TSP-ep-1p"`: The exact partitioning of the initial TSP tour is performend. Then, all possible one-point moves are performed on the initial TSP tour, which is followed by exact partitioning of new TSP tours.
    - `"TSP-ep-2p"`: All possible two-point moves.
    - `"TSP-ep-2opt"`: All possible 2-opt exchange moves.
    - `"TSP-ep-all"`: All possible 1p, 2p, and 2opt moves.
* `flying_range`: The limited flying range of the drone. The default value is `Inf`. The flying range is compared with the values in the drone cost matrix; that is, `drone_cost_mtx` or the Euclidean distance multiplied by `drone_cost_factor`. 
* `time_limit`: The total time limit to solve the problem in seconds. For each group, the time limit is split equally. For example, if `time_limit=3600.0` and `n_groups=5`, then each group has time limit of 3600/5 = 720 seconds.
* `initial_tour`: Optional initial TSP tour. If `nothing`, a TSP tour is computed using Concorde. The tour should start at node 1 and end at node `n+1` (dummy depot). If the final depot is missing, it will be added automatically.
* `drone_eligible`: Set of nodes that can be served by the drone. If `nothing`, all nodes are eligible. Nodes not in this set will only be served by the truck. 



# Using the Deep Reinforcement Learning (DRL) method

This package comes with pre-trained weights for `n ∈ [11, 15, 20, 50, 100]`; these sizes include the depot.
For training, the followings were used:
- The x,y coordinate of the depot is uniformly distributed in `[0, 1]`.
- The x,y coordinates of the customers are uniformly distributed in `[0, 100]`.
- `truck_cost_factor = 1.0` and `drone_cost_factor = 0.5`.
- The drone fyling range is ∞.

For example, 
```julia
# the problem size, n ∈ [11, 15, 20, 50, 100]
n = 11 

# depot coordinates from Uniform[0, 1]
depot_x = rand()
depot_y = rand()

# customer coorindates from Uniform[0, 100]
customers_x = rand(n - 1) .* 100
customers_y = rand(n - 1) .* 100

# the first elements are for the depot
x_coordinates = [depot_x; customers_x]
y_coordinates = [depot_y; customers_y]

@assert n == length(x_coordinates) == length(y_coordinates)

using TSPDrone
result = solve_tspd_RL(x_coordinates, y_coordinates; n_samples = 100)
```

In a sample run, the outcome was:
```
(result.total_cost, result.truck_route, result.drone_route) = 
(239.78793778400822, [1, 3, 8, 2, 11, 6, 7, 12], [1, 10, 3, 5, 8, 9, 6, 4, 12])
```
In the truck and drone routes, both node 1 and node 12 refers to the depot. 

## Options for DRL
```julia
result = solve_tspd_RL(x_coordinates, y_coordinates; n_samples = 100, device = "cpu")
```

If `n_samples = 1` or not provided, it will use the greedy decoding.
If `n_samples > 1`, then it will generate multiple samples and return the best result.

You can also pass `device`. The default is `device = "cpu"`, but if you have a GPU, then you can also pass the device name, so that `torch` can run on the GPU. 

## Related Packages

- [TSPDroneLIB](https://github.com/chkwon/TSPDroneLIB)
- [TSPDrone.jl](https://github.com/chkwon/TSPDrone.jl)
- [TSPDroneHGATAC.jl](https://github.com/Sasanm88/TSPDroneHGATAC.jl)
- [TSPDroneICP.jl](https://github.com/0505daniel/TSPDroneICP.jl)
