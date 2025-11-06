function find_tsp_tour(Ct::Matrix{Float64})
    scale_factor = 1000
    dist_mtx = round.(Int, Ct .* scale_factor)
    
    tsp_tour, tsp_tour_len = Concorde.solve_tsp(dist_mtx)

    @assert tsp_tour[1] == 1

    return tsp_tour
end


function find_tsp_tour(x::Vector{Float64}, y::Vector{Float64})
    scale_factor = 1000
    
    tsp_tour, tsp_tour_len = Concorde.solve_tsp(x * scale_factor, y * scale_factor; dist="EUC_2D")

    @assert tsp_tour[1] == 1

    return tsp_tour
end

function precompute_dist_order(Cd)
    n, _ = size(Cd)
    precomputed_order = repeat(reshape(1:n, 1, :), n, 1)
    for (i, row) in enumerate(eachrow(precomputed_order))
        row .= sort(collect(row), by = x -> Cd[i, x])
    end
    return precomputed_order
end
function exact_partitioning(initial_tour, Ct, Cd; flying_range=MAX_DRONE_RANGE, quadratic_ep_boost=true, precomputed_order=nothing, drone_eligible::Union{Nothing,Set{Int}}=nothing)
    n, _ = size(Ct)

    r = initial_tour
    T = fill(Inf, n, n)
    M = fill(-1, n, n)

    sum_Ct = zeros(n, n)
    for i in 1:n-1
        sum_Ct[i, i+1] = Ct[r[i], r[i+1]]
        T[i, i+1] = sum_Ct[i, i+1]
        for j in i+2:n
            sum_Ct[i, j] = sum_Ct[i, j-1] + Ct[r[j-1], r[j]]
        end
    end
    inv_r = zeros(Int, n)
    for i in 1:n
        inv_r[r[i]] = i
    end
    
    # Check if node is eligible for drone service
    @inline is_drone_eligible(k_node::Int) = (drone_eligible === nothing || k_node in drone_eligible)
    
    if quadratic_ep_boost
        if flying_range == MAX_DRONE_RANGE
            @inbounds for i in 1:n-2
                @inbounds for k in i+1:n-1
                    k_node = r[k]
                    if !is_drone_eligible(k_node)
                        continue
                    end
                    for j = k+1:n
                        t_cost = sum_Ct[i, k-1] + Ct[r[k-1], r[k+1]] + sum_Ct[k+1, j]
                        d_cost = Cd[r[i], r[k]] + Cd[r[k], r[j]]
                        cost = max(t_cost, d_cost)
                        if T[i, j] > cost
                            T[i, j] = cost
                            M[r[i], r[j]] = r[k]
                        end
                        if t_cost > d_cost
                            break
                        end
                    end
                end
            end
        else
            if isnothing(precomputed_order)
                precomputed_order = precompute_dist_order(Cd)
            end
            J = Vector{Int}(undef, n)
            for k_tour in 2:n-1
                k_node = r[k_tour]
                if !is_drone_eligible(k_node)
                    continue
                end
                @simd for j in k_tour:n
                    J[j] = j+1
                end
                for i_node in precomputed_order[k_node, :]
                    i_tour = inv_r[i_node]
                    if i_tour >= k_tour
                        continue
                    end
                    j_tour = J[k_tour]
                    prev_j_tour = k_tour
                    while j_tour !== n+1
                        j_node = r[j_tour]
                        d_cost = Cd[i_node, k_node] + Cd[k_node, j_node]
                        if d_cost > flying_range
                            j_tour = J[j_tour]
                            J[prev_j_tour] = j_tour
                            continue
                        end
                        t_cost = sum_Ct[i_tour, k_tour-1] + Ct[r[k_tour-1], r[k_tour+1]] + sum_Ct[k_tour+1, j_tour]
                        cost = max(t_cost, d_cost)
                        if T[i_tour, j_tour] > cost
                            T[i_tour, j_tour] = cost
                            M[i_node, j_node] = k_node
                        end
                        if t_cost > d_cost
                            break
                        end
                        prev_j_tour = j_tour
                        j_tour = J[j_tour]
                    end
                end
            end
        end
    else
        @inbounds for i in 1:n-1
            @inbounds for j in i+1:n
                @inbounds for k in i+1:j-1
                    k_node = r[k]
                    if !is_drone_eligible(k_node)
                        continue
                    end
                    d_cost = Cd[r[i], r[k]] + Cd[r[k], r[j]]
                    if d_cost <= flying_range
                        t_cost = sum_Ct[i, k-1] + Ct[r[k-1], r[k+1]] + sum_Ct[k+1, j]
                        cost = max(t_cost, d_cost)
                        if cost < T[i, j]
                            T[i, j] = cost
                            M[r[i], r[j]] = r[k]
                        end
                    end
                end
            end
        end
    end

    V = zeros(n)
    P = fill(-1, n)
    V[1] = 0
    @inbounds for i in 2:n
        VV = [V[k] + T[k, i] for k in 1:i-1]
        am = argmin(VV)
        V[i] = VV[am]
        P[i] = r[am]
    end

    # Retrieving solutions.
    combined_nodes = Int[]
    current_idx = n
    current = r[current_idx]
    while current != -1
        push!(combined_nodes, current)
        current_idx = inv_r[current]
        current = P[current_idx]
    end
    reverse!(combined_nodes)
    drone_only_nodes = Int[]
    drone_route = Int[]
    push!(drone_route, combined_nodes[1])
    @assert combined_nodes[1] == r[1]
    @inbounds for i in 1:length(combined_nodes)-1
        j1 = combined_nodes[i]
        j2 = combined_nodes[i+1]
        if M[j1, j2] != -1
            push!(drone_only_nodes, M[j1, j2])
            push!(drone_route, M[j1, j2])
        end
        push!(drone_route, j2)
    end
    truck_route = setdiff(initial_tour, drone_only_nodes)

    final_time = V[end]
    obj_val = objective_value(truck_route, drone_route, Ct, Cd)
    @assert isapprox(obj_val, final_time)

    return final_time, truck_route, drone_route
end


function two_point_move(tour, i, j)
    if i >= j 
        return tour, false
    end

    tmp = copy(tour)
    tmp[j] = tour[i]
    tmp[i] = tour[j]
    return tmp, true
end

function one_point_move(tour, i, j)
    tmp = copy(tour)
    deleteat!(tmp, i)
    insert!(tmp, j, tour[i])
    return tmp, true
end

function two_opt_move(tour, i, j)
    if i >= j 
        return tour, false
    end

    tmp = copy(tour)
    tmp[i:j] = tour[j:-1:i]
    return tmp, true
end

# Main function to call
function tsp_ep_all(
    x_coordinates, 
    y_coordinates, 
    truck_cost_factor, 
    drone_cost_factor; 
    local_search_methods=[two_point_move, one_point_move, two_opt_move], 
    flying_range=MAX_DRONE_RANGE, 
    time_limit=MAX_TIME_LIMIT,
    quadratic_ep_boost=true,
    initial_tour::Union{Vector{Int}, Nothing}=nothing,
    drone_eligible::Union{Nothing,Set{Int}}=nothing
)    
    """
    Runs `TSP-ep-all` heuristic algorithm of Agatz et al.

    `x_coordinates[1]`, `y_coordinates[1]`: the coordinates of the depot, then followed by all customer location coordinates
    `truck_cost_factor`: as defined in Agatz et al. instances
    `drone_cost_factor`: as defined in Agatz et al. instances
    `initial_tour`: optional initial TSP tour (if provided, skips TSP tour generation)
    `drone_eligible`: Set of nodes that can be served by drone (if nothing, all nodes are eligible)
    """

    Ct, Cd = cost_matrices_with_dummy(x_coordinates, y_coordinates, truck_cost_factor, drone_cost_factor)
    n_nodes = length(x_coordinates)
    n1, n2 = size(Ct)
    @assert n_nodes + 1 == n1
    @assert size(Ct) == size(Cd)

    if initial_tour === nothing
        tsp_tour = find_tsp_tour(x_coordinates, y_coordinates)
        push!(tsp_tour, n_nodes+1) # adding a dummy node for the returning depot 
    else
        tsp_tour = copy(initial_tour)
        # Ensure it ends with dummy depot
        if tsp_tour[end] != n_nodes + 1
            push!(tsp_tour, n_nodes+1)
        end
    end

    return tsp_ep_all(Ct, Cd, tsp_tour, flying_range=flying_range, local_search_methods=local_search_methods, time_limit=time_limit, quadratic_ep_boost=quadratic_ep_boost, drone_eligible=drone_eligible)
end


function tsp_ep_all(
    Ct, 
    Cd, 
    init_tour; 
    local_search_methods=[two_point_move, one_point_move, two_opt_move], 
    flying_range=MAX_DRONE_RANGE, 
    time_limit=MAX_TIME_LIMIT,
    quadratic_ep_boost=true,
    drone_eligible::Union{Nothing,Set{Int}}=nothing
)   
    n, _ = size(Ct)

    improved = true
    precomputed_order = (quadratic_ep_boost && flying_range != MAX_DRONE_RANGE) ? precompute_dist_order(Cd) : nothing
    best_obj, t_route, d_route = exact_partitioning(init_tour, Ct, Cd, flying_range=flying_range, quadratic_ep_boost=quadratic_ep_boost, precomputed_order=precomputed_order, drone_eligible=drone_eligible)

    best_tour = copy(init_tour)
    best_t_route = copy(t_route)
    best_d_route = copy(d_route)

    if isempty(local_search_methods)
        return best_obj, best_t_route, best_d_route
    end

    time0 = time()

    is_time_over = false

    while improved && !is_time_over
        improved = false
        cur_best_obj = best_obj
        cur_best_tour = copy(best_tour)
        cur_best_t_route = copy(best_t_route)
        cur_best_d_route = copy(best_d_route)

        for i in 2:n-1
            if is_time_over
                break
            end

            for local_search in local_search_methods
                for j in 2:n-1
                    if is_time_over
                        break
                    end
                                        
                    new_tour, is_valid = local_search(best_tour, i, j)
                    is_time_over = time() - time0 > time_limit 

                    if is_valid
                        ep_time, t_route, d_route = exact_partitioning(new_tour, Ct, Cd, flying_range=flying_range, quadratic_ep_boost=quadratic_ep_boost, precomputed_order=precomputed_order, drone_eligible=drone_eligible)
                        if ep_time < cur_best_obj
                            cur_best_tour = copy(new_tour)
                            cur_best_t_route = copy(t_route)
                            cur_best_d_route = copy(d_route)
                            cur_best_obj = ep_time
                            improved = true
                        end
                    end
                end
            end
        end

        if improved 
            best_obj = cur_best_obj 
            best_tour = cur_best_tour
            best_t_route = cur_best_t_route
            best_d_route = cur_best_d_route
        end
    end

    return best_obj, best_t_route, best_d_route
end
