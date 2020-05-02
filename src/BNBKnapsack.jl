module BNBKnapsack

using LinearAlgebra
    using JuMP, CPLEX
    import Base.print

    mutable struct Problem
        n_objects::Int64        # Also the number of variables.
        values::Array{Int64,1}
        weights::Array{Int64,1}
        max_weight::Int64
        integer_precision::Float64  # Integers are represented as floating-point values: if they are close enough to an integer, they are considered integer.
        gap_precision::Float64
    end

    problem = nothing

    mutable struct BBNode
        # Tree structure.
        parent                              #::Union(Nothing, BBNode)
        children::Array{BBNode, 1}
        # Branching information.
        variable::Int                       # Just an index of the variable for the branching. Indexing starts at 1 (0: no branching; only for root node).
        variable_value::Int
        # Node's LP information.
        lp_value::Float64
        lp_feasibility                      #::Symbol # :Optimal or something else.
        solution::Array{Float64, 1}
        integer_feasible::Bool
    end

    mutable struct BBStatus
        upper_bound::Float64 # Computed from the LPs.
        lower_bound::Float64 # Incumbent.
        incumbent::Array{Int, 1} # Rounded from floating-point solution.
        root::BBNode
        # Optimisations: list of nodes to avoid going through all the tree, sort nodes so the most interesting ones are picked up first, etc.
    end

    function get_lp()
        m = Model(CPLEX.Optimizer)
        @variable(m, 0 <= x[1:problem.n_objects] <= 1)
        @constraint(m, dot(problem.weights, x) <= problem.max_weight)
        @objective(m, Max, dot(problem.values, x))
        return m, x
    end

    function solution_is_boolean(sol::Array{Float64,1})
        for s in sol
            if s > problem.integer_precision && s < 1 - problem.integer_precision
                return false
            end
        end
        return true
    end

    function gap(status::BBStatus)
        # Will return NaN if no integer-feasible solution is found.
        return (status.upper_bound - status.lower_bound) / status.upper_bound
    end

    function is_pruned(node::BBNode, status::BBStatus; quiet::Bool = true)
        if node.lp_feasibility != :Optimal
            if !quiet && node.variable != 0                 # If not the root
                println("Node pruned by infeasibility.")
            end
            return true, :Feasibility
        end
        if node.integer_feasible
            if !quiet println("Node pruned by optimality.") end
            return true, :Optimality
        end
        if node.lp_value < status.lower_bound
            if !quiet println("Node pruned by bounds.") end
            return true, :Bounds
        end
        return false, :None
    end

    function get_next_node(status)
        # Find a node which has no children and is not pruned.
        # Pruning is then used here, together with tree-exploration strategy!
        function explore_node(node::BBNode)
            pruned, why_pruned = is_pruned(node, status; quiet=false)
            if pruned
                return nothing
            end
            if length(node.children) == 0
                println("self no children: $node")
                return node
            end
            # Binary variables: two children or zero, no other case.
            node_left = node.children[1]
            if node_left != nothing
                possible_left = explore_node(node_left)
            else
                possible_left = nothing
            end
            node_right = node.children[2]
            if node_right != nothing
                possible_right = explore_node(node_right)
            else
                possible_right = nothing
            end
            if possible_left != nothing
                return possible_left
            elseif possible_right != nothing
                return possible_right
            end
        end
        return explore_node(status.root)
    end

    function is_variable_branched_on(node::BBNode, var::Int)
        # Depends on the path followed from root to now.
        if node.variable == var
            return true
        end
        if node.variable == 0 # Root node.
            return false
        end
        return is_variable_branched_on(node.parent, var)
    end

    function find_branching_variable(sol::Array{Float64,1}, current_node::BBNode)
        # Only called on non-integer solutions.
        n_vars = length(sol)
        for s in 1:n_vars
            v = sol[s]
            if v > problem.integer_precision && v < 1 - problem.integer_precision && ! is_variable_branched_on(current_node, s)
                return s
            end
        end
        println(" -> Where shall I branch?")
    end

    function branch_lp(current_node::BBNode, new_variable::Int, value::Int)
        lp, vars = get_lp()
        # Retrieve the previous nodes' branching decisions.
        node = current_node
        while true
            if node.variable == 0
                break
            end
            @constraint(lp, vars[node.variable] == node.variable_value)
            node = node.parent
        end
        @constraint(lp, vars[new_variable] == value)
        return lp, vars
    end

    function create_node(parent::BBNode, new_variable::Int, value::Int)
        lp, vars = branch_lp(parent, new_variable, value)
        lp_feasibility = optimize!(lp)
        lp_value = JuMP.objective_value(lp)
        lp_solution = JuMP.value.(vars)
        int_feas = solution_is_boolean(lp_solution)
        return BBNode(parent, [], new_variable, value, lp_value, lp_feasibility, lp_solution, int_feas)
    end

    function round_solution(sol::Array{Float64, 1})
        return [(s < .5) ? 0 : 1 for s in sol]
    end

    function get_upper_bound(status::BBStatus)
        # As the tree is developed such that both children are always created at the same time,
        # retrieve the leaves' LP values, take the minimum of them.
        function get_leaves(node::BBNode)
            if length(node.children) == 0
                pruned, why = is_pruned(node, status)
                if node.integer_feasible || (pruned && why != :Bounds)
                    return Inf
                else
                    return node.lp_value
                end
            else
                left = get_leaves(node.children[1])
                right = get_leaves(node.children[2])
                return min(left, right)
            end
        end
        return get_leaves(status.root)
    end

    function print(tree::BBNode, status::BBStatus)
        function space(level::Int)
            return "  " ^ level
        end
        function sub_print(node::BBNode, level::Int)
            print(" . ")
            print(space(level))

            if node.variable != 0
                print("Variable " * string(node.variable) * "   :   " * string(node.variable_value) * "   ;   " * string(node.lp_value))
            else
                print("Root node:               " * string(node.lp_value))
            end

            if node.integer_feasible
                print("  (I)")
            else
                print("     ")
            end

            pruned, why_pruned = is_pruned(node, status; quiet=true)
            if pruned
                print("  (P: " * string(why_pruned) * ")")
            else
                print("     ")
            end

            print("\n")

            for child in node.children
                sub_print(child, level + 1)
            end
        end
        sub_print(tree, 2)
    end

    function print(status::BBStatus)
        println(" . Complete branch-and-bound status, including tree.")
        println(" . Upper bound: " * string(status.upper_bound))
        println(" . Lower bound: " * string(status.lower_bound))
        println(" . Incumbent: " * string(status.incumbent))
        println(" . Tree: ")
        print(status.root, status)
        print("\n")
    end

    function execute(p::BNB_knapsack.Problem)
        global problem = p

        println("\n=====================\n")

        root_lp, root_vars = get_lp()
        root_feasibility = optimize!(root_lp)

        if root_feasibility != :Optimal
            println("Root node could not be solved to optimality: $root_feasibility")
        end

        root_solution = JuMP.value.(root_vars)
        root_obj = JuMP.objective_value(root_lp)
        if solution_is_boolean(root_solution)
            println("Solved at root node.")
            println(root_solution)
            println("Objective: $root_obj")
            return
        end

        # parent, children, variable, variable_value, lp_value, lp_feasibility, solution, integer_feasible
        root = BBNode(nothing, [], 0, 0, root_obj, root_feasibility, root_solution, false)
        # upper_bound, lower_bound, incumbent, root
        status = BBStatus(root_obj, -Inf, [], root)

        println("[0] Root node solved. LP value: $root_obj.")

        # Node exploring: do both children at once, while exploring current node. Not what is usually done, but easier
        # (no list of nodes to explore).
        n_nodes_explored = 0
        while gap(status) > problem.gap_precision

            n_nodes_explored += 1

            println("[ $n_nodes_explored ] Exploring one more node.")

            # Find a node to explore. There may be none.
            node_to_explore = 0
            if n_nodes_explored == 1
                node_to_explore = root
            else
                node_to_explore = get_next_node(status)
            end

            if node_to_explore == nothing && n_nodes_explored != 1
                println("No more branches to explore in the tree.")
                break
            end

            # Decide the branching variable.
            var_index = 0
            if n_nodes_explored == 1
                var_index = find_branching_variable(root.solution, root)
            else
                var_index = find_branching_variable(node_to_explore.solution, node_to_explore)
            end

            println("[ $n_nodes_explored ] Branching on variable $var_index")

            # Create new nodes with branching on this *binary* variable (with all constraints from root to this new branch).
            left = create_node(node_to_explore, var_index, 0)
            right = create_node(node_to_explore, var_index, 1)

            # Update the parent with the new children.
            node_to_explore.children = [left, right]

            # Update the bounds and, if necessary, update the incumbent.
            if left.integer_feasible && left.lp_value > status.lower_bound
                println("[ $n_nodes_explored ] Found a new incumbent. Value: $(left.lp_value)")
                status.lower_bound = left.lp_value
                status.incumbent = round_solution(left.solution)
            end

            if right.integer_feasible && right.lp_value > status.lower_bound
                println("[ $n_nodes_explored ] Found a new incumbent. Value: $(right.lp_value)")
                status.lower_bound = right.lp_value
                status.incumbent = round_solution(right.solution)
            end
            status.upper_bound = get_upper_bound(status)

            # End iteration.
            println("[ $n_nodes_explored ] Iteration done. Gap: $(gap(status)). Bound: [ $(status.lower_bound) ; $(status.upper_bound) ].")

        end
        print(status)
        println("\n\n")
    end
    
end # module
