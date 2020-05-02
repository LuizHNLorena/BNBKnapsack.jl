using BNBKnapsack
using Test

@testset "BNBKnapsack.jl" begin
    # Execute the BNB
    #@test BNBKnapsack.execute(BNBKnapsack.Problem(4,[50, 40, 45, 55],[5, 4, 3, 4],13,1e-4, 1e-3)) == 0
end

n_objects = 4               # Also the number of variables.
values = [16, 22, 12, 8]
weights = [5, 7, 4, 3]
max_weight = 14


integer_precision = 1e-4    # Integers are represented as floating-point values: if they are close enough to an integer, they are considered integer.
gap_precision = 1e-3

problem = BNBKnapsack.Problem(n_objects,values,weights,max_weight,integer_precision,gap_precision)
BNBKnapsack.execute(problem)
