using BNBKnapsack
using Test

@testset "BNBKnapsack.jl" begin
    # Execute the BNB
    @test BNBKnapsack.execute(BNBKnapsack.Problem(4,[50, 40, 45, 55],[5, 4, 3, 4],13,1e-4, 1e-3)) == 0
end
