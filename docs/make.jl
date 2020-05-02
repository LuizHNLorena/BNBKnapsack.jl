using Documenter, BNBKnapsack

makedocs(;
    modules=[BNBKnapsack],
    format=Documenter.HTML(),
    pages=[
        "Home" => "index.md",
    ],
    repo="https://github.com/LuizHNLorena/BNBKnapsack.jl/blob/{commit}{path}#L{line}",
    sitename="BNBKnapsack.jl",
    authors="Luiz Henrique Nogueira Lorena, ICT-UNIFESP",
    assets=String[],
)

deploydocs(;
    repo="github.com/LuizHNLorena/BNBKnapsack.jl",
)
