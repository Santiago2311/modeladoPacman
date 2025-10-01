using Agents

@agent struct Ghost(GridAgent{2})
    type::String = "Ghost"
end

possible_transitions = [(1, 0), (0, 1), (-1, 0), (0, -1)]

function agent_step!(agent, model)
    posible_moves = []
    for movement in possible_transitions
        new_pos = agent.pos .+ movement
        if all(1 .<= new_pos .<= size(abmspace(model))) && matrix[new_pos[2]][new_pos[1]] == 1
            push!(posible_moves, movement)
        end
    end
    if !isempty(posible_moves)
        walk!(agent, rand(posible_moves), model)
    end
end

function initialize_model()
    space = GridSpace((17, 14); periodic=false, metric=:manhattan)
    model = StandardABM(Ghost, space; agent_step!)
    return model
end

model = initialize_model()
a = add_agent!(Ghost, pos=(3, 3), model)