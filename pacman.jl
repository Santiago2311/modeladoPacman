using Agents, Agents.Pathfinding

@agent struct Ghost(GridAgent{2})
    type::String = "Ghost"
end

possible_transitions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
matrix = [
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 0;
    0 1 0 1 0 0 0 1 1 1 0 1 0 1 0 1 0;
    0 1 1 1 0 1 0 0 0 0 0 1 0 1 1 1 0;
    0 1 0 0 0 1 1 1 1 1 1 1 0 0 0 1 0;
    0 1 0 1 0 1 0 0 0 0 0 1 1 1 0 1 0;
    0 1 1 1 0 1 0 1 1 1 0 1 0 1 0 1 0;
    0 1 0 1 0 1 0 1 1 1 0 1 0 1 0 1 0;
    0 1 0 1 1 1 0 0 1 0 0 1 0 1 1 1 0;
    0 1 0 0 0 1 1 1 1 1 1 1 0 0 0 1 0;
    0 1 1 1 0 1 0 0 0 0 0 1 0 1 1 1 0;
    0 1 0 1 0 1 0 1 1 1 0 0 0 1 0 1 0;
    0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 0;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
]

start_pos = (2,2)
target_pos = (16,13)
walkmaps = BitArray(falses(size(matrix)))

for i in 1:size(matrix)[1], j in 1:size(matrix)[2]
    if matrix[i, j] == 1
        walkmaps[i, j] = true
    end
end

function agent_step!(agent, model)
    if agent.pos == target_pos
        return
    end

    if is_stationary(agent, model.pathfinder)
        plan_route!(agent, target_pos, model.pathfinder)
    end

    move_along_route!(agent, model, model.pathfinder)
end

function initialize_model()
    space = GridSpace((size(matrix,2), size(matrix,1)); periodic=false, metric=:manhattan)
    
    walkmaps = BitMatrix((matrix .== 1)')

    pathfinder = AStar(space; walkmap=walkmaps, diagonal_movement=false)

    model = StandardABM(Ghost, space; agent_step!, properties=Dict(:pathfinder => pathfinder))
    return model
end


model = initialize_model()
a = add_agent!(start_pos, model)