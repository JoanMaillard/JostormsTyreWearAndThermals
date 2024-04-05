local M = {}

function getGroundModels()
    local cmd = "groundModels = {"
    local i = 1
    for k, v in pairs(core_environment.groundModels) do
        local name = tostring(k)
        if #name > 0 then
            cmd = cmd .. name .. " = { staticFrictionCoefficient = " .. v.cdata.staticFrictionCoefficient .. ", slidingFrictionCoefficient = " .. v.cdata.slidingFrictionCoefficient .. " }, "
        end
    end
    cmd = cmd .. "debug = 0 };"
    be:getPlayerVehicle(0):queueLuaCommand(cmd)
end

M.getGroundModels = getGroundModels

return M
