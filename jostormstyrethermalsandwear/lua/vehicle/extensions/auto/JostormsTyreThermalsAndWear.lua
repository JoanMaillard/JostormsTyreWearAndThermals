groundModels = {} -- Intentionally global
groundModelsLut = {} -- Intentionally global

local M = {}

local BASELINE_TYRE_VOLUME = 0.029 -- in meters cubed
local BASELINE_OUTER_TYRE_AREA = 0.474 -- in meters squared
local BASELINE_INNER_TYRE_AREA = 0.147 -- in meters squared
local BASELINE_TYRE_SKIN_AREA = 0.197 -- in meters squared
local BASELINE_TYRE_WIDTH = 0.215 -- in meters

local WORKING_TEMP = 60         -- The average ideal working temperature for your tyres
local ENV_TEMP = 21             -- In celsius. Represents both the outside air and surface temp in 1 variable.
local SLIP_ENERGY_TEMP_NORMALISATION_FACTOR = 0.001375
local SLIP_ENERGY_WEAR_NORMALISATION_FACTOR = 0.00035
local LOAD_ENERGY_TEMP_NORMALISATION_FACTOR = 0.000235
local HORIZONTAL_LOAD_WEAR_NORMALISATION_FACTOR = 0.000035

local SKIN_THERMAL_PERMITTANCE = 0.5  -- Global modifier for how fast temperature changes
local SKIN_INTERNAL_HEAT_EXCHANGE_RATE = 2.0 -- Modifier for how fast the temperature inside the tyre skin unifies
local SKIN_OUTSIDE_HEAT_EXCHANGE_RATE = 0.1 -- Modifier for how fast temperature cools down related to ENV_TEMP
local AIRFLOW_TREAD_NORMALISATION_FACTOR = 0.05 -- modifier for how much the tread coefficient increases air surface
local TREAD_COHERENCE_NORMALISATION_FACTOR = 1
local ROLLING_RESISTANCE_HEAT_FACTOR = 0.01 -- Modifier for how much the rolling resistance of the tyre affects temperature
local SOFTNESS_TO_DEFORMATION_FACTOR = 1.5 -- Modifier for how much the softness of the tyre affects its heating up through deformation


local CORE_THERMAL_PERMITTANCE = 0.4 -- Global modifier for how fast the core temperature changes
local CORE_BRAKES_HEAT_EXCHANGE_RATE = 0.004    -- Modifier for how fast core temperature rises from brake temp
local CORE_SKIN_HEAT_EXCHANGE_RATE = 0.5     -- Modifier for how fast core temperature flows to and from skin temperature

local WEAR_RATE = 0.015

local tyreData = {}
local wheelCache = {}


local totalTimeMod60 = 0

-- Research notes on tyre thermals and wear:
-- - Thermals have an obvious impact on grip, but not as much as wear.
-- - Tyres that are too hot or cold wear quicker (although for different reasons).
-- - Tyre pressure heavily affects the thermals and wear (mostly thermals I think).
-- - Brake temperature influences tyre thermals a decent amount as well.

local function sigmoid(x, k)
    local k = k or 10
    return 1 / (1 + k^-x)
end

local function lerp(base, value, slope) -- linear interpolation
    return base + (value - base) * slope
end

local function GetGroundModelData(id)
    local materials, materialsMap = particles.getMaterialsParticlesTable()
    local matData = materials[id] or {}
    local name = matData.name or "DOESNT EXIST"
    -- local name = groundModelsLut[id] or "DOESNT EXIST"
    local data = groundModels[name] or { staticFrictionCoefficient = 1, slidingFrictionCoefficient = 1 }
    return name, data
end

local function CalcBiasWeights(loadBias)
    local weightLeft = math.max(math.min(loadBias, 0) * -0.8, 0.2)
    local weightRight = math.max(math.max(loadBias, 0) * 0.8, 0.2)
    local weightCenter = 1 - weightLeft - weightRight
    return { weightLeft, weightCenter, weightRight }
end

local function TempRingsToAvgTemp(temps, loadBias)
    local weights = CalcBiasWeights(loadBias)
    return temps[1] * weights[1] + temps[2] * weights[2] + temps[3] * weights[3]
end

local function GetTyreVolume(tyreOuterRadius, tyreInnerRadius, tyreWidth) 
    local outerVolume = math.pi * tyreOuterRadius * tyreOuterRadius * tyreWidth
    local innerVolume = math.pi * tyreInnerRadius * tyreInnerRadius * tyreWidth
    return outerVolume - innerVolume
end

local function GetTyreAreas(tyreOuterRadius, tyreInnerRadius, tyreWidth)
    local sidesOuterArea = math.pi * tyreOuterRadius * tyreOuterRadius
    local sidesInnerArea = math.pi * tyreInnerRadius * tyreInnerRadius
    local sidesArea = (sidesOuterArea - sidesInnerArea) * 2

    local outerSurface = math.pi * tyreOuterRadius * tyreWidth
    local innerSurface = math.pi * tyreInnerRadius * tyreWidth
    return { sidesArea + outerSurface, outerSurface, innerSurface }
end

local function TyreBandDeltaTemp(
    dt,
    bandTemp,
    coreTemp,
    workingTemp,
    angularVel,
    slipEnergy,
    avgTemp,
    loadCoeffWeight,
    softnessCoef,
    treadCoef,
    horizontalLoad,
    staticFrictionCoef,
    dynamicFrictionCoef,
    adjacentTemps,
    skinTotalPermittance,
    skinTotalOutsideExchangeRate,
    wheelWidthDeformationFactor
)
    -- load-independent changes
    local deltaTempFromOptimal = math.abs(bandTemp - workingTemp)
    local airflowMagnitudeCoeff = (1 + (angularVel / 250)^2) * (1 + treadCoef * AIRFLOW_TREAD_NORMALISATION_FACTOR)
    local skinOutsideTempDelta = (ENV_TEMP - bandTemp) * skinTotalPermittance * dt * skinTotalOutsideExchangeRate * airflowMagnitudeCoeff
    
    local skinTempSmoothingDelta = 0
    for _, entry in ipairs(adjacentTemps) do
        skinTempSmoothingDelta = skinTempSmoothingDelta + ((entry - bandTemp) * skinTotalPermittance * SKIN_INTERNAL_HEAT_EXCHANGE_RATE * dt) / #adjacentTemps
    end

    -- load-dependant changes
    local bandSlipEnergy = slipEnergy * loadCoeffWeight -- each band is affected depending on load
    local rollingHeatDelta = angularVel * ROLLING_RESISTANCE_HEAT_FACTOR
    local skinSlipLoadTempDelta =
        (bandSlipEnergy * SLIP_ENERGY_TEMP_NORMALISATION_FACTOR * dynamicFrictionCoef + rollingHeatDelta)
        * softnessCoef * SOFTNESS_TO_DEFORMATION_FACTOR * wheelWidthDeformationFactor
        * skinTotalPermittance
        * dt

    local bandLoadEnergy = horizontalLoad * loadCoeffWeight
    
    local skinLoadTempDelta =
        bandLoadEnergy * LOAD_ENERGY_TEMP_NORMALISATION_FACTOR
        * softnessCoef * SOFTNESS_TO_DEFORMATION_FACTOR * wheelWidthDeformationFactor
        * skinTotalPermittance
        * staticFrictionCoef
        * dt

    local skinCoreTempDelta = (coreTemp - avgTemp) * CORE_SKIN_HEAT_EXCHANGE_RATE * dt

    local bandDelta = skinOutsideTempDelta + skinTempSmoothingDelta + skinCoreTempDelta + skinSlipLoadTempDelta + skinLoadTempDelta
    return bandDelta
end

local function RecalcTyreTemp(
    dt,
    defaultTyreData,
    wheelID,
    loadBias,
    slipEnergy,
    angularVel,
    brakeTemp,
    softnessCoef,
    treadCoef,
    horizontalLoad,
    staticFrictionCoef,
    dynamicFrictionCoef,
    wheelRadius,
    rimRadius,
    wheelWidth
)
    
    local data = tyreData[wheelID] or defaultTyreData
    local weights = CalcBiasWeights(loadBias)
    local avgTemp = TempRingsToAvgTemp(data.temp, loadBias)
    local tyreTempAdjacenceMatrix = {
        {data.temp[2]}, 
        {data.temp[1], data.temp[3]}, 
        {data.temp[2]}
    }

    -- Calculating temperature change of the core
    local avgSkinTemp = (data.temp[1] + data.temp[2] + data.temp[3]) / 3
    local skinCoreDeltaT = (avgSkinTemp - data.temp[4])
    local coreBrakeDeltaT = (brakeTemp - data.temp[4])

    local coreTotalPermittance = (BASELINE_TYRE_VOLUME / GetTyreVolume(wheelRadius, rimRadius, wheelWidth)) * CORE_THERMAL_PERMITTANCE
    local tyreAreas = GetTyreAreas(wheelRadius, rimRadius, wheelWidth)
    local skinAreaFactor = (BASELINE_TYRE_SKIN_AREA / tyreAreas[2])
    local skinTotalPermittance = skinAreaFactor * SKIN_THERMAL_PERMITTANCE
    local skinTotalOutsideExchangeRate = (tyreAreas[1] / BASELINE_OUTER_TYRE_AREA) * SKIN_OUTSIDE_HEAT_EXCHANGE_RATE
    local coreTotalBrakeExchangeRate = (BASELINE_INNER_TYRE_AREA / tyreAreas[3]) * CORE_BRAKES_HEAT_EXCHANGE_RATE
    local wheelWidthDeformationFactor = (BASELINE_TYRE_WIDTH / wheelWidth)

    local coreHeatupFromSkin = skinCoreDeltaT * coreTotalPermittance * dt * CORE_SKIN_HEAT_EXCHANGE_RATE
    local coreHeatupFromBrakes = coreBrakeDeltaT * coreTotalPermittance * dt * coreTotalBrakeExchangeRate

    data.temp[4] = data.temp[4] + coreHeatupFromSkin + coreHeatupFromBrakes

    for i=1,3 do -- for each tyre band, inside - middle - outside
        data.temp[i] = data.temp[i] + TyreBandDeltaTemp(
            dt,
            data.temp[i],
            data.temp[4],
            data.working_temp,
            angularVel,
            slipEnergy,
            avgTemp,
            weights[i],
            softnessCoef,
            treadCoef,
            horizontalLoad,
            staticFrictionCoef,
            dynamicFrictionCoef,
            tyreTempAdjacenceMatrix[i],
            skinTotalPermittance,
            skinTotalOutsideExchangeRate,
            wheelWidthDeformationFactor
        )
    end
end


-- Calculate tyre wear and thermals based on tyre data
local function RecalcTyreWear(dt, defaultTyreData, wheelID, groundModel, loadBias, slipEnergy, softnessCoef, horizontalLoad)
    -- Wear is mainly based on the same information as thermals
    -- but also thermals. The further from working temperatures, the
    -- more the tyre will wear in general.
    
    local data = tyreData[wheelID] or defaultTyreData
    local avgTemp = TempRingsToAvgTemp(data.temp, loadBias)
    local thermalCoeff = (math.abs(avgTemp - data.working_temp) / data.working_temp)^0.8
    local wear = 
        (slipEnergy * SLIP_ENERGY_WEAR_NORMALISATION_FACTOR * groundModel.slidingFrictionCoefficient 
            + horizontalLoad * HORIZONTAL_LOAD_WEAR_NORMALISATION_FACTOR * groundModel.staticFrictionCoefficient) 
        * WEAR_RATE 
        * softnessCoef * SOFTNESS_TO_DEFORMATION_FACTOR 
        * dt 
        * math.max(thermalCoeff, 0.5) 

    data.condition = math.max(data.condition - wear, 0)

    tyreData[wheelID] = data
end

local function CalculateTyreGrip(wheelID, loadBias, treadCoef)
    local data = tyreData[wheelID]

    local avgTemp = TempRingsToAvgTemp(data.temp, loadBias)

    local tyreGrip = 1
    if data.condition > 25 then
        tyreGrip = tyreGrip * (math.min(data.condition / 97, 1)^3.5 * 0.22 + 0.78)
    else
        tyreGrip = tyreGrip * (math.min(data.condition / 100, 0.01)^0.05 - 0.15)
    end        
    local tempDist = math.abs(avgTemp - data.working_temp)
    local tempLerpValue = (tempDist / data.working_temp)^0.8
    tyreGrip = tyreGrip * lerp(1, 0.82, tempLerpValue * (treadCoef + 0.3))

    return tyreGrip
end

-- This is a special function that runs every frame, and has full access to
-- vehicle data for the current vehicle.
local function updateGFX(dt)
    local stream = { data = {} }

    local vectorForward = obj:getDirectionVector()
    local vectorUp = obj:getDirectionVectorUp()
    local vectorRight = vectorForward:cross(vectorUp)

    local isAI = false

    for i, wd in pairs(wheels.wheelRotators) do
        local w = wheelCache[i] or {}
        w.name = wd.name
        w.radius = wd.radius
        w.tireWidth = wd.tireWidth
        w.tireVolume = wd.tireVolume
        w.hubRadius = wd.hubRadius
        w.wheelDir = wd.wheelDir
        w.angularVelocity = wd.angularVelocity
        w.propulsionTorque = wd.propulsionTorque
        w.lastSlip = wd.lastSlip
        w.lastSideSlip = wd.lastSideSlip
        w.slipEnergy = wd.slipEnergy
        w.downForce = wd.downForce
        w.brakingTorque = wd.brakingTorque
        w.brakeTorque = wd.brakeTorque
        w.contactMaterialID1 = wd.contactMaterialID1
        w.contactMaterialID2 = wd.contactMaterialID2
        w.treadCoef = wd.treadCoef
        w.softnessCoef = wd.softnessCoef
        w.isBroken = wd.isBroken
        w.isTireDeflated = wd.isTireDeflated
        w.brakeCoreTemperature = wd.brakeCoreTemperature or ENV_TEMP -- Fixes AI

        -- Get camber/toe/caster data
        w.camber = (90 - math.deg(math.acos(obj:nodeVecPlanarCos(wd.node2, wd.node1, vectorUp, vectorRight))))

        wheelCache[i] = w
    end

    -- Based on sensor data, we can estimate how far the load is shifted left-right on the tyre
    local tyreTreadLRLoadShift = sensors.gx2 / 5
    tyreTreadLRLoadShift = sigmoid(tyreTreadLRLoadShift, 2) * 2 - 1
    -- We don't use this system for front-back load, because we can simply guess this

    for i=0,#wheels.wheelRotators do
        local wheel = obj:getWheel(i)
        if wheel then
            local groundModelName, groundModel = GetGroundModelData(wheelCache[i].contactMaterialID1)

            local staticFrictionCoefficient = groundModel.staticFrictionCoefficient
            print("Static coef: " .. staticFrictionCoefficient)
            local slidingFrictionCoefficient = groundModel.slidingFrictionCoefficient
            print("Sliding coef: " .. staticFrictionCoefficient)

            local angularVel = math.max(math.abs(wheelCache[i].angularVelocity), obj:getVelocity():length() * 3)
            angularVel = math.floor(angularVel * 10) / 10 -- Round to reduce small issues
            local slipWearFactor = wheelCache[i].slipEnergy
            -- Multiply with wheel direction to flip the torque for right side wheels
            local brakeTemp = wheelCache[i].brakeCoreTemperature

            local treadCoef = wheelCache[i].treadCoef
            local softnessCoef = wheelCache[i].softnessCoef
            local verticalLoad = wheelCache[i].downForce * (1 + treadCoef * TREAD_COHERENCE_NORMALISATION_FACTOR)
            local sideAccel = sensors.gx2
            local horizontalLoad = verticalLoad * math.abs(sideAccel)
            local loadBias = tyreTreadLRLoadShift * 0.22 * softnessCoef + (wheelCache[i].camber / 12) * wheelCache[i].wheelDir
            loadBias = sigmoid(loadBias * 2.5, 5) * 2 - 1

            local default_working_temp = WORKING_TEMP * (math.sqrt(-0.5 * treadCoef + 2))
            local defaultTyreData = {
                working_temp = default_working_temp,
                temp = { default_working_temp, default_working_temp, default_working_temp, default_working_temp },
                condition = 100, -- 100% perfect tyre condition
            }

            RecalcTyreTemp(
                dt,
                defaultTyreData,
                i,
                loadBias,
                slipWearFactor,
                angularVel,
                brakeTemp,
                softnessCoef,
                treadCoef,
                horizontalLoad,
                groundModel.staticFrictionCoefficient,
                groundModel.slidingFrictionCoefficient,
                wheelCache[i].radius,
                wheelCache[i].hubRadius,
                wheelCache[i].tireWidth
            )

            RecalcTyreWear(
                dt,
                defaultTyreData,
                i,
                groundModel,
                loadBias,
                slipWearFactor,
                softnessCoef,
                horizontalLoad
            )

            local tyreGrip = CalculateTyreGrip(i, loadBias, treadCoef)
            

            local temps = {}
            for j=1,4 do
                table.insert(temps, math.floor(tyreData[i].temp[j] * 10) / 10)
            end
            table.insert(stream.data, {
                name = wheelCache[i].name,
                tread_coef = treadCoef,
                working_temp = math.floor(tyreData[i].working_temp * 10) / 10,
                temp = temps,
                avg_temp = math.floor(TempRingsToAvgTemp(tyreData[i].temp, loadBias) * 10) / 10,
                condition = math.floor(tyreData[i].condition * 10) / 10,
                tyreGrip = math.floor(tyreGrip * 1000) / 1000,
                load_bias = loadBias,
                contact_material = groundModelName,
                brake_temp = brakeTemp,
                brake_working_temp = 800
            })

            if tyreData[i].condition <= 0 or tyreData[i].temp[4] >= 3.5 * tyreData[i].working_temp then
                beamstate.deflateTire(i)
            end


            wheel:setFrictionThermalSensitivity(
                -300,       -- frictionLowTemp              default: -300
                1e7,        -- frictionHighTemp             default: 1e7
                1e-10,      -- frictionLowSlope             default: 1e-10
                1e-10,      -- frictionHighSlope            default: 1e-10
                10,         -- frictionSlopeSmoothCoef      default: 10
                tyreGrip,   -- frictionCoefLow              default: 1
                tyreGrip,   -- frictionCoefMiddle           default: 1
                tyreGrip    -- frictionCoefHigh             default: 1
            )
            
        end
    end
    totalTimeMod60 = (totalTimeMod60 + dt) % 60 -- Loops every 60 seconds
    stream.total_time_mod_60 = totalTimeMod60
    gui.send("TyreWearThermals", stream)
end

local function onReset()
    tyreData = {}

    obj:queueGameEngineLua("scripts_JostormsTyreThermalsAndWear_extension.getGroundModels()")
end

local function onInit()
    obj:queueGameEngineLua("scripts_JostormsTyreThermalsAndWear_extension.getGroundModels()")
end

M.onInit = onInit
M.onReset = onReset
M.updateGFX = updateGFX
M.groundModelsCallback = groundModelsCallback

return M
