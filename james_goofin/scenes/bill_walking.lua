--[[
Description: Bill's walking controller for CoppeliaSim human-aware mobile manipulator scene

    Modes (BILL_MODE)
        0     Static Bill. Bill's placed at one of a few predefined test spots
        1     Walking Bill. Bill walks around at constant speed
        666   Wild Bill. Bill walks around with random speed and direction changes

Inputs:
    CoppeliaSim scene (james_goofin/scenes/_main_scene.ttt) with
        /Bill
        /Path_Both      -- Bill's safety zone can interfere with Omni + UR5 Reach
        /Path_UR5Only   -- Bill's safety zone can interfere with UR5 reach only

        static test dummies
            /BillSpot_1     fully outside robot reach
            /BillSpot_2     inside UR5/barrel operating area
            /BillSpot_3     blocking OmniPlatform path

Settings:
    BILL_MODE       Selects Bill's behavior
    BILL_SPEED      Walking speed in m/s when BILL_MODE = 1
    BILL_TEST_SPOT  Selects what predefined BillSpot is used when BILL_MODE = 0

Outputs:
    Bill's either held at a static test location or he's moving around the Path. Go Bill Go!
    While walking, Bill's animations update and his orientation follows direction of travel

NOTES:
    - Path is a separate scene object, not one of Bill's children (Speaking of, are his kids coming over for the Holidays?)
    - BILL_MODE = 0 is used for repeatable static human-position tests
    - BILL_MODE = 1 is used for predictable moving-human tests
    - BILL_MODE = 666 introduces mild changes in walking speed and direction
    - Bill's position's updated directly from the Path each simulation step
    - The robot's human-safety response is handled separately by barrel_pick_place.py.
--]]


sim = require'sim'


BILL_MODE      = 1      -- 0 = static Bill, 1 = constant-speed Bill, 666 = Chaos Bill
BILL_PATH      = 1      -- 1 = both Omni + UR5 reach, 2 = UR5 reach only
BILL_SPEED     = 0.7    -- Bill's speed; m/s when BILL_MODE = 1
BILL_TEST_SPOT = 1      -- Only used when BILL_MODE = 0
-- 1 = outside robot reach
-- 2 = inside UR5/barrel area
-- 3 = blocking OmniPlatform path


function sysCall_init() 
    BillHandle=sim.getObject('./Bill')

    if BILL_MODE == 0 then
        local billSpots={
            sim.getObject('/BillSpot_1'),   -- Dummy marking safe/outside position
            sim.getObject('/BillSpot_2'),   -- Dummy marking position inside UR5/barrel operating area
            sim.getObject('/BillSpot_3')    -- Dummy marking position directly in OmniPlatform travel path
            }
        
        local spot = billSpots[BILL_TEST_SPOT]          -- Select whichever BillSpot matches BILL_TEST_SPOT
        local pos  = sim.getObjectPosition(spot,-1)     -- Read selected dummy's world position
        local ori  = sim.getObjectOrientation(spot,-1)  -- Read selected dummy's world orientation

        sim.setObjectPosition(BillHandle,pos,-1)        -- move Bill to current dummy position
        sim.setObjectOrientation(BillHandle,ori,-1)     -- match Bill's orientation to current dummy
    end

    legJointHandles      = {sim.getObject('./leftLegJoint'),      sim.getObject('./rightLegJoint')}
    kneeJointHandles     = {sim.getObject('./leftKneeJoint'),     sim.getObject('./rightKneeJoint')}
    ankleJointHandles    = {sim.getObject('./leftAnkleJoint'),    sim.getObject('./rightAnkleJoint')}
    shoulderJointHandles = {sim.getObject('./leftShoulderJoint'), sim.getObject('./rightShoulderJoint')}
    elbowJointHandles    = {sim.getObject('./leftElbowJoint'),    sim.getObject('./rightElbowJoint')}
    neckJoint            = sim.getObject('./neck')
    --pathHandle = sim.getObject('./path')
    --pathHandle           = sim.getObject('/Path')  -- path Bill follows, made another one not tied to Bill, not his kid
    if BILL_PATH == 1 then
        pathHandle = sim.getObject('/Path_Both')    -- Bill can interfere with Omni and UR5 reach
    elseif BILL_PATH == 2 then
        pathHandle = sim.getObject('/Path_UR5Only') -- Bill only enters UR5 reach
    end

    --pausePoints={}                                                               --commented out, don't need
    --pausePoints[1]={handle=sim.getObject('./pausePoint[0]'),duration=3,dir=3}    --commented out, don't need
    --pausePoints[2]={handle=sim.getObject('./pausePoint[1]'),duration=2,dir=2}    --commented out, don't need
    --setPausePointVisibility(0)                                                   --commented out, don't need
    
    local pathData = sim.unpackDoubleTable(sim.getBufferProperty(pathHandle, 'customData.PATH', {noError = true}))
    local m        = Matrix(#pathData//7,7,pathData)

    pathPose          = sim.getObjectPose(pathHandle)
    pathPositions     = m:slice(1,1,m:rows(),3):data()
    pathPositions     = sim.multiplyVector(pathPose,pathPositions)
    pathLengths,pathL = sim.getPathLengths(pathPositions,3)
    
    velocity             = 1
    setRandomColors      = true
    startPosInitialPause = 0
    startPosPause        = 3
    goalPosPause         = 3
    maxJourneys          = 9999
   
    -- Now we set random colors:
    HairColors    = {4, {0.30,0.22,0.14}, {0.75,0.75,0.75}, {0.075,0.075,0.075}, {0.75,0.68,0.23}}
    skinColors    = {2, {0.91,0.84,0.75}, {0.52,0.45,0.35}}
    shirtColors   = {5, {0.37,0.46,0.74}, {0.54,0.27,0.27}, {0.31,0.51,0.33}, {0.46,0.46,0.46}, {0.18,0.18,0.18}}
    trouserColors = {2,   {0.4,0.34,0.2}, {0.12,0.12,0.12}}
    shoeColors    = {2, {0.12,0.12,0.12}, {0.25,0.12,0.045}}
    if setRandomColors then
        math.randomseed(sim.getFloatProperty(sim.handle_app, 'randomFloat')*10000) -- each lua instance should start with a different and 'good' seed
        setColor('HAIR', HairColors[1+math.random(HairColors[1])])
        setColor('SKIN', skinColors[1+math.random(skinColors[1])])
        setColor('SHIRT', shirtColors[1+math.random(shirtColors[1])])
        setColor('TROUSERS', trouserColors[1+math.random(trouserColors[1])])
        setColor('SHOE', shoeColors[1+math.random(shoeColors[1])])
    end

    -- Data for body movement:
    timings           = {0,1/21,2/21,3/21,4/21,5/21,6/21,7/21,8/21,9/21,10/21,11/21,12/21,13/21,14/21,15/21,16/21,17/21,18/21,19/21,20/21,21/21}
    legWaypoints      = {0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450,-0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,0.145,0.188,0.237}
    kneeWaypoints     = {0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243,0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092,0.282}
    ankleWaypoints    = {-0.133,0.041,0.244,0.382,0.304,0.232,0.266,0.061,-0.090,-0.145,-0.043,0.041,0.001,0.011,-0.099,-0.127,-0.121,-0.120,-0.107,-0.100,-0.090,-0.133}
    shoulderWaypoints = {0.028,0.043,0.064,0.078,0.091,0.102,0.170,0.245,0.317,0.337,0.402,0.375,0.331,0.262,0.188,0.102,0.094,0.086,0.080,0.051,0.058,0.028}
    elbowWaypoints    = {-1.148,-1.080,-1.047,-0.654,-0.517,-0.366,-0.242,-0.117,-0.078,-0.058,-0.031,-0.001,-0.009,0.008,-0.108,-0.131,-0.256,-0.547,-0.709,-0.813,-1.014,-1.148}
end
 
function sysCall_cleanup()
    -- setPausePointVisibility(1)   -- Disabled, pausePoints aren't used
    setColor('HAIR', HairColors[2])
    setColor('SKIN', skinColors[2])
    setColor('SHIRT', shirtColors[2])
    setColor('TROUSERS', trouserColors[2])
    setColor('SHOE', shoeColors[2])
end

function setColor(colorName,color)
    local objectTable = sim.getObjectsInTree(BillHandle,sim.sceneobject_shape)
    for i = 1, #objectTable, 1 do
        sim.setShapeColor(objectTable[i],colorName,0,color)
    end
end

function moveBody(dist)
    if dist then
        local t1 = math.mod(dist/1,1)
        local t2 = math.mod(t1+0.5,1)
      
        sim.setJointPosition(legJointHandles[1], sim.getPathInterpolatedConfig(legWaypoints,timings,t1)[1])
        sim.setJointPosition(kneeJointHandles[1], sim.getPathInterpolatedConfig(kneeWaypoints,timings,t1)[1])
        sim.setJointPosition(ankleJointHandles[1], sim.getPathInterpolatedConfig(ankleWaypoints,timings,t1)[1])
        sim.setJointPosition(shoulderJointHandles[1], sim.getPathInterpolatedConfig(shoulderWaypoints,timings,t1)[1])
        sim.setJointPosition(elbowJointHandles[1], sim.getPathInterpolatedConfig(elbowWaypoints,timings,t1)[1])
        
        sim.setJointPosition(legJointHandles[2], sim.getPathInterpolatedConfig(legWaypoints,timings,t2)[1])
        sim.setJointPosition(kneeJointHandles[2], sim.getPathInterpolatedConfig(kneeWaypoints,timings,t2)[1])
        sim.setJointPosition(ankleJointHandles[2], sim.getPathInterpolatedConfig(ankleWaypoints,timings,t2)[1])
        sim.setJointPosition(shoulderJointHandles[2], sim.getPathInterpolatedConfig(shoulderWaypoints,timings,t2)[1])
        sim.setJointPosition(elbowJointHandles[2], sim.getPathInterpolatedConfig(elbowWaypoints,timings,t2)[1])
    else
        for i=1,2,1 do
            sim.setJointPosition(legJointHandles[i],0)
            sim.setJointPosition(kneeJointHandles[i],0)
            sim.setJointPosition(ankleJointHandles[i],0)
            sim.setJointPosition(shoulderJointHandles[i],0)
            sim.setJointPosition(elbowJointHandles[i],0)
        end
    end
end

function setPausePointVisibility(l)
    for i = 1, #pausePoints, 1 do
        local pp = pausePoints[i]
        sim.setIntProperty(pp.handle, 'layer', l)
    end
end

function handlePossiblePause(op,dist)
    local st = sim.getSimulationTime()
    if op == 0 then
        for i = 1, #pausePoints, 1 do
            local p=sim.getObjectPosition(pausePoints[i].handle)
            local d=sim.getClosestPosOnPath(pathPositions,pathLengths,p)
            pausePoints[i].dist=d
            pausePoints[i].visited=false
        end
    else
        for i = 1, #pausePoints, 1 do
            local pp = pausePoints[i]
            if not pp.visited and (op&pp.dir~=0) then
                if (op == 1 and dist>pp.dist) or (op == 2 and dist<pp.dist) then
                    pp.visited = true
                    local p  = sim.getObjectPosition(pausePoints[i].handle)
                    local b  = sim.getObjectPosition(BillHandle)
                    local dx = p[1]-b[1]
                    local dy = p[2]-b[2]
                    sim.setObjectOrientation(BillHandle,{0,0,math.atan2(dy,dx)})
                    moveBody()
                    sim.wait(pp.duration)
                end
            end
        end
    end
    return sim.getSimulationTime()-st
end


function sysCall_thread()
    sim.setStepping(true)

    ------------------------------------------------------------
    -- MODE 0: Bill stays wherever sysCall_init() placed him
    ------------------------------------------------------------
    if BILL_MODE == 0 then
        while true do
            sim.step()  -- do nothing, stay alive until sim ends
        end
    end

    ------------------------------------------------------------
    -- Shared walking variables
    ------------------------------------------------------------
    local dist       = 0            -- current distance around circl, Bill's path
    local walkedDist = 0            -- total distance walked, need for leg animation
    local direction  = 1            -- 1 = forward, -1 = backward
    local speed      = BILL_SPEED

    local nextChaosChange = sim.getSimulationTime() + 3

    ------------------------------------------------------------
    -- MODE 1 / MODE 666: Walk forever around path
    ------------------------------------------------------------
    while true do
        local dt = sim.getSimulationTimeStep()

        --------------------------------------------------------
        -- MODE 666: mildly chaotic behavior
        --------------------------------------------------------
        if BILL_MODE == 666 then
            local now = sim.getSimulationTime()

            if now >= nextChaosChange then
                speed = 0.4 + math.random() * 0.6               -- rando speed, 0.4 - 1.0 m/s

                if math.random() < 0.20 then
                    direction = -direction                      -- reverse direction, rando!
                end

                nextChaosChange = now + 2 + math.random() * 4   -- change again anytime in 2-6 sec
            end
        end

        --------------------------------------------------------
        -- Move along path
        --------------------------------------------------------
        local stepDist = speed * dt

        dist       = dist + direction * stepDist
        walkedDist = walkedDist + stepDist

        --------------------------------------------------------
        -- Wrap around circular path forever
        --------------------------------------------------------
        if dist >= pathL then
            dist = dist - pathL
        elseif dist < 0 then
            dist = dist + pathL
        end

        --------------------------------------------------------
        -- Get Bill's position on path
        --------------------------------------------------------
        local p = sim.getPathInterpolatedConfig(pathPositions,
                                                pathLengths,
                                                dist
                                                )

        --------------------------------------------------------
        -- Turn Bill, look a little ahead to face Bill the right way, creepy otherwise
        --------------------------------------------------------
        local lookDist = dist + direction * 0.05

        if lookDist >= pathL then
            lookDist = lookDist - pathL
        elseif lookDist < 0 then
            lookDist = lookDist + pathL
        end

        local pAhead = sim.getPathInterpolatedConfig(pathPositions,
                                                     pathLengths,
                                                     lookDist
                                                     )

        local dx = pAhead[1] - p[1]
        local dy = pAhead[2] - p[2]

        --------------------------------------------------------
        -- Move Bill; Walk, Bill! Walk!
        --------------------------------------------------------
        sim.setObjectPosition(BillHandle,p,-1)
        sim.setObjectOrientation(BillHandle,{0,0,math.atan2(dy,dx)},-1)
        moveBody(walkedDist)  -- Animate arms/legs based on distance walked

        sim.step()
    end
end
