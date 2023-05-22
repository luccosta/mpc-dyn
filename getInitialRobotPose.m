%% Get initial pose = [position orientation]
function [robPosI,robOriI] = getInitialRobotPose(clientID, robotHandle, sim)
    e=1;
    while (e~=0)
        [e,robPosI]=sim.simxGetObjectPosition(clientID,robotHandle,-1,sim.simx_opmode_streaming);
    end
    e=1;
    while (e~=0)
        [e,robOriI]=sim.simxGetObjectOrientation(clientID,robotHandle,-1,sim.simx_opmode_streaming);
    end
end