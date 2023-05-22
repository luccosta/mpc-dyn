function [robotHandle, rLJoint, leftMotor, rRJoint, rightMotor] = getRobotHandles(clientID, sim)
    e=1;
    while (e~=0)
        [e,robotHandle] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);
    end
    [rLJoint, leftMotor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
    [rRJoint, rightMotor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
end