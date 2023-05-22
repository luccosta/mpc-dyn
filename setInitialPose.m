function setInitialPose(clientID, robotHandle, sim, position, orientation)
    sim.simxSetObjectPosition(clientID, robotHandle, -1, position, sim.simx_opmode_blocking);
    sim.simxSetObjectOrientation(clientID, robotHandle, -1, orientation, sim.simx_opmode_blocking);
end