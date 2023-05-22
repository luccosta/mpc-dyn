function [sim, clientID] = initCoppeliaSim()
    % Loading coppeliaSim remote API - client side
    sim = remApi('remoteApi');

    % Closing any previously opened connections
    sim.simxFinish(-1);

    % Connecting to remote coppeliaSim API server
    connectionAddress              = '127.0.0.1';
    connectionPort                 = 19997;
    waitUntilConnected             = true;
    doNotReconnectOnceDisconnected = true;
    timeOutInMs                    = 5000;
    commThreadCycleInMs            = 5;
    
    res=0;
    while(res == 0)
        [clientID]=sim.simxStart(connectionAddress,connectionPort,...
            waitUntilConnected,doNotReconnectOnceDisconnected,...
            timeOutInMs,commThreadCycleInMs);
        
        if(clientID > -1)   
            sim.simxSynchronous(clientID,true);
            fprintf('Starting simulation\n');
            res=1;
        else
            fprintf ('Waiting for CoppeliaSim ...\n');
        end
    end
end