close all;
clc; 
clear all;

%% Defining variables
global Ra La Kb Kt N % Actuator
global R L mw mc m d Im Ic Iw I %Body
global M B %Dynamic eq.

initGlobalVariables();

%% Loading coppeliaSim remote API - client side
[sim, clientID] = initCoppeliaSim();

%% Getting robot handles
[robotHandle, rLJoint, leftMotor, rRJoint, rightMotor] = getRobotHandles(clientID, sim);

%% Time span
tStart = 0;
tEnd = 30;

%% Defining coppeliaSim client side parameters
hd=50e-3;      % Taxa de amostragem
np= tEnd / hd; % Número de pontos
tf= tEnd;      % Tempo final
tc=0;          % ?
td=0;          % Tempo de simulação corrente
id=1;          % Contador para os arrays

t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);
torqueRp=zeros(np,1);
torqueLp=zeros(np,1);
phiDotRp=zeros(np,1);
phiDotLp=zeros(np,1);

%% Starting coppeliaSim simulation
[res]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);

%% Get initial pose = [position, orientation]
[robPosI, robOriI] = getInitialRobotPose(clientID, robotHandle, sim);

%% Setting initial and final pose
robotInitialPose = [-2.0, -2.0, 0.0];
robotFinalPose = [2.0, 1.7, pi];

setInitialPose(clientID, robotHandle, sim,...
    [robotInitialPose(1), robotInitialPose(2), robPosI(3)],...
    [robOriI(1), robOriI(2), robotInitialPose(3)]);

x0 = robotInitialPose';
xs = robotFinalPose';

%% MPC

[args_MPC, solver_MPC, N_MPC, ff_MPC, f_MPC, T_MPC] = initMPC();
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP

% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

xxx(:,1) = x0; % xx contains the history of states

t0 = 0;
ttt(1) = t0;
u0 = zeros(N_MPC,2);  % two control inputs 

controller_time = 0;
rightVel = 0;
leftVel = 0;

robOri_noTempo = [];
leftVel_noTempo = [];
rightVel_noTempo = [];

%% Para medição do tempo

controllerTime_noTempo = [];
solverTime_noTempo = [];

%% Main control loop - coppeliaSim client side
while (td<tf)
    disp('Faltam: ')
    disp(tf - td)
    disp(' segundos\n')
    
    sim.simxSynchronousTrigger(clientID);
    sim.simxGetPingTime(clientID);
    
    %% Current sampling instant
    td=td+hd;
    id=id+1;
    t(id)=td;
    
    %% Measuring
    [~,robPos]=sim.simxGetObjectPosition(clientID,robotHandle,-1,sim.simx_opmode_buffer);
    [~,robOri]=sim.simxGetObjectOrientation(clientID,robotHandle,-1,sim.simx_opmode_buffer);
    robOri_noTempo(length(robOri_noTempo) + 1) = robOri(3);
    if (td>0)
        [~,robPhiDotR]=sim.simxGetObjectFloatParameter(clientID,rightMotor,2012,sim.simx_opmode_buffer);
        [~,robPhiDotL]=sim.simxGetObjectFloatParameter(clientID,leftMotor,2012,sim.simx_opmode_buffer);
        [~,robTorqueR]=sim.simxGetJointForce(clientID,rightMotor,sim.simx_opmode_buffer);
        [~,robTorqueL]=sim.simxGetJointForce(clientID,leftMotor,sim.simx_opmode_buffer);
    else
        [~,robPhiDotR]=sim.simxGetObjectFloatParameter(clientID,rightMotor,2012,sim.simx_opmode_streaming);
        [~,robPhiDotL]=sim.simxGetObjectFloatParameter(clientID,leftMotor,2012,sim.simx_opmode_streaming);
        [~,robTorqueR]=sim.simxGetJointForce(clientID,rightMotor,sim.simx_opmode_streaming);
        [~,robTorqueL]=sim.simxGetJointForce(clientID,leftMotor,sim.simx_opmode_streaming);
    end
    %% Controlling
    controller_time = controller_time + hd;
    if(controller_time >= T_MPC)
        controller_tic = tic();
        controller_time = 0;

        x0 = [double(robPos(1)); double(robPos(2)); double(robOri(3))]; % Estado atualizado do robô

        args_MPC.p   = [x0;xs]; % set the values of the parameters vector
        args_MPC.x0 = reshape(u0',2*N_MPC,1); % initial value of the optimization variables   

        % Solver
        solver_tic = tic();
        sol = solver_MPC('x0', args_MPC.x0, 'lbx', args_MPC.lbx, 'ubx',...
            args_MPC.ubx,'lbg', args_MPC.lbg, 'ubg', args_MPC.ubg,'p',args_MPC.p);    
        solver_toc = toc(solver_tic);
        
        u = reshape(full(sol.x)',2,N_MPC)';
        ff_value = ff_MPC(u',args_MPC.p); % compute OPTIMAL solution TRAJECTORY

        xx1(:,1:3,mpciter+1)= full(ff_value)'; % ?
        
        u_cl= [u_cl ; u(1,:)]; % Salvando comandos de controle
        
        ttt(mpciter+1) = t0;

        [t0, x0, u0] = shift(T_MPC, t0, x0, u,f_MPC); % get the initialization of the next optimization step

        xxx(:,mpciter+2) = x0; % Salvando todos os estados

        mpciter = mpciter + 1;

        % Velocidade de cada roda
        leftVel = (u(1,1) - (L)*u(1,2))/R;
        rightVel = (u(1,1) + (L)*u(1,2))/R;

        % Salvando velocidade de roda
        leftVel_noTempo(length(leftVel_noTempo) + 1) = leftVel;    
        rightVel_noTempo(length(rightVel_noTempo) + 1) = rightVel; 
        
        controller_toc = toc(controller_tic);
        
        % Salvando tempos de execução
        controllerTime_noTempo(length(controllerTime_noTempo) + 1) = controller_toc;    
        solverTime_noTempo(length(solverTime_noTempo) + 1) = solver_toc;  
    end

    %% Actuating
    setActuatorVelocity(clientID, sim, leftMotor, leftVel, rightMotor, rightVel);
    
    %% Saving
    xp(id)=robPos(1,1);
    yp(id)=robPos(1,2);
    fp(id)=robOri(1,3);
    torqueRp(id)=robTorqueR;
    torqueLp(id)=robTorqueL;  
    phiDotRp(id)=robPhiDotR(1);
    phiDotLp(id)=robPhiDotL(1);    
end

%% Stoping coppeliaSim simulation
sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot_wait);
fprintf('Ending simpleTest.m\n');
sim.simxFinish(clientID);
sim.delete();

%% Plotting results
figure();
subplot(2,1,1);
plot(t,xp,t,yp,t,fp,'LineWidth',2),grid
legend('$x(t)$','$y(t)$','$\theta(t)$',"Interpreter","latex",...
        "Location","southeast");
xlabel('t [s]',"Interpreter","latex")
title("Pose of the DDMR in simulation");

subplot(2,1,2);
plot(t,phiDotRp,t,phiDotLp,'LineWidth',2),grid
legend('$\phi_R(t)$','$\phi_L(t)$',"Interpreter","latex",...
        "Location","southeast");
xlabel('t [s]',"Interpreter","latex")
title("Angular velocities of the DDMR wheels in simulation");

%% Gráficos
figure(),
stairs(xxx(1,:), xxx(2,:)),
legend('(x,y)'),
title('Posição')

%% Velocidades por roda
figure(),
stairs(leftVel_noTempo),
hold on,
stairs(rightVel_noTempo),
legend('left', 'right'),
title('Velocidades de cada roda')

%% Velocidades linear/angular
figure(),
stairs(u_cl(:,1)),
hold on,
stairs(u_cl(:,2)),
legend('linear', 'angular'),
title('Velocidades linear e angular')

%% Tempo de execução
fig_aux_x = 1:1:length(controllerTime_noTempo); 

figure(),
stairs(controllerTime_noTempo),
hold on,
plot(fig_aux_x, mean(controllerTime_noTempo) * ones(1, length(controllerTime_noTempo))  ),
stairs(solverTime_noTempo),
plot(fig_aux_x, mean(solverTime_noTempo) * ones(1, length(solverTime_noTempo))  ),
legend('Tempo do controle', ...
    strcat('Tempo médio controle:', num2str( mean(controllerTime_noTempo),'%0.5f')),...
    'Tempo do solver', ...
    strcat('Tempo médio solver:', num2str( mean(solverTime_noTempo),'%0.5f')))
title('Tempos de execução')





