close all;
clc; 
clear all;
%% Defining variables

% Actuator
global Ra La Kb Kt N 

Ra  = 0.71;      %Ohms
La  = 0.66e-3;   %H
Kb  = 0.023;     %V/(rad-s)
Kt  = 0.029;     %N-m/A
N   = 38.3;

%Body
global R L mw mc m d Im Ic Iw I

R   = 0.0975;      %m
L   = 0.331/2;        % 0.38/2;   %m
mw  = 1.5;        %Kg
mc  = 16;        %Kg
m   = mc + 2*mw;%Kg
d   = 0.04451;     %m
Im  = 0.02;   %Kg-m^2
Ic  = 0.1307;        %Kg-m^2
Iw  = 0.04;    %Kg-m^2
I   = Ic + mc*d^2 + 2*mw*L^2+2*Im; %Kg-m^2

%Dynamic eq.
global M B

M =[Iw + R^2/(4*L^2)*(m*L^2+I), ...
    R^2/(4*L^2)*(m*L^2-I);...
    R^2/(4*L^2)*(m*L^2-I),...
    Iw + R^2/(4*L^2)*(m*L^2+I)];
B   = [1 0;0 1];    
%% Time span
tStart = 0;
tEnd = 20;
timeSpan = [tStart tEnd];

%% Initial conditions
initPhiR_Dot = 0;
initPhiL_Dot = 0;
n = [initPhiR_Dot; initPhiL_Dot];

initTheta = 0;
initPos = [-2;-2];

initIaR = 0;
initIaL = 0;
i = [initIaR;initIaL];

x=[n;i;initTheta;initPos];
% Solving ODE
%[t,out] = ode45(@dinModel,timeSpan,x); % Não é útil agora
%% Plotting Data
% subplot(2,1,1)
% plot(t,out(:,3)), grid;
% xlabel('$t\;[s]$',"Interpreter","latex");
% ylabel('$I_{a_R}\;[A]$',"Interpreter","latex");
% title("Armature current from right motor");
% 
% subplot(2,1,2);
% plot(t,out(:,4)), grid;
% xlabel('$t\;[s]$',"Interpreter","latex");
% ylabel('$I_{a_L}\;[A]$',"Interpreter","latex");
% title("Armature current from left motor");

% figure();
% subplot(2,1,1);
% plot(t,out(:,6),t,out(:,7),t,out(:,5),'LineWidth',2),grid
% legend('$x(t)$','$y(t)$','$\phi(t)$',"Interpreter","latex",...
%         "Location","southeast");
% xlabel('t [s]',"Interpreter","latex")
% title("Pose of the DDMR in the dynamic model");
% 
% subplot(2,1,2);
% plot(t,out(:,1),t,out(:,2),'LineWidth',2),grid
% legend('$\phi_R(t)$','$\phi_L(t)$',"Interpreter","latex",...
%         "Location","southeast");
% xlabel('t [s]',"Interpreter","latex")
% title("Angular velocities of the DDMR wheels in the dynamic model");
%% Exporting to Coppelia
% timeSim = t;
% timeSeries = timeseries([out(:,1)';out(:,2)';out(:,6)';out(:,7)';out(:,5)']',t);
%% Loading coppeliaSim remote API - client side
sim=remApi('remoteApi');
%% Closing any previously opened connections
sim.simxFinish(-1);
%% Connecting to remote coppeliaSim API server
connectionAddress='127.0.0.1';
connectionPort=19997;
waitUntilConnected=true;
doNotReconnectOnceDisconnected=true;
timeOutInMs=5000;
commThreadCycleInMs=5;
res=0;
while(res == 0)
    [clientID]=sim.simxStart(connectionAddress,connectionPort,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs);
    if(clientID > -1)   
        sim.simxSynchronous(clientID,true);
        fprintf('Starting simpleTest.m\n');
        res=1;
    else
        fprintf ('Waiting for coppeliaSim ...\n');
    end
end
%% Getting robot handle
e=1;
while (e~=0)
    [e,rob]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);
end
[rLJoint, leftMotor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
[rRJoint, rightMotor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);

goalE = 1;
while (goalE~=0)
    [goalE,goalHandle]=sim.simxGetObjectHandle(clientID,'Goal',sim.simx_opmode_blocking); % Goal handle
end

%% Defining coppeliaSim client side parameters
hd=50e-3; % Taxa de amostragem
np= tEnd / hd;
tf= tEnd;
tc=0;
td=0;
id=1;
%
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);
torqueRp=zeros(np,1);
torqueLp=zeros(np,1);
phiDotRp=zeros(np,1);
phiDotLp=zeros(np,1);
% timeResampled = resample(timeSeries,0:hd:tf);
%% Starting coppeliaSim simulation
[res]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);
%% Get initial pose = [position orientation]
e=1;
while (e~=0)
    [e,robPosI]=sim.simxGetObjectPosition(clientID,rob,-1,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    [e,robOriI]=sim.simxGetObjectOrientation(clientID,rob,-1,sim.simx_opmode_streaming);
end
%

%% Locating goal
goalPos = [1.0, 1.0, -pi/3];

goalOri = [0.0, 0.0, 0.0];

xs = [double(goalPos(1)); double(goalPos(2)); double(goalOri(3))];

%% MPC

addpath('C:/Users/savio/OneDrive/Documentos/MATLAB/Lib/casadi-3.6.2-windows64-matlab2018b')
import casadi.*

T = 0.2; % 0.2; % sampling time [s]
Nn = 10; % prediction horizon
rob_diam = 0.3; % Apenas para visualização

% prevendo N * T segundos no futuro

v_max = 0.6; v_min = -v_max; % restrições do controlador para a velocidade
omega_max = pi/4; omega_min = -omega_max; % restrições do controlador para a valocidade angular

xx = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [xx;y;theta]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega');
controls = [v;omega]; n_controls = length(controls);
rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,Nn); % Decision variables (controls)
P = SX.sym('P',n_states + n_states); % estados iniciais e estados de referência
% parameters (which include the initial and the reference state of the robot)

X = SX.sym('X',n_states,(Nn+1)); % Estados previstos para cada ação de controle
% A Matrix that represents the states over the optimization problem.

% compute solution symbolically
X(:,1) = P(1:3); % initial state
for k = 1:Nn
    st = X(:,k);  con = U(:,k);
    f_value  = f(st,con);
    st_next  = st +(T*f_value);
    X(:,k+1) = st_next;
end
% this function to get the optimal trajectory knowing the optimal solution
ff=Function('ff',{U,P},{X});

obj = 0; % Objective function
g = [];  % constraints vector

% As matrizes Q e R funionam como ganhos
Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
Rr = zeros(2,2); Rr(1,1) = 0.5; Rr(2,2) = 0.05; % weighing matrices (controls)
% compute objective
for k=1:Nn
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*Rr*con; % calculate obj
end

% compute constraints
for k = 1:Nn+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   %state x
    g = [g ; X(2,k)];   %state y
end

% make the decision variables one column vector
OPT_variables = reshape(U,2*Nn,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
% inequality constraints (state constraints)
args.lbg = -2;  % lower bound of the states x and y
args.ubg = 2;   % upper bound of the states x and y

% input constraints
args.lbx(1:2:2*Nn-1,1) = v_min; args.lbx(2:2:2*Nn,1)   = omega_min;
args.ubx(1:2:2*Nn-1,1) = v_max; args.ubx(2:2:2*Nn,1)   = omega_max;

%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP

% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------



% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

x0 = [-2.0; -2.0; 0.0];
xxx(:,1) = x0; % xx contains the history of states
xxxx(:,1) = x0;

t0 = 0;
ttt(1) = t0;
u0 = zeros(Nn,2);  % two control inputs 

controller_time = 0;
rightVel = 0;
leftVel = 0;

robOri_noTempo = [];
leftVel_noTempo = [];
rightVel_noTempo = [];

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
    
    %% Current sampling instant
    t(id)=td;
    %% Castle wheel debugging
%     if and(td<0.7,td>=0.4)
%         keyboard;
%     end
    %% Measuring
    [~,robPos]=sim.simxGetObjectPosition(clientID,rob,-1,sim.simx_opmode_buffer);
    [~,robOri]=sim.simxGetObjectOrientation(clientID,rob,-1,sim.simx_opmode_buffer);
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
%     leftVel= timeResampled.Data(id,2);
%     rightVel= timeResampled.Data(id,1);

controller_time = controller_time + hd;
if(controller_time >= T)
    controller_time = 0;

    x0 = [double(robPos(1)); double(robPos(2)); double(robOri(3))];
    
    args.p   = [x0;xs]; % set the values of the parameters vector
    args.x0 = reshape(u0',2*Nn,1); % initial value of the optimization variables   
    
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx',...
        args.ubx,'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',2,Nn)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    
    xx1(:,1:3,mpciter+1)= full(ff_value)';
    u_cl= [u_cl ; u(1,:)];
    ttt(mpciter+1) = t0;
    
    xxxx(:,mpciter+2) = x0;
    
    [t0, x0, u0] = shift(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xxx(:,mpciter+2) = x0;
    
    mpciter = mpciter + 1;
    
    % Velocidade de cada roda
    
    leftVel = (u(1,1) - (L)*u(1,2))/R;
    rightVel = (u(1,1) + (L)*u(1,2))/R;
    
    leftVel_noTempo(length(leftVel_noTempo) + 1) = leftVel;
    rightVel_noTempo(length(rightVel_noTempo) + 1) = rightVel;
    
%     A_temp = [1 1;
%               1 -1];
%     B_temp = [2 * u(1) ; u(2) * 2 * L];
%     
%     sol_temp = A_temp\B_temp; % A x = B
%     
%     if(abs(leftVel - sol_temp(2)) <= 0.001)
%         disp('igual')
%         disp(leftVel)
%         disp(sol_temp(2))
%     end
    
%     rightVel = sol_temp(1);
%     leftVel = sol_temp(2);
    
    %2 * u(1) = rightVel + leftVel
    %u(2) * 2 * L = rightVel - leftVel
end

    %% Actuating
    
    [~] = sim.simxSetJointTargetVelocity(clientID, leftMotor, leftVel , sim.simx_opmode_blocking); % * (180/pi)
    [~] = sim.simxSetJointTargetVelocity(clientID, rightMotor, rightVel , sim.simx_opmode_blocking);
    %simx_opmode_oneshot
    %% Saving
    %dotPhiR(id)= phiRCop(1,1);
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

%% Comparing both models

%% Opcional

figure(),
stairs(xxx(1,:), xxx(2,:)),
legend('(x,y)')

figure(),
stairs(leftVel_noTempo),
hold on,
stairs(rightVel_noTempo),
legend('left', 'right')

%%
figure(),
stairs(u_cl(:,1)),
hold on,
stairs(u_cl(:,2)),
legend('linear', 'angular')

%%

% leftVel_3 = [];
% rightVel_3 = [];
% for i=1:length(u_cl)
%    leftVel_3(i) = (u_cl(i,1) - (L)*u_cl(i,2))/R;
%    rightVel_3(i) = (u_cl(i,1) + (L)*u_cl(i,2))/R;
% end
% 
% figure(),
% stairs(leftVel_3),
% hold on,
% stairs(rightVel_3),
% legend('left', 'right')

%%
% leftVel_2 = (u_cl(:,1) - (L)*u_cl(:,2))/R;
% rightVel_2 = (u_cl(:,1) + (L)*u_cl(:,2))/R;
% 
% figure(),
% stairs(leftVel_2),
% hold on,
% stairs(rightVel_2),
% legend('left', 'right')


% figure();
% axis equal
% hold on %hold for drawRobot(.)
% h(1) = plot(timeResampled.Data(:,3),timeResampled.Data(:,4),'b:','LineWidth',2.0);
% xlabel('$x\;[m]$',"Interpreter","latex");
% ylabel('$y\;[m]$',"Interpreter","latex");
% for i=1:round(size(timeResampled.Data,1)/20):size(timeResampled.Data,1)
%     drawRobot(timeResampled.Data(i,3),timeResampled.Data(i,4),timeResampled.Data(i,5),0.02,'b-');
% end
% h(2) = plot(xp,yp,'r:','LineWidth',2.0);
% for i=1:round(length(xp)/20):length(xp)
%     drawRobot(xp(i),yp(i),fp(i),0.02,'r-');
% end
% legend(h([1 2]),"Dynamic Model","Simulation");
% grid on;
% hold off %release it

% figure();
% subplot(2,1,1);
% plot(t,torqueRp,timeSim,N*Kt*out(:,3),'LineWidth',2),grid
% legend("Dynamic Model","Simulation","Location","southeast");
% xlabel('t [s]',"Interpreter","latex");
% ylabel('$\tau_R\;[N\cdot m]$',"Interpreter","latex")
% title("Torque of the right wheel");
% 
% subplot(2,1,2);
% plot(t,torqueLp,timeSim,N*Kt*out(:,4),'LineWidth',2),grid
% legend("Dynamic Model","Simulation","Location","southeast");
% xlabel('t [s]',"Interpreter","latex")
% ylabel('$\tau_L\;[N\cdot m]$',"Interpreter","latex")
% title("Torque of the left wheel");
%% Functions
function v = Va(t)
    if t < 1
        v = [3;3];
    else
        v = [3;3];
    end
end

function xDot = dinModel(t,x)
    global R L mc d Kb Ra La N Kt M B
        
    tau = N*Kt*[x(3);x(4)];
    
    thetaDot = R/(2*L)*(x(1) - x(2)); 
    V = [0 R^2/(2*L)*mc*d*thetaDot; ...
         -R^2/(2*L)*mc*d*thetaDot 0];
    
    n = [x(1);x(2)];
    nDot = M\(B*tau - V*n);    
    
    ea = Kb*N*n;
    ia = [x(3);x(4)];     
    iaDot = (Va(t)-ea-Ra*ia)/La;
    
    xPosDot = R/2*(x(1) + x(2))*cos(x(5));
    yPosDot = R/2*(x(1) + x(2))*sin(x(5));   
    
    xDot = [nDot;iaDot;thetaDot;xPosDot;yPosDot];
end