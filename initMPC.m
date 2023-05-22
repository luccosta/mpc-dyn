function [args_MPC, solver_MPC, N_MPC, ff_MPC, f_MPC, T_MPC] = initMPC()
    %% MPC

    addpath('C:/Users/savio/OneDrive/Documentos/MATLAB/Lib/casadi-3.6.2-windows64-matlab2018b')
    import casadi.*

    T_MPC = 0.2; % 0.2; % sampling time [s]
    N_MPC = 10; % prediction horizon
    rob_diam = 0.3; % Apenas para visualização

    % prevendo N * T segundos no futuro

    v_max = 0.6; v_min = -v_max; % restrições do controlador para a velocidade
    omega_max = pi/4; omega_min = -omega_max; % restrições do controlador para a valocidade angular

    xx = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
    states = [xx;y;theta]; n_states = length(states);

    % Model
    %v = SX.sym('v'); omega = SX.sym('omega');
    %controls = [v;omega]; n_controls = length(controls);
    %rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s

    v = SX.sym('v'); omega = SX.sym('omega');
    IaR = SX.sym('IaR'); IaL = SX.sym('IaL');
    pose = [xx; y];
    i = [IaR;IaL];
    x = [n;i;theta;pose];
    [t,out] = ode45(@dinModel,timeSpan,x);
    nDot = out(1);
    iaDot = out(2);
    thetaDot = out(3);
    xPosDot = out(4);
    yPosDot = out(5);
    rhs = [xPosDot;yPosDot;thetaDot];


    f_MPC = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
    U = SX.sym('U',n_controls,N_MPC); % Decision variables (controls)
    P = SX.sym('P',n_states + n_states); % estados iniciais e estados de referência
    % parameters (which include the initial and the reference state of the robot)

    X = SX.sym('X',n_states,(N_MPC+1)); % Estados previstos para cada ação de controle
    % A Matrix that represents the states over the optimization problem.

    % compute solution symbolically
    X(:,1) = P(1:3); % initial state
    for k = 1:N_MPC
        st = X(:,k);  con = U(:,k);
        f_value  = f_MPC(st,con);
        st_next  = st +(T_MPC*f_value);
        X(:,k+1) = st_next;
    end
    % this function to get the optimal trajectory knowing the optimal solution
    ff_MPC=Function('ff',{U,P},{X});

    obj = 0; % Objective function
    g = [];  % constraints vector

    % As matrizes Q e R funionam como ganhos
    Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
    Rr = zeros(2,2); Rr(1,1) = 0.5; Rr(2,2) = 0.05; % weighing matrices (controls)
    % compute objective
    for k=1:N_MPC
        st = X(:,k);  con = U(:,k);
        obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*Rr*con; % calculate obj
    end

    % compute constraints
    for k = 1:N_MPC+1   % box constraints due to the map margins
        g = [g ; X(1,k)];   %state x
        g = [g ; X(2,k)];   %state y
    end

    % make the decision variables one column vector
    OPT_variables = reshape(U,2*N_MPC,1);
    nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

    opts = struct;
    opts.ipopt.max_iter = 100;
    opts.ipopt.print_level =0;%0,3
    opts.print_time = 0;
    opts.ipopt.acceptable_tol =1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;

    solver_MPC = nlpsol('solver', 'ipopt', nlp_prob,opts);

    args_MPC = struct;
    % inequality constraints (state constraints)
    args_MPC.lbg = -2;  % lower bound of the states x and y
    args_MPC.ubg = 2;   % upper bound of the states x and y

    % input constraints
    args_MPC.lbx(1:2:2*N_MPC-1,1) = v_min; args_MPC.lbx(2:2:2*N_MPC,1)   = omega_min;
    args_MPC.ubx(1:2:2*N_MPC-1,1) = v_max; args_MPC.ubx(2:2:2*N_MPC,1)   = omega_max;

    %----------------------------------------------
    % ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP

end