function initGlobalVariables()

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

    % Dynamic init values
    global initIaL initIaR 

    initIaR = 0;
    initIaL = 0;
end