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