function move(velocity_msg, velocity_pub, rho, theta)
% Theta becomes rad/s 
    
    spinVelocity = pi/4;       % Angular velocity (rad/s)
    forwardVelocity = .1;    % Linear velocity (m/s)
    backwardVelocity = -.1;    % Linear velocity (m/s)
    %spinTime = 0.5; %theta/spinVelocity;
    
    if abs(theta) > 0 
        spinTime = abs(theta)/spinVelocity;
        velocity_msg.Angular.Z = sign(theta)*spinVelocity;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
        pause(spinTime); 
        
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
    end 
    
    if rho > 0
 
        forwardTime = rho/forwardVelocity;
       
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = forwardVelocity;
        send(velocity_pub, velocity_msg);
        pause(forwardTime); 
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
        
    elseif rho < 0
        forwardTime = abs(rho)/abs(backwardVelocity);
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = backwardVelocity;
        send(velocity_pub, velocity_msg);
        pause(forwardTime); 
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
    else
        velocity_msg.Angular.Z = 0.0;
        velocity_msg.Linear.X = 0.0;
        send(velocity_pub, velocity_msg);
    end
end