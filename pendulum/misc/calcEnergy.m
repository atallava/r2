function [te,ke,pe] = calcEnergy(states,physicalParams)
    % kinetic energy
    vel = states(:,2)*physicalParams.l;
    ke = 0.5*physicalParams.m*vel.^2;
    
    % potential energy
    height = physicalParams.l*(1-cos(states(:,1)));
    pe = physicalParams.m*physicalParams.g*height;
    
    % total energy
    te = ke+pe;
end