function [x_next,phi_next] = robotkinematics(x_curr,phi_curr,RM_curr,deltaT,v_ctrl,w_ctrl)

x_next = x_curr + deltaT*RM_curr*v_ctrl;
phi_next = wrapToPi(phi_curr + deltaT*w_ctrl);

end

