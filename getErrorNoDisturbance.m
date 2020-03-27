function [yp, ym] = getErrorNoDisturbance(u,vl,v_ini,vm_ini)

dT=0.1;
% simulating yp at the next instance using Eulers method
% Plant parameter values: a=1, b=5
% plant model v_dot= -a*v+b*u+d
dv=(-1*v_ini+5*u)*dT;
yp=v_ini+dv;


% simulating ym at the next instance using Eulers method
% model parameter values: am=2
% reference model v_dot= -am*vm+am*(vl+k*del)
% reference signal r= vl+ k*del
% k is design constant, assumed to be 5

%dvm=(-2*vm_ini+2*(vl+5*del))*dT;
dvm=(-2*vm_ini+2*(vl))*dT;
ym=vm_ini+dvm;

% calculating error e
%e=yp-ym;
end