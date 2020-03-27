clc;
clear all;
close all;

% defining time interval
dT=0.1;
% simulating vl the velocity of lead vehicle
T = 0:dT:100;
 vl=[20*ones(500,1); 30*ones(501,1)];
% vl=sin(T)+cos(T)+2;
%vl=sin(T)+cos(T);
N = 1001;
%plot(T', vl, '--');

% The control input in this case is u=l*v-k*r
% r is reference signal, in this case equal to vl, the lead car velocity
% k=(am-a)/b (1/5), l=am/b (2/5)
% a, b are plant parameters (a=1, b=5), am is model parameter (am is 2)

% Taking V =e^2/2+|b|*k_tilde^2/(2*gamma1)+|b|*l_tilde^2/(2*gamma2)
% We get two adaptive laws k_dot=gamma1*e*v, l_dot=-gamma2*e*vl
% assuming b is positive
% We take gamma1, gamma2 as 2

gamma=2;
k=0.1;
l=0.3;
e_vals=zeros(N,2);
yp_vals=zeros(N,2);
ym_vals=zeros(N,2);
yp_vals(1,:)=[0 10]; % initial v
ym_vals(1,:)=[0 0]; % initial v
vm_ini=10; %initial vm of reference system
e_vals(1,:)=[0 10];

for n=1:N-1
      t=dT*n;
      yp=yp_vals(n,2);
      v_ini=yp;
      e=e_vals(n,2);
      
      % Using Euler's method to numerically solve k, l
      % from adaptive law eqns
       %dk=dT*(-gamma*e*yp);
       %k=k+dk;
       %dl=dT*(-gamma*e*vl(n));
       %l=l+dl;
       
       
       up=l*vl(n)-k*yp; %control input
       
       [yp, ym]=getErrorNoDisturbance(up,vl(n),v_ini,vm_ini);
       e=yp-ym;
       % storing computed values
       vm_ini= ym;
       yp_vals(n+1,:)= [t yp];
       ym_vals(n+1,:)= [t ym];
       e_vals(n+1,:)= [t e];
end

figure(1)
plot(yp_vals(:,1), yp_vals(:,2), 'r');
hold on
plot(T', vl, 'b')
title('Plot of vehicle speed and lead vehicle speed')
legend('Control vehicle', 'Lead Vehicle')
ylim([0, 40])
%xlim([0, 40])
figure(2)
%plot(e_vals(:,1), e_vals(:,2));
plot(T', yp_vals(:,2)-vl, 'b')
ylim([-50, 50])
xlim([0, 100])
title('Plot of error')
%plot(T', e_vals(:, 1));
%xlabel('Time')
%ylabel('Controller parameters')
%legend('v','vl','e')
%title('Plot of theta with r=10')
%hold off;