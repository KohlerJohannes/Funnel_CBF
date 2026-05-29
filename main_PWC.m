
clear all;
close all;
clc;
%
%% funnel
load params %robot params set in setup.m
params.CBF=true; %toggle if CBF or pure funnel is run
Tsim = 5;
params.psi=1e-2*[1;1e2];%constant funnel; position; velocity
params.barrier_min=1e-10;

params.t_interpol=4;
params.lambda_min=1e-3;
params.lambda_max=1e3;
%params.torque_max=44;%Nm
params.omega_ref=200/5;%frequency sinusoid
params.tau_ref=3;%amplitude
%initial condition in funnel, but perturbed from refrence
x0=[rand(n,1);zeros(n,1)];
x0= 0.5*x0/norm(x0)*params.psi(1);
x0=x0+1e-6*rand(2*n,1)+[params.q0;zeros(n,1)];
%% simulate funnel-QP
Ts=5*1e-3;
N=Tsim/Ts;
%pre-allocate
lambda=NaN(N,1);t=NaN(N,1);b=NaN(N,1);q_des=NaN(n,N);dq_des=NaN(n,N);tau=NaN(n,N);tau_ref=NaN(n,N);psi=NaN(2,N);
t(1)=0;
x(:,1)=x0;
options = odeset('RelTol',1e-6,'AbsTol',1e-6); % As Option: resolution of the simulator
for k=1:N
k/N %show progress
%compute lambda
if params.CBF
[lambda(k)] = CBF(t(k), x(:,k),robot,params);   
else
lambda(k)=1;%standard funnel controller
end
%simulate cont. time (with constant lambda)
tspan=[t(k),t(k)+Ts];
sol = ode15s(@(t,x) robotDynamics(t, x, lambda(k),robot,params), tspan, x(:,k),options);
[q_des(:,k),dq_des(:,k),psi(:,k),tau_ref(:,k)] = reference(t(k),robot,params);
[tau(:,k),b(k)] = funnel(t(k), x(:,k),lambda(k),robot,params);    
t(k+1)=sol.x(end);
x(:,k+1)=sol.y(:,end);
end
%%
if params.CBF
save('Results/result_CBF')
else
save('Results/result_funnel')
end
%%

function [dx,tau] = robotDynamics(t, x,lambda, robot,params)
    n = numel(robot.homeConfiguration);  % Number of DOFs
    q = x(1:n);
    dq = x(n+1:end);
    t;
    % Compute dynamics terms
    M = massMatrix(robot, q);
    C = velocityProduct(robot, q, dq);   % C(q,dq)*dq
    G = gravityTorque(robot, q);
    %compute torque from funnel
     [tau,b] = funnel(t, x,lambda,robot,params);    
     ddq = M \ (tau - C - G);
    dx = [dq; ddq];
end
function [q_des,dq_des,psi,tau_ref] = reference(t,robot,params)
%linear interpolation from start to end in time
 n = numel(robot.homeConfiguration);  % Number of DOFs
 psi=params.psi; %constant funnel for now
%linear interpolation reference --> doens't work, velocity jumps!
%     q_des = params.q_0 + t / params.t_interpol *(params.q_des-params.q_0);
%     dq_des= (params.q_des-params.q_0)/ params.t_interpol;
%quadratic interpolation/cubic spline
s=min(t / params.t_interpol,1);
q_des = params.q_0 + (3*s^2-2*s^3) *(params.q_des-params.q_0);
dq_des = (6*s-6*s^2)/params.t_interpol *(params.q_des-params.q_0);
tau_ref=gravityTorque(robot, q_des)+ones(n,1)*sin(params.omega_ref*t)*params.tau_ref;
end
function [tau,b] = funnel(t, x,lambda,robot,params)
[beta_1,beta_2,b]=funnel_var(t, x,robot,params);
tau_0=-beta_2/b;
tau=lambda*tau_0;
end
function [beta_1,beta_2,b]=funnel_var(t, x,robot,params)
%compute aux variables funnel
n=length(x)/2;
q=x(1:n);dq=x(n+1:2*n);
[q_des,dq_des,psi,tau_ref] = reference(t,robot,params);
beta_1=(q-q_des)/psi(1);
beta_2=(dq-dq_des)/psi(2)+beta_1/(1-norm(beta_1,2)^2);
b=0.5*(1-norm(beta_2,2)^2);%this barrier =0 <-> crash
if  b<0
 b=max(b,1e-5);
 error('warning: outside of funnel...')
 %error('left funnel!')
end
end
function lambda=CBF(t, x,robot,params)
%relative degree r=2 currently hard coded
[beta_1,beta_2,b]=funnel_var(t, x,robot,params);
[q_des,dq_des,psi,tau_ref] = reference(t,robot,params);
tau_0=-beta_2/b;
%optimize over lambda    
lambda_temp=tau_ref'*tau_0/(tau_0'*tau_0);
lambda=max(min(lambda_temp,params.lambda_max),params.lambda_min);
end
 
