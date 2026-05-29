clear all;
close all;
clc;
%% robot
robot = loadrobot('kinovaGen3', 'DataFormat','column');  % 6/7 DOF example
robot.Gravity = [0 0 -9.81];
% Initial state: [q; dq]
n = numel(robot.homeConfiguration);
q0 = zeros(n,1);
dq0 = zeros(n,1);
x0 = [q0; dq0];
%% funnel
tspan = [0 2];
params.psi=1e-2*[1;1e4];%constant funnel; position; velocity
params.barrier_min=1e-10;
params.q_0=q0;
params.q_des=pi/4*ones(n,1);
params.t_interpol=5e-1;
params.lambda_min=1e-3;
params.lambda_max=1e3;
%params.torque_max=44;%Nm
params.funnel_option=1;%0:PD; 1:funnel
params.CBF=true;
%params.funnel_gain=40;%constant k>2 in paper

params.omega_ref=200;%frequency sinusoid
params.tau_ref=3;%amplitude

% PD gains (you can tune these)
 params.Kp = 100 * eye(n);       % Proportional gain
 params.Kd = 1 * eye(n);      % Derivative gain

 x0=[rand(n,1);zeros(n,1)];
 %x0=x0*params.psi(1)/norm(x0)*0.8;
 %scale based on funnel:
 %take desired funnel c
x0= 0.5*x0/norm(x0)*params.psi(1);
%(params.psi(1)+sqrt(1-params.psi(1)^2))/(2*params.psi(1))
%temp=norm(x0(1:n),2)
%x0=temp/(1+temp^2)/2;

%% simulate funnel-QP
options = odeset('RelTol',1e-7,'AbsTol',1e-7); % As Option: resolution of the simulator
sol = ode15s(@(t,x) robotDynamics(t, x, robot,params), tspan, x0,options);
t=sol.x;
x=sol.y;
t=double(t).';
x=double(x).';
% funnel-QP simulation
[~,tau,q_des,dq_des,psi,barrier,lambda,tau_ref] = cellfun(@(t,x) robotDynamics(t,x.',robot,params), num2cell(t), num2cell(x,2),'uni',0);
tau=cell2mat(tau);
q_des=cell2mat(q_des);
dq_des=cell2mat(dq_des);
tau = reshape(tau,[n,numel(t)]);
q_des = reshape(q_des,[n,numel(t)]);
dq_des = reshape(dq_des,[n,numel(t)]);
barrier=cell2mat(barrier);
lambda=cell2mat(lambda);
tau_ref=cell2mat(tau_ref);
tau_ref = reshape(tau_ref,[n,numel(t)]);
%%
sum((tau(:,1:end-1)-tau_ref(:,1:end-1)).^2*(t(2:end)-t(1:end-1)))
%% plot
figure;
plot(t, x(:,1:n)');
%plot(t, x(:,1:n)-q_des');
hold on
plot(t,q_des','b--')
xlabel('Time [s]');
ylabel('Joint Angles [rad]');
title('Joint Trajectories');
legend(arrayfun(@(i) sprintf('q%d', i), 1:n, 'UniformOutput', false));
%%
figure;
plot(t, x(:,n+1:2*n)-dq_des');
xlabel('Time [s]');
ylabel('Joint velocities [rad/s]');
legend(arrayfun(@(i) sprintf('q%d', i), 1:n, 'UniformOutput', false));
%%
figure
plot(t,tau,'linewidth',1)
hold on
%plot(t,tau./lambda','--')
plot(t,tau_ref',':','linewidth',1)
%plot(t,tau,'--')
xlabel('Time [s]');
ylabel('Torque [Nm]');
%%
figure
plot(t,log(lambda)/log(10))
%%
figure
for k=1:length(t)
   distance(k)=norm(x(k,1:n)'-q_des(:,k),2);
end
plot(t,distance,'-')
hold on
plot([0,max(t)],params.psi*[1,1],'--')
%%
figure
plot(t,barrier,'-')
%legend('barrier','distance')
%%

function [dx,tau,q_des,dq_des,psi,barrier,lambda,tau_ref] = robotDynamics(t, x, robot,params)
    n = numel(robot.homeConfiguration);  % Number of DOFs
    q = x(1:n);
    dq = x(n+1:end);
    t
    % Compute dynamics terms
    M = massMatrix(robot, q);
    C = velocityProduct(robot, q, dq);   % C(q,dq)*dq
    G = gravityTorque(robot, q);
    [q_des,dq_des,psi] = reference_funnel(t,robot,params);
    if params.funnel_option==0
    % Compute control input (torque) with PD
    tau = PD_controller(t, q, dq,q_des,dq_des,robot,params);   
    lambda=1;
    
    elseif params.funnel_option==1
    % Compute control input (torque) with funnel
    [tau,barrier,lambda,tau_ref] = funnel(t, q, dq,q_des,dq_des,psi,robot,params);   
    else
        error('feedback not defined')
    end
    ddq = M \ (tau - C - G);

    dx = [dq; ddq];
end
function [q_des,dq_des,psi] = reference_funnel(t,robot,params)
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
% q_des = pi/40 * ones(n,1);   % e.g., move all joints to 45 degrees
% dq_des=zeros(n,1);
end
function tau = PD_controller(t, q, dq,q_des,dq_des,robot,params)
    % tau_func: Computes joint torques using a PD controller
    % Compute control torques
    tau = params.Kp * (q_des - q) +params.Kd * (dq_des - dq);
end
function [tau,barrier,lambda,tau_ref] = funnel(t, q, dq,q_des,dq_des,psi,robot,params)
r=2;%relative degree (currently hard coded)
%recursive definition
beta_1=(q-q_des)/psi(1);
beta_2=(dq-dq_des)/psi(2)+beta_1/(1-norm(beta_1,2)^2);
barrier=0.5*(1-norm(beta_2,2)^2);%this barrier =0 <-> crash
%barrier
%-beta_2/b
%b=max(barrier,params.barrier_min)
b=barrier
if norm(beta_1,2)>1 || b<0
 error('left funnel')
end
tau_0=-beta_2/b;
tau_ref=gravityTorque(robot, q)+ones(length(q),1)*sin(params.omega_ref*t)*params.tau_ref;
%;
if params.CBF==true
%optimize over lambda    
    lambda_temp=tau_ref'*tau_0/(tau_0'*tau_0);
    lambda=max(min(lambda_temp,params.lambda_max),params.lambda_min);
else 
    lambda=1;
end
tau=lambda*tau_0;
%tau=tau*min(1,params.torque_max/norm(tau,inf));
%min(1,params.torque_max/norm(tau,inf))
end
 
