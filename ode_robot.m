%% System Dynammics Surface vessel
function [dx,tau] = ode_robot(~, x, robot)
%other desired outputs ,psi,b,k
n=size(x)/2;%n=6 ideally
    q = x(1:n);%configuration
    dq = x(n+1:2*n);%velocity
    B = eye(n);%fully actuated
    M = robot.inertia(q);
    C = robot.coriolis(q, dq);
    G = robot.gravload(q)';  % Returns row vector

    tau = zeros(n,1);  % Passive system (no actuation, to be changed)

    ddq = M \ (tau - C*dq' - G');

    dx = [dq'; ddq];
end
function unusedCBF
nabla_b = -e; % nabla cbf
u_0=nabla_b/b;%standard input, which can be scaled by diagonal matrix K
if params.CBF.QP==false
k=params.k;
else
    u_ref=Ref_input_vessel(t,params)';
    %solve \min_u |u-u_ref|^2 s.t. \in U(y,t)
    %analytical solution: compute unconstrained solution, then project
    k_temp=u_ref'*u_0/(u_0'*u_0);
    %for u_0=0, this is not well defined, and we get k=inf; but u=0 anyway
    %since k_max*u_0=0, so still works.
    k=max(min(k_temp,params.CBF.m_max),params.CBF.k_min);
end
u =k* u_0; % control
end