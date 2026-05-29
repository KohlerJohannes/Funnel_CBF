clear all
close all
clc
%%
load Results/result_CBF.mat
%careful, both results are called the same
%plot q, dotq; torque
x_CBF=x;
tau_CBF=tau;
lambda_CBF=lambda;
sum(sum((tau_CBF(:,1:end-1)-tau_ref(:,1:end-1)).^2))/N%*(t(2:end)-t(1:end-1)))
%%
%load Results/result_funnel
%x_funnel=x;
%tau_funnel=tau;
%sum(sum((tau_funnel(:,1:end-1)-tau_ref(:,1:end-1)).^2))/N%*(t(2:end)-t(1:end-1)))

%% position
figure(1)
plot(t(1:N),x_CBF(1:n,1:N)','-')
%hold on
%plot(t(1:N),q_des(:,1:N)',':')
% plot(t, x(1:n,:)');
hold on
%plot(t(1:N),x_funnel(1:n,1:N)','--')
plot(t(1:end-1),q_des','--')
xlabel('Time [s]');
ylabel('Joint Angles [rad]');
%title('Joint Trajectories');
legend(arrayfun(@(i) sprintf('q%d', i), 1:n, 'UniformOutput', false));
set(gca, 'fontname','Arial','fontsize',16)
print('Configuration','-depsc')
%% velocity
figure(2);
%plot(t(1:N), (x(n+1:2*n,1:N)-dq_des(:,1:N))');
plot(t(1:N), x_CBF(n+1:2*n,1:N)')
hold on
%plot(t(1:N), x_funnel(n+1:2*n,1:N)','--')
plot(t(1:N),dq_des(:,1:N)','--');
xlabel('Time [s]');
ylabel('Joint velocities [rad/s]');
legend(arrayfun(@(i) sprintf('v%d', i), 1:n, 'UniformOutput', false));
set(gca, 'fontname','Arial','fontsize',16)
print('Velocity','-depsc')
%%
figure
hold on
plot(t(1:N),tau_CBF(:,:)','linewidth',1)
plot(t(1:N),tau_ref(:,:)',':','linewidth',0.5)
plot(t(1:N),tau_CBF(:,:)','linewidth',1)
%plot(t(1:N),tau_funnel','--','linewidth',1)
%plot(t,tau./lambda','--')
%plot(t,tau,'--')
xlabel('Time [s]');
ylabel('Torque [Nm]');
legend(arrayfun(@(i) sprintf('u%d', i), 1:n, 'UniformOutput', false));
set(gca, 'fontname','Arial','fontsize',16)
print('Torque','-depsc')
%% lambda
figure
plot(t(1:N),log(lambda_CBF)/log(10))
xlabel('Time [s]');
ylabel('log(\lambda)');
print('Lambda','-depsc')
%% funnel
figure
for k=1:N
   distance(k)=norm(x(1:n,k)-q_des(:,k),2);
end
plot(t(1:N),distance,'-')
hold on
plot([0,max(t)],params.psi(1)*[1,1],'--')