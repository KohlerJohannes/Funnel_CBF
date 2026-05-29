%define setup (robot, initial configuration, desired configuration)
%define robot
robot = loadrobot('kinovaGen3', 'DataFormat','column');  % 6/7 DOF example
robot.Gravity = [0 0 -9.81];
% Initial state: [q; dq]
n = numel(robot.homeConfiguration);
%params.q_des=linspace(pi/4,pi/2,n)';
%set meaningful initial + desired configuration based on inverse kinematics
ee = robot.BodyNames{end};
% --- Final pose 
Tf = trvec2tform([0.45 0.10 0.35]) * ...
     axang2tform([0 1 0 pi/2]);  % desired end-effector pose
% --- Initial pose (new, visually distinct)
T0_pos = [0.35 -0.15 0.05];           % X,Y,Z in meters
T0_orient = axang2tform([0 1 0 -pi/4]);  % rotated around Y axis
T0 = trvec2tform(T0_pos) * T0_orient;
% --- Set up IK
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.3 0.3 0.3 1 1 1];
% --- Solve for final joint configuration
[qf, ~] = ik(ee, Tf, weights, robot.homeConfiguration);  
% --- Solve for initial joint configuration
q0 = ik(ee, T0, weights, qf);  % use qf as initial guess for stability
params.q_0=q0;
params.q_des=qf;
x0=[q0;zeros(n,1)];
save params
%% plot robot
%params.q_des=linspace(pi/4,pi/2,n)';
figure
hold on
show(robot,params.q_0,'PreservePlot',true);
show(robot,params.q_des,'PreservePlot',true);
axis equal
view(160,10)
camlight headlight
lighting gouraud
xlim([-0,0.1])
ylim([-0.1,0.1])
zlim([-0.1,0.7])
set(gcf,'Color','w')          % white background
axis equal
axis off
axis tight          % shrink axes limits to the robot
    
text(0.5, -0.2, 0.15, 'Initial configuration', 'FontSize', 16);
text(0.7,  0.1, 0.45, 'Final configuration', 'FontSize', 16);

set(gca,'FontSize',12)

exportgraphics(gcf,'kinova_gen3.png','Resolution',600)
I = imread('kinova_gen3.png');
%
 [m, n, ~] = size(I);        % m = rows, n = columns
I_cropped = I(:, round(n*1/3):end, :);  % keep right half
%
imwrite(I_cropped, 'kinova_gen3_trim.png');