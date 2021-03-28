clear all;
clc;
addpath('\Morden Robotics\MATLAB\mr')
%% Mile Stone 1 - youBot Kinematics Simulator
% initial state
%   [φ x y θ1 θ2 θ3  θ4  θ5 ω1 ω2 ω3 ω4 gripper]
% = [0 0 0 0  0 0.2 -1.6 0  0  0  0  0   0]  
q_initial = [0 0 0 0 0 0.2 -1.6 0 0 0 0 0 0]';
%      dot[θ1 θ2 θ3 θ4 θ5 ω1 ω2 ω3 ω4]
u =       [0 0 0 0 0 0 0 0 0]';
% delta_t time step
delta_t = 1;
% speed limit of the joint and the wheels
speed_limit = 15;
% calculate the H0 of the AGV
 l = 0.47/2;
 w = 0.3/2;
 r = 0.0475;
 H0 = (1/r)*[-l-w  1 -1; ...
              l+w  1  1; ...
              l+w  1 -1; ...
             -l-w  1  1];

NextState(q_initial, [0 0 0 0 0 10 10 10 10]', delta_t, speed_limit, H0);
NextState(q_initial, [0 0 0 0 0 -10 10 -10 10]', delta_t, speed_limit, H0);
NextState(q_initial, [0 0 0 0 0 -10 10 10 -10]', delta_t, speed_limit, H0);
%% Mile Stone 2 - Reference Trajectory Generation
% end effector initial configuration
Tse_initial = [0 0 1 0; ...
               0 1 0 0; ...
              -1 0 0 0.5; ...
               0 0 0 1];
% cube's initial configuration
theta_cube = 0;
w_cube = [0 0 1]';
p_cube = [1 0 0.025]';
Tsc_initial = RpToTrans(MatrixExp3(theta_cube*VecToso3(w_cube)),p_cube);
% cube's final configuration
theta_cube = -pi/2;
w_cube = [0 0 1]';
p_cube = [0 -1 0.025]';
Tsc_goal = RpToTrans(MatrixExp3(theta_cube*VecToso3(w_cube)),p_cube);
% The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp
Rce = MatrixExp3(((pi/2)+(pi/6))*VecToso3([0 1 0]'));
Tce_grasp = [Rce, [0.012 0 0.003]'; 0 0 0 1];
% The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: Tce,standoff
Tce_standoff = [Rce, [-0.06 0 0.1]'; 0 0 0 1];
%The number of trajectory reference configurations per 0.01 seconds: k
k = 1;
configlist = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k);
csvwrite('milestone2.csv', configlist);
%% Mile Stone 3 - Feedforward Control
Tse = [0.17 0 0.985 0.387; 0 1 0 0; -0.985 0 0.17 0.57; 0 0 0 1];
Tse_d = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Tse_dnext = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1];
Kp = 0;
Ki = 0;
delta_t = 0.01; % delta_t time step

[Ve, Vd, first_term, Xerr] = FeedbackControl(Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t);

q_start = [0 0 0 0 0 0.2 -1.6 0]';
[ud, Je] = commandSpeed(q_start, H0, Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t);
%% Capstone - control a AGV robot to grab a cubic from the starting location to goal location.
%set parameters
delta_t = 1; % delta_t time step
speed_limit = 30; % set speed limit to 30

Blist = [0 0 1 0 0.033 0; 0 -1 0 -0.5076 0 0; 0 -1 0 -0.3526 0 0; 0 -1 0 -0.2176 0 0; 0 0 1 0 0 0]'; % body twists of the arm
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % M matrix of the arm's zero configuration
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % Translation of 0 from b frame
Tse = Tse_initial; % end effector initial configuration

q = q_initial; % angle inital state
q_list = []; % list to store all the joints angles
Xerr_list = []; % list to store all the errors

Kp = eye(6); % preportional control gain
Ki = 0.2*eye(6); % intergrator control gain

% loop through the trajectory and use the control to follow the path 
disp("runscript");
for i = 2: length(configlist)-1
    % step1 find Tse_d and Tse_dnext
    statei = configlist(i,:);
    Tse_d = [statei(1:3) statei(10); statei(4:6) statei(11); statei(7:9) statei(12); 0 0 0 1];
    stateiplus1 = configlist(i+1,:);
    Tse_dnext = [stateiplus1(1:3) stateiplus1(10); stateiplus1(4:6) stateiplus1(11); stateiplus1(7:9) stateiplus1(12); 0 0 0 1];
    
    % step2 find speed of the robot u from control
    [ud, Xerr] = commandSpeed(q, H0, Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t);
    Xerr_list = [Xerr_list; Xerr'];
    u = [ud(5:9); ud(1:4)]; % mile-stone 1 has the oposite AGV speed and joint speed parameter layout of milestone3.  
    
    % step3 find next state q = u*△t
    q = NextState(q, u, delta_t, speed_limit, H0);
    q = setjointlimit(q);
    q_list = [q_list; [q(1:12)' statei(13)]];
    
    % step4 find new Tse
        % a. find the AGV frame Tsb(q)
    phy = q(1);
    x = q(2);
    y = q(3);
    Tsb = [cos(phy) -sin(phy) 0 x; sin(phy) cos(phy) 0 y; 0 0 1 0.0963; 0 0 0 1];
        % b. find the arm end effector frame T0e(q)
    T0e = FKinBody(M0e, Blist, q(4:8));
        % c. find the Tse
    Tse = Tsb*Tb0*T0e;
end
disp("Generating animation csv file.");
csvwrite('CoppeliaSim.csv', q_list);
%% functions
% function set the joint limit
function qd = setjointlimit(q)
    qd = q;
    % set joint 3 limits to -120° < θ < 0
    if qd(6) < -pi*2/3
        qd(6) = -pi*2/3;
    elseif qd(6) > 0
        qd(6) = 0;
    end
end
% function to find the 
function [ud, Je, Xerr] = commandSpeed(q_start, H0, Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t)
    % default parameters
    Blist = [0 0 1 0 0.033 0; 0 -1 0 -0.5076 0 0; 0 -1 0 -0.3526 0 0; 0 -1 0 -0.2176 0 0; 0 0 1 0 0 0]'; % body twists of the arm
    M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1]; % M matrix of the arm's zero configuration
    Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1]; % Translation of 0 from b frame
    
    % step 1 find Jacobians
        % Jacobian of the arm
    Je_arm = JacobianBody(Blist, q_start(4:8)); 
        % jacobian of the base(AGV) Jecar = [AdT0e-1Tb0-1]F6. where F6 = [0m; 0m; inv(H0); 0m]
    T0e = FKinBody(M0e, Blist, q_start(4:8));
    F = pinv(H0,1e-4);
    F6 = [zeros(2,4);F;zeros(1,4)];
    Je_car = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6;
    
        % Jacobian of the AGV+robotic arm
    Je = [Je_car Je_arm];
    
    % step 2 find speed
        % use the control to find the speed to chase the target frame
    [Ve, Xerr] = FeedbackControl(Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t);
    ud = pinv(Je,1e-4)*Ve; % Ve = Je*ud → ud = Je-1Ve. 1e-4 eliminates very small numbers at the singularity
                           % ud = dot[ω1 ω2 ω3 ω4 θ1 θ2 θ3 θ4 θ5]
end
% function to calculate the kinematic task-space feedforward plus feedback control law, written both as Equation (11.16) and (13.37) in the textbook
function [Ve, Vd, first_term, Xerr] = FeedbackControl(Tse, Tse_d, Tse_dnext, Kp, Ki, delta_t)
    X = Tse;
    Xd = Tse_d;
    Xd_next = Tse_dnext;
    % find Vd from [Vd] = (1/△t)log(inv(Xd)*Xd_next)
    Vd_se3mat = (1/delta_t)*MatrixLog6(TransInv(Xd)*Xd_next);
    Vd = se3ToVec(Vd_se3mat);
    % find the first term of the control : [AdX-1Xd]Vd(t)
    first_term = Adjoint(TransInv(X)*Xd)*Vd;
    % find Xerr from [Xerr] = log(inv(X)*Xd)
    Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd));
    % find Ve = [AdX-1Xd]Vd(t) + KpXerr(t) + Ki∫Xerr(t)dt
    Ve = first_term + Kp*Xerr + Ki*Xerr*delta_t;

end

% function generate the reference trajectory for the end-effector frame {e}
function configlist = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k)
    configlist = [];
    T_se = [];
    % time length of each segment
    T = k*[50 10 68 25 1000 10 68 10];
    % segment 1 - move the gripper from its initial configuration to a "standoff" configuration a few cm above the block
    for t = 0:T(1)
        % T(s) = T_initial*exp(log(T_initial-1T_goal)*s(t))
        T_se = Tse_initial*MatrixExp6(TimeScaling(T(1), t)*MatrixLog6(TransInv(Tse_initial)*Tsc_initial*Tce_standoff));
        configlist = [configlist; [T_se(1,1:3) T_se(2,1:3) T_se(3,1:3) T_se(1:3,4)' 0]];
    end
    % segment 2 - move the gripper down to the grasp position
    for t = 0:T(2)
        T_se = T_se*MatrixExp6(TimeScaling(T(2), t)*MatrixLog6(TransInv(T_se)*Tsc_initial*Tce_grasp));
        configlist = [configlist; [T_se(1,1:3) T_se(2,1:3) T_se(3,1:3) T_se(1:3,4)' 0]];
    end
    % segment 3 - Closing of the gripper
    for t = 0:T(3)
        configlist = [configlist; [T_se(1,1:3) T_se(2,1:3) T_se(3,1:3) T_se(1:3,4)' 1]];
    end
    % segment 4 - move the gripper back up to the "standoff" configuration
    for t = 0:T(4)
        T_se = T_se*MatrixExp6(TimeScaling(T(4), t)*MatrixLog6(TransInv(T_se)*Tsc_initial*Tce_standoff));
        configlist = [configlist; [T_se(1,1:3) T_se(2,1:3) T_se(3,1:3) T_se(1:3,4)' 1]];
    end
    % segment 5 - move the gripper to a "standoff" configuration above the final configuration
    for t = 0:T(5)
        T_se = T_se*MatrixExp6(TimeScaling(T(5), t)*MatrixLog6(TransInv(T_se)*Tsc_goal*Tce_standoff));
        configlist = [configlist; [T_se(1,1:3) T_se(2,1:3) T_se(3,1:3) T_se(1:3,4)' 1]];
    end
    % segment 6 - move the gripper to the final configuration of the object
    for t = 0:T(6)
        T_se = T_se*MatrixExp6(TimeScaling(T(6), t)*MatrixLog6(TransInv(T_se)*Tsc_goal*Tce_grasp));
        configlist = [configlist; [T_se(1,1:3) T_se(2,1:3) T_se(3,1:3) T_se(1:3,4)' 1]];
    end
    % segment 7 - Opening of the gripper
    for t = 0:T(7)
        configlist = [configlist; [T_se(1,1:3) T_se(2,1:3) T_se(3,1:3) T_se(1:3,4)' 0]];
    end
    % segment 8 - move the gripper back to the "standoff" configuration
    for t = 0:T(8)
        T_se = T_se*MatrixExp6(TimeScaling(T(8), t)*MatrixLog6(TransInv(T_se)*Tsc_goal*Tce_standoff));
        configlist = [configlist; [T_se(1,1:3) T_se(2,1:3) T_se(3,1:3) T_se(1:3,4)' 0]];
    end
end
% function generate the Fifth-order polynomial time scaling with total period T and current time t
function s = TimeScaling(T, t)
    a0 = 0; a1 = 0; a2 = 0; a3 = 10/T^3; a4 = -15/T^4; a5 = 6/T^5;
    s = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
%     a0 = 0; a1 = 0; a2 = 3/(T^2); a3 = -2/(T^3);
%     s = a0 + a1*t + a2*t^2 + a3*t^3;
end
% function that generate the next robot state from speed and δt
function qnext = NextState(q, u, delta_t, speed_limit, H0)
    % limit the speed
    for i = 1:length(u)
        if u(i) > speed_limit
            u(i) = speed_limit;
        elseif u(i) < -speed_limit
            u(i) = -speed_limit;
        end
    end
    qnext = q;
    % increment the angeles base on first-order Euler step
    %   *new arm joint angles = (old arm joint angles) + (joint speeds) * Δt
    %   *new wheel angles = (old wheel angles) + (wheel speeds) * Δt
    qnext(4:12) = q(4:12) + u*delta_t;
    %   *new chassis configuration is obtained from odometry, as described in Chapter 13.4
    qnext(1:3) = qnext(1:3) + pinv(H0,1e-4)*u(6:9)*delta_t;
end

