clear all
close all
clc

%% Assigned Parameters
x_disturbance = 0.5;    % Disturbance applined on x-axis
y_disturbance = 0.5;    %
yaw_disturbance = 0.2;

m = 1.5;                                 % Initial Supposed Mass
m = 1.25;                               % New Mass Computed to minimize the error on the z-axis

Ib = diag([1.2416 1.2416 2*1.2416]);     % Inertia Matrix referred to the body frame 

% Flight values load from the .mat file
load("ws_homework_3_2024.mat");

time = attitude.time;
eta_b = attitude.signals.values;
p_b_dot = linear_vel.signals.values;
eta_b_dot = attitude_vel.signals.values;
u_T = thrust.signals.values;
tau = tau.signals.values;

Tc = 0.001;                              % Estimator Sampling Time
g = 9.81;                                % Gravity



%% Butterworth Polynomial
r = 1;                                  % Estimation Filter Order

[num,den] = butter(r,30,'low','s');     % The butter function generates a transfer function G(s) 
                                        % taking as input the order of the filter and the cutoff 
                                        % frequency (set equal to one after sperimental results)
                                        % cutoff frequency set to 30
                                        % (arbitrary in simulation mode) to
                                        % avoid an high switching frequency

G = tf(num,den)
G = zpk(G);                              % Filter Transfer Function

c = den(2:end)                         % Coefficients c_j of the polynomial
K = zeros(r,1);                         % Gain Matrix
var = 1;

%Inverse relationship --> Prod(K)= c_j --> K= DIV(c_j)
for i = 1:r
    K(i) = c(i)/var;
    var = var*K(i);
end

K = flip(K)                        % Gains need to be flipped (to order them from the lower to higher order)

%% Wrench Estimator

q = zeros(6,length(time));             % Momentum
wrench_e = zeros(6,length(time));      % [f_e tau_e]'
gamma = zeros(6,length(time),r);       % gamma initialization
e_3 = [0 0 1]';                        % to extract the f_z from u_T

for t = 1:length(time)-1
    % compute matrix 
    Q = compute_Q(eta_b(t,:));                            % Transformation Matrix 
    Q_dot = compute_Q_dot(eta_b(t,:),eta_b_dot(t,:));     % Derivative of Transformation Matrix
    C = compute_C(eta_b_dot(t,:),Ib,Q,Q_dot);             
    M = compute_M(compute_Q(eta_b(t+1,:)),Ib);            % Mass Matrix
    Rb = compute_Rb(eta_b(t,:));                           % Attitude
    
    q(:,t+1) = [m*eye(3) zeros(3,3); zeros(3,3) M]*[p_b_dot(t+1,:)'; eta_b_dot(t+1,:)'];   % Generalised momentum vector 𝑞
    
    % gamma_1 Computation
    additionalTerm = [m*g*e_3-u_T(t)* Rb *e_3; C' * eta_b_dot(t,:)' + Q' * tau(t,:)'];
    gamma(:,t+1,1) = gamma(:,t,1)+K(1) * ((q(:,t+1)-q(:,t))- Tc * additionalTerm -Tc * wrench_e(:,t));
    
    % higher order Gamma Computation (gamma_i) 
    if r >= 2             
        for i = 2:r
            gamma(:,t+1,i) = gamma(:,t,i) + Tc * K(i) * (-wrench_e(:,t)+gamma(:,t,i-1));
        end
    end
    wrench_e(:,t+1) = gamma(:,t+1,r);   % wrench update
end

%% Real Mass Computation

% Real mass computed from flight values (from the definition of flat
% output)
u_T_end = (u_T(end) * compute_Rb(eta_b(end,:))*e_3);
mass_1 = u_T_end(3)/g

% Estimated Mass computed from the estimated wrench
mass_2 = wrench_e(3,end)/g + m


%% Plots Saved in PDF with Butterworth filter

%Plots of Estimated force f_e = [fe_x, fe_y, fe_z]
double_plot_pdf(time,wrench_e(1,:),x_disturbance.*ones(length(time),1),'Time[s]','Force [N]','$$f_{ex}$$','$$dist_{x}$$','fx.pdf'); %Comparison with the disturbance
double_plot_pdf(time,wrench_e(2,:),y_disturbance.*ones(length(time),1),'Time[s]','Force [N]','$$f_{ey}$$','$$dist_{y}$$','fy.pdf'); %Comparison with the disturbance
single_plot_pdf(time,wrench_e(3,:),'Time[s]','$$f_{ez} [N]$$','fz.pdf');    %Useful to check if the Mass is correct 

%Plots of Estimated torque tau_e = [taue_x, taue_y, taue_z]
single_plot_pdf(time,wrench_e(4,:),'Time[s]','$$\tau_{ex} [Nm]$$','tx.pdf');
single_plot_pdf(time,wrench_e(5,:),'Time[s]','$$\tau_{ey} [Nm]$$','ty.pdf');
double_plot_pdf(time,wrench_e(6,:),yaw_disturbance.*ones(length(time),1),'Time[s]','Torque [Nm]','$$\tau_{ez}$$','$$dist_{yaw}$$','tz.pdf'); %Comparison with the disturbance

%Plots of the total Estimated wrench = [f_e , tau_e]
single_plot_pdf(time,wrench_e,'time[sec]','external wrench estimated','wrench_e.pdf');
legend('$$f_{ex}$$','$$f_{ey}$$','$$f_{ez}$$','$$\tau_{ex}$$','$$\tau_{ey}$$','$$\tau_{ez}$$')

%Plots of Errors Convergence
single_plot_pdf(time,(abs(wrench_e(1,:)-x_disturbance)),'Time[s]','$$f_{ex}\ error [N]$$','err_x_dist.pdf');
single_plot_pdf(time,(abs(wrench_e(2,:)-y_disturbance)),'Time[s]','$$f_{ey}\ error [N]$$','err_y_dist.pdf');
single_plot_pdf(time,(abs(wrench_e(6,:)-yaw_disturbance)),'Time[s]','$$\tau_{ez}\ yaw\ error [Nm]$$','err_yaw_dist.pdf');


%% MATLAB (not saved) plots of Butterworth Filter
figure()
subplot(3,2,1)
plot(time,wrench_e(1,:),'lineWidth',1)
xlabel('time[sec]')
ylabel('force [N]')
grid on
hold on
plot(time,x_disturbance.*ones(length(time),1),'LineStyle','--','LineWidth',1)
legend('f_x','x_disturbance' )

subplot(3,2,3)
plot(time,wrench_e(2,:),'lineWidth',1)
xlabel('time[sec]')
ylabel('force [N]')
grid on
hold on
plot(time,y_disturbance.*ones(length(time),1),'LineStyle','--','LineWidth',1)
legend('f_y','y_disturbance')

subplot(3,2,5)
plot(time,wrench_e(3,:),'lineWidth',1)
xlabel('Time[s]')
ylabel('f_z [N]')
grid on

subplot(3,2,2)
plot(time,wrench_e(4,:),'lineWidth',1)
xlabel('Time[s]')
ylabel('tau_x [Nm]')
grid on

subplot(3,2,4)
plot(time,wrench_e(5,:),'lineWidth',1)
xlabel('Time[s]')
ylabel('tau_y [Nm]')
grid on

subplot(3,2,6)
plot(time,wrench_e(6,:),'lineWidth',1)
xlabel('Time[s]')
ylabel('tau_z [Nm]')
grid on
hold on
plot(time,yaw_disturbance.*ones(length(time),1),'LineStyle','--','LineWidth',1)
legend('tau_z','yaw_disturbance')

figure()
plot(time,wrench_e,'lineWidth',1)
xlabel('Time[s]')
ylabel('External Wrench Estimation')
grid on
legend('f_ex','f_ey','f_ez','t_ex','t_ey','t_ez')

figure()
subplot(3,1,1)
plot(time,abs(wrench_e(1,:)-x_disturbance),'lineWidth',1)
xlabel('time[sec]')
ylabel('force [N]')
grid on

subplot(3,1,2)
plot(time,abs(wrench_e(2,:)-y_disturbance),'lineWidth',1)
xlabel('time[sec]')
ylabel(' force [N]')
grid on


subplot(3,1,3)
plot(time,abs(wrench_e(6,:)-yaw_disturbance),'lineWidth',1)
xlabel('time[sec]')
ylabel('f_z [N]')
grid on

%% Functions 
%Q Matrix Computation
function Q = compute_Q(eta_b)
    
    Q=[1 , 0 , -sin(eta_b(2)); 0, cos(eta_b(1)), sin(eta_b(1))*cos(eta_b(2)); 0, -sin(eta_b(1)), cos(eta_b(1))*cos(eta_b(2))];

end

%Q_dot Matrix Computation
function Q_dot = compute_Q_dot(eta_b,eta_b_dot)

    Q_dot=[0 0 -cos(eta_b(2))*eta_b_dot(2);
           0 -sin(eta_b(1))*eta_b_dot(1) -sin(eta_b(2))*sin(eta_b(1))*eta_b_dot(2)+cos(eta_b(2))*cos(eta_b(1))*eta_b_dot(1);
           0 -cos(eta_b(1))*eta_b_dot(1) -sin(eta_b(2))*cos(eta_b(1))*eta_b_dot(2)-cos(eta_b(2))*sin(eta_b(1))*eta_b_dot(1)];

end

%C Matrix Computation
function C = compute_C(eta_b_dot,Ib,Q,Q_dot)

    S = skew(Q*eta_b_dot');
    C = Q'*S*Ib*Q + Q'*Ib*Q_dot; 
    
end

%M Matrix Computation
function M = compute_M(Q,I_b)
    M = Q'*I_b*Q;
end

%Rb Matrix Computation
function Rb = compute_Rb(eta_b)
    phi=eta_b(1);
    theta=eta_b(2);
    psi=eta_b(3);
    Rb= [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                  cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                  -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
end
