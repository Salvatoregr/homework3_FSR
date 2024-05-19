%Before running this script, make sure the 'geometric_control.slx' file has
%runned (check if out is available in the workspace, then it's possible to
%run this script)
close all

%% Parameters redefinition:
%Linear Position Errors
err_p_x = out.err_p.Data(:,1);
err_p_y = out.err_p.Data(:,2);
err_p_z = out.err_p.Data(:,3);

%Linear Velocity Errors
err_dp_x = out.err_dot_p.Data(:,1);
err_dp_y = out.err_dot_p.Data(:,2);
err_dp_z = out.err_dot_p.Data(:,3);

%Angular Position Errors
e_R_r = out.err_R.Data(:,1); 
e_R_p = out.err_R.Data(:,2);
e_R_y = out.err_R.Data(:,3);

%Angular Velocity Errors
e_W_r = out.err_W.Data(:,1);
e_W_p = out.err_W.Data(:,2);
e_W_y = out.err_W.Data(:,3);

%Total Thrust
u_T = out.uT.Data;

%Torque tau^b
taub_x = out.tau_b.Data(:,1);
taub_y = out.tau_b.Data(:,2);
taub_z = out.tau_b.Data(:,3);

%% Plots
%Linear Position Errors Plot
plot_pdf(out.err_p.Time,     err_p_x, err_p_y, err_p_z,    'Time [s]',  'Linear Position Errors',  'pos_err_x', 'pos_err_y', 'pos_err_z', 'PositionError.pdf');

%Linear Velocity Errors Plot
plot_pdf(out.err_dot_p.Time, err_dp_x, err_dp_y, err_dp_z, 'Time [s]', 'Linear velocity Errors',  'vel_err_x', 'vel_err_y', 'vel_err_z', 'LinearVelocityError.pdf');

%Angular Position Errors Plot
plot_pdf(out.err_R.Time,     e_R_r, e_R_p, e_R_y,      'Time [s]', 'Angular Position Errors', 'ang_pos_err_p', 'ang_pos_err_r', 'ang_pos_err_y', 'AngularError.pdf');

%Angular Velocity Erros Plot
plot_pdf(out.err_W.Time,     e_W_r, e_W_p, e_W_y,     'Time [s]','Angular Velocity Errors',  'ang_vel_err_p', 'ang_vel_err_r', 'ang_vel_err_y',  'AngularVelocityError.pdf');

%Total Thrust Plot
single_plot_pdf(out.uT.Time, u_T, 'Time[s]', 'Total Thrust', 'thrust_plot.pdf');

%Torque Plot
plot_pdf(out.tau_b.Time, taub_x,taub_y,taub_z, 'Time [s]', 'Input Torques [Nm]', 'taub_x','taub_y','taub_z',  'torques.pdf');
