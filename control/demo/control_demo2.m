%%Control_demo
% Description
% This function demonstrates the implementation of the MPC controller.
% I copy-pasted the MPC_test2.m here. You can follow the format of
% control_demo1.m to modify this file.
clc; clear; close all;

import casadi.*
% states = [ u v r x y psi delta n ]'
states_linear = SX.sym('states_liner',8,1);
states_nlinear = SX.sym('states_nliner',8,1);
inputs = SX.sym('inputs',2,1);

ode_l = Dynamics_simplified_CA(states_linear,inputs);

ode_nl = Dynamics_CA(states_nlinear,inputs);


Dy_l = Function('Dy_l',{states_linear,inputs},{ode_l}); % creating a casadi function for calculation of simplified dynamics

Dy_nl = Function('Dy_l',{states_nlinear,inputs},{ode_nl}); % creating a casadi function for calculation of dynamics

T = 0.1; % Sampling Time
N = 50; % Prediction Horizon

num_states = length(states_linear);
num_control = length(inputs);

% X = [ u v r x y psi delta n ]'

X = SX.sym('X', num_states, N+1);
U = SX.sym('U', num_control, N);

% controled_states = X(4:6,:);

% num_controled_states = size(controled_states,1);

P = SX.sym('P', num_states + num_states, 1);

g = [];

obj = 0;
Q = zeros(8,8); Q(4,4) = 10; Q(5,5) = 50; Q(6,6) = 0.1;

R = zeros(2,2); R(1,1) = 0.01; R(2,2) = 0.0;

st = X(:,1);
g = [g;st-P(1:num_states)];
for k = 1:N
    st = X(:,k); con = U(:,k);
    obj = obj + ...
        (st - P(num_states + 1:num_states + num_states))'*Q*...
        (st - P(num_states + 1:num_states + num_states)) +...
        con'*R*con;
    st_next = X(: , k + 1);
    f_value = Dy_l(X(:,k) , U(:,k));
    st_next_euler = st + T*f_value; %based on controlled states

    g = [g;st_next - st_next_euler];

end

OPT_variables = [reshape(X,num_states*(N+1),1);reshape(U,num_control*N,1)];

nlp_prob = struct('f',obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level = 0; %0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver =nlpsol('solver', 'ipopt', nlp_prob, opts);

args =struct;

args.lbg(1:num_states*(N+1)) = -1e-20;
args.ubg(1:num_states*(N+1)) = 1e-20;

args.lbx(1:num_states*(N+1),1) = -inf;
args.ubx(1:num_states*(N+1),1) = inf;

args.lbx(num_states*(N+1)+1:2:num_states*(N+1) + num_control*N) = -20; % min delta
args.ubx(num_states*(N+1)+1:2:num_states*(N+1) + num_control*N) = 20; % max delta

args.lbx(num_states*(N+1)+2:2:num_states*(N+1) + num_control*N) = 0; % min rpm
args.ubx(num_states*(N+1)+2:2:num_states*(N+1) + num_control*N) = 350; % max rpm

%% creating the sim
t0 = 0;
x0 = [3 0.5 0.06 0 0 0 0 340]';
u0 = zeros(num_control, N);
X0 = repmat(x0,1,N+1);

sim_time = 50;

XS = [3 0.5 0.06 25 5 0.2 0 0]';
% XS = [3 0.5 0.06 0.7 0.1 1.5 0 0]';

x_his = [];
t_his = [];
u_his = [];

% updating the history
x_his = [x_his;x0'];
t_his = [t_his;t0];
u_his = [u_his;u0(:,1)'];

mpc_iter = 0;

while(norm(x0(4:6)-XS(4:6),2)>2e-1 && mpc_iter < sim_time/T)
    args.p = [x0;XS];

    args.x0 = [reshape(X0,num_states*(N+1),1); reshape(u0,num_control*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
    u = reshape(full(sol.x(num_states*(N+1)+1:end)), num_control, N);

    xx = reshape(full(sol.x(1:num_states*(N+1))), num_states, N+1);

    [t0, x0, u0] = shift(T, t0, x0, u, Dy_nl);

    x_his = [x_his;x0'];
    t_his = [t_his;t0];
    u_his = [u_his;u0(:,1)'];

    X0 = reshape(full(sol.x(1:num_states*(N+1))),num_states,N+1);

    mpc_iter  = mpc_iter + 1;

    disp(mpc_iter);

end
disp('error: ') 
disp(XS - x0)

figure(1);
plot(x_his(:,5),x_his(:,4));
hold on;
plot(XS(5),XS(4),'o')
grid,axis('equal'),xlabel('East (y)'),ylabel('North (x)'),title('Ship position')

figure(2)
subplot(211), plot(t_his,u_his(:,1)), grid, xlabel('Time (s)'),ylabel('Rudder Angle(deg)'),title('Ship Command')
subplot(212), plot(t_his,u_his(:,2)), grid, xlabel('Time (s)'),ylabel('N (rpm)'),title('Ship Command')