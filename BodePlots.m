% This script should be run after the RunSimulation.m script has been
% run, as it relies on the variables declared in that script

D_mat = zeros(size(C, 1), size(B, 2));

% Get the continuous open loop transfer function
[num, den] = ss2tf(A, B, C, D_mat, 1);
open_loop_tf_c = tf(num, den);
figure()
pzplot(open_loop_tf_c)

% Get the continuous closed loop transfer function for the LQR controller
[num, den] = ss2tf(A-B(:,1)*K_lqr, B, C, D_mat, 1);
closed_loop_tf_c_lqr = tf(num, den);

% Show the bode plot
figure()
subplot(1,2,1)
bode(open_loop_tf_c)
subplot(1,2,2)
bode(closed_loop_tf_c_lqr)

% Get the  loop transfer function
open_loop_tf_d = c2d(open_loop_tf_c, Ts, 'tustin');
figure()
pzplot(open_loop_tf_d)