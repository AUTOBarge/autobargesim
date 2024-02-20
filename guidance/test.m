clear;
clc;

wp_pos = [50 50;
          70 70;
          90 90;
          100 100];
wp_speed = [2;
            3;
            3;
            2];
x = [50, 45, deg2rad(45), 0]; %[x, y, chi, U]
wp_idx = 1;

los = LOSguidance();
state = x; %[x, y, chi, U]
Tp=0.05; %sampling time
state_new =zeros(1,4);
% inside the loop
for i=1:600
wp_idx = los.find_active_wp_segment(wp_pos, state, wp_idx);

[chi, U] = los.compute_LOSRef(wp_pos, wp_speed, state, wp_idx);
% Update state
state_new(1) =state(1) + Tp*U*cos(state(3));
state_new(2) =state(2) + Tp*U*sin(state(3));
state_new(3) =chi;
state_new(4) =U;
state=state_new;
% Plot the trajectory for visualization
cla
plot(wp_pos(:,1),wp_pos(:,2),'-*r')
hold on
plot(state(1),state(2),'ob',LineWidth=2);
plot([state(1),state(1)+2*cos(state(3))],[state(2),state(2)+2*sin(state(3))],'-b',LineWidth=2);
pause(0.01);
end
% inside the loop