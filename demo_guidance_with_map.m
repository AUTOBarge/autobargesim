clear
clc
close all
%%
currentfolder = pwd;
addpath( genpath(currentfolder)); % Add the simulator directory to search path

mapfolder = strcat(currentfolder,'\maps\demo\.shp\Albert canal');  % Set the folder path for reading .shp files
desirename=["depare","bridge","wtwaxs","lndare"]; %give the desirename %"notmrk"

p = maps.processor(desirename, mapfolder);
p.plot();
%%
pl = maps.planner(p.pgon_memory);
% Defines the given starting and ending points
given_point1 = [4.4266966, 51.2383286]; % start at point 16th [lon, lat] 
given_point2 = [4.5088209, 51.2361481];  % end at point 11th [lon, lat]




% planned path
pl = pl.plan_path(given_point1, given_point2);

% Draw Path
pl.plot_path(1);

%%
wp_wgs84 = pl.path_points;
wgs84 = wgs84Ellipsoid;
lon0 = wp_wgs84(1,1);
lat0 = wp_wgs84(1,2);
h =0;
for i =1:length(wp_wgs84)
    [xEast,yNorth,zUp] = geodetic2enu(wp_wgs84(i,2),wp_wgs84(i,1),h,lat0,lon0,h,wgs84);
    wp_pos(i,:) =[xEast,yNorth];
end
figure(2)
plot(wp_pos(:,1),wp_pos(:,2))
wp_speed = 100*ones(length(wp_pos),1);
x_0 = [0 0 0 0 0 0];
wp_idx = 1;
los = LOSguidance();
% State of own ship w.r.t inertial coordinate frame state_x=[x, y, chi, U]
state = x_0;
% sampling time
Tp=0.05;
% temporary variable
state_new =zeros(1,6);
% Start the loop for simulation
% inside the loop
for i=1:800
    % Find the active waypoint
    wp_idx = los.find_active_wp_segment(wp_pos, state, wp_idx);
    % Call LOS algorithm
    [chi, U] = los.compute_LOSRef(wp_pos, wp_speed, state, wp_idx,1);

    % Update state
    state_new(4) =state(4) + Tp*U*cos(state(6));
    state_new(5) =state(5) + Tp*U*sin(state(6));
    state_new(6) =state(6) + 1/2*(chi-state(6));
    state_new(1) =U;
    state=state_new;
    % Plot the trajectory for visualization
    figure(3)
    
    plot(wp_pos(:,1),wp_pos(:,2),'-*r')
    
    hold on
    plot(state(4),state(5),'ob',LineWidth=2);
    plot([state(4),state(4)+5*cos(state(6))],[state(5),state(5)+5*sin(state(6))],'-b',LineWidth=2);
    xlabel('X (m)'),ylabel('Y (m)'),grid on
    axis([0 2000 -450 0]);
    legend({'waypoint','position of ship','heading direction'},'Location','southeast');
    pause(0.01);
end