clear
clc

folder = 'pdw';  % Set the folder path
files = dir(fullfile(folder, '*.shp'));  % Get information of all .shp files in the folder
desirename=["depare","bridge","wtwaxs","lndare","notmrk"]; %give the desirename

pgon_memory = maps.shape_multi(desirename,files,folder,desirename(1));%Refer to Categories in the workspace to determine the order of desirename
pgon_memory_2 = maps.shape_multi(desirename,files,folder,desirename(2));
pgon_memory_3 = maps.shape_multi(desirename,files,folder,desirename(3));
pgon_memory_4 = maps.shape_multi(desirename,files,folder,desirename(4));
pgon_memory_5 = maps.shape_multi(desirename,files,folder,desirename(5));

%%%%polt
os_x = 2.65*10^5;
os_y = 7.048*10^6;
traj_x = 2.6*10^5;
traj_y = 7.06*10^6;
traj_xx = 2.6*10^5;
traj_yy = 7.05*10^6;
pgon_pl=plot(pgon_memory.polygons);
hold on
pgon_pl2=plot(pgon_memory_2.polygons);
hold on
line_pl=line(pgon_memory_3.lines(1,:),pgon_memory_3.lines(2,:));
line_pl.LineStyle = '--' ;
line_pl.LineWidth = 1;
hold on
pgon_pl4=plot(pgon_memory_4.polygons);
hold on
sz=5;
point_pl5=scatter(pgon_memory_5.points(1,:),pgon_memory_5.points(2,:),sz,'MarkerEdgeColor',[0 .5 .5],...
'MarkerFaceColor',[0 .7 .7],'LineWidth',1.5);
hold off

