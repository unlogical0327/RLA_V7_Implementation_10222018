%%%%Plot lidar data map, reflector and lidar trace
function []=Plot_world_map(Lidar_update_Table,match_reflect_pool,matched_reflect_ID,detected_reflector,detected_ID,Lidar_trace)

figure(103)

%subplot(1,2,2)
%figure(103,'Name','World map Display'); 
grid on
xlabel('x(mm)')
ylabel('y(mm)')
title('World map Display')
set (gcf,'Position',[700,50,600,600], 'color','w')
%hold on;
%zz=length(Lidar_update_Table);

%plot(Lidar_update_Table(end,1),Lidar_update_Table(end,2),'+','Color',[rand(),rand(),rand()]);
plot(match_reflect_pool(matched_reflect_ID,1),match_reflect_pool(matched_reflect_ID,2),'+k')
color='b';
%hold on;
color='g';
plot_reflector(detected_reflector,detected_ID,color)
%hold on;
plot(Lidar_trace(end,1),Lidar_trace(end,2),'o-k')
plot(Lidar_update_Table(:,1),Lidar_update_Table(:,2),'.','Color',[rand(),rand(),rand()]);
axis equal %square
hold off