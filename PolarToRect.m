% this program change polar coordinate data to rectangular x-y coordinate
function [calibration_data,scan_data]=PolarToRect(Reflector_map, Lidar_data, data_source_flag)
% Lidar data --1 scan angle; --2 distance; --3 amplitude
calibration_data=0;

if data_source_flag==1
    for ii=1:length(Lidar_data)
        calibration_data(ii,1) = cos(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii);
        calibration_data(ii,2) = sin(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii);
        calibration_data(ii,3) = Lidar_data(3,ii);
    end
    scan_data = Lidar_data;
    %plot(calibration_data(:,1),calibration_data(:,2),'.');
elseif data_source_flag==2     % generate simulated data
    fname = ['Lidar_data_example2'];
    Lidar_data = dlmread( fname, ' ', 3, 0)';
    for ii=1:length(Lidar_data)
        calibration_data(ii,1) = cos(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii);
        calibration_data(ii,2) = sin(Lidar_data(1,ii)/180*pi)*Lidar_data(2,ii);
        calibration_data(ii,3) = Lidar_data(3,ii);
    end
    ll(1:length(Reflector_map))=100;
    Reflector_Table=[Reflector_map';ll]';
    calibration_data=[calibration_data;Reflector_Table];
    Reflector_data(1,:)=atan(Reflector_Table(:,2)./Reflector_Table(:,1));
    Reflector_data(2,:)=(Reflector_Table(:,1).^2+Reflector_Table(:,2).^2).^0.5;
    Reflector_data(3,:)=100;
    scan_data = Lidar_data;
    scan_data=[scan_data';Reflector_data']';
end

