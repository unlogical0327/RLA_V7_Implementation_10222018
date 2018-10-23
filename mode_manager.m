%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RLA flow design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is the top level code to implement matlab-to-C++
% verification platform
% RLA has options to generate test vectors to verify the algorithm.
% This program is developed and copyright owned by Soleilware LLC
% The code is writen to build the blocks for the localization
% algorithm process and efficiency.
% --------------------------------
% Created by Qi Song on 9/18/2018
%function [status]=RLA_toplevel(list_source_flag)% RLA top level function to convert Matlab code to C++ package and run C++ test code
function [mode,status,update_match_pool] = mode_manager(interrupt,scan_freq,reflector_source_flag,req_update_match_pool,num_ref_pool,num_detect_pool,scan_data,amp_thres,reflector_diameter,dist_delta,thres_dist_match,thres_dist_large)
%% -interrupt:              interrupt from GUI console to control the Lidar computing engine
%% -reflector_source_flag:  flag to define the reflector source from GUI
%% -data_source_flag:       flag to define the data source from GUI
%% -req_update_match_pool:  request to ask match pool to update to include more reflectors
%% -Reflector_map:          load Reflector map from GUI console
%% -scan_data:              load 3D Lidar data to module
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_source_flag=1;

%if reflector_source_flag == 0 % read from file
%elseif reflector_source_flag == 2 % generate the 120 reflector matrix
%    for i=1:2
%    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%end
scan_flag=1;
%% Load Reflector map
if scan_flag==0
    [Reflector_map, Reflector_ID, load_ref_map_status] = load_reflector_map();
elseif scan_flag==1
    %%-- Read scan data as the reference reflector map
    fname = ['Lidar_data_example2'];
    %Lidar_data = dlmread( fname, ' ', 3, 0)';
    %scan_data = load(fname)';
    scan_data= dlmread( fname, '\t', 3, 0)';
    read_file=0;
    [Reflector_map, Reflector_ID, load_ref_map_status]=reflector_map_cali_scan(amp_thres,reflector_diameter,dist_delta,scan_data,data_source_flag,read_file);
end
%%-- Only for test data
fname = ['Lidar_data_example3'];
Lidar_data = dlmread( fname, '\t', 3, 0)';
%scan_data = Lidar_data;
b=1;
while(b==1)
    %% convert polar data to rectangle data
    [calibration_data,scan_data]=PolarToRect(Reflector_map,Lidar_data,data_source_flag);
    %%-- Run calibration mode
    [cali_status,Lidar_trace] = calibration_mode(amp_thres,reflector_diameter,dist_delta,Reflector_map,Reflector_ID,calibration_data,scan_data,thres_dist_match,thres_dist_large);
    
    if cali_status==0
        disp('Calibration successful! Proceed to measurement mode....')
        break
    elseif cali_status==3
        disp('Data is bad, wait for new Lidar data!!')
    else
        disp('Calibration failed, please check Lidar data!!')
        break
    end
    mode='Calibration';
end
%% Measurement mode
%-- need to read the scan data and process the data at each scan
%measurement
Lidar_trace_p=0;
Lidar_update_Table_p=0;
detected_ID_p=0;
detected_reflector_p=0;
match_reflect_pool_p=0;
match_reflect_ID_p=0;

Loop_num=scan_freq;
c=1;
for ll=1:Loop_num     % simulation loop start from here!!!
    while c==1
        %% scan data is 2D data
        %% measurement_data only need angle and distance;
        % -- Could be replace by 2D scan data directly
        [measurement_data3,scan_data]=PolarToRect(Reflector_map,Lidar_data,data_source_flag);
        %%-- Plot raw data
        plot_Lidar_data(measurement_data3)
        %%-- Run measurement mode to find Robot location in the world coordinate
        [mea_status,Lidar_trace,Lidar_update_Table,match_reflect_pool,match_reflect_ID,detected_reflector,detected_ID] = measurement_mode(num_ref_pool,num_detect_pool,Reflector_map,Reflector_ID,measurement_data3,scan_data,amp_thres,reflector_diameter,dist_delta,Lidar_trace,thres_dist_match,thres_dist_large);
        %%
        if mea_status==3
            Lidar_trace=Lidar_trace_p;
            Lidar_update_Table=Lidar_update_Table_p;
            detected_ID=detected_ID_p;
            detected_reflector=detected_reflector_p;
            match_reflect_pool=match_reflect_pool_p;
            match_reflect_ID=match_reflect_ID_p;
        else
            Lidar_trace_p=Lidar_trace;
            Lidar_update_Table_p=Lidar_update_Table;
            detected_ID_p=detected_ID;
            detected_reflector_p=detected_reflector;
            match_reflect_pool_p=match_reflect_pool;
            match_reflect_ID_p=match_reflect_ID;
        end
        if mea_status==0
            disp('Measurement successful! continuing.....')
            status='good';
            break
        elseif mea_status==1
            disp('Measurement error found! Please check Lidar data!!')
            status='minor error';
            break
        elseif mea_status==2
            disp('Measurement large error found! Please stop test and check Lidar data!!')
            status='major error';
            break
        elseif mea_status==3
            disp('Measurement failed!')
            status='broken';
        end
    end
    %% --Plot final result in the world coordinate
    Plot_world_map(Lidar_update_Table,match_reflect_pool,match_reflect_ID,detected_reflector,detected_ID,Lidar_trace)
    pause(1)
end
mode='navigation';
update_match_pool='true';
