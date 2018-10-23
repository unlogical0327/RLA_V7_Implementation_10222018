%% Screen the data point based on the reflection and the adjacent point
% continuity. Identify the reflector from background and check if the reflector is identical.
function [detected_ID,detected_reflector]=identify_reflector(amp_thres,reflector_diameter,distance_delta,Lidar_Table,Lidar_data)
detected_ID=0;
iii=0;
loss=-30;    % loss in dB, need the calibration
%amp=2096;    % max amplitude from calibration
%reflector_diameter=80;  % reflector diameters
amp_thres;
detected_reflector=0;
size(Lidar_data)
for ii=2:length(Lidar_data)-1
    if Lidar_data(3,ii)>=amp_thres
            if Lidar_data(3,ii)>Lidar_data(3,ii-1) && Lidar_data(3,ii)>Lidar_data(3,ii+1)
                iii=iii+1;
                detected_ID_total(iii)=iii;
                detected_reflector_x(detected_ID_total(iii))=Lidar_Table(ii,1);
                detected_reflector_y(detected_ID_total(iii))=Lidar_Table(ii,2);
                detected_reflector_angle(detected_ID_total(iii))=Lidar_data(1,ii);
                detected_reflector_dist(detected_ID_total(iii))=Lidar_data(2,ii);
                disp('Raw detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID_total(iii)));
            end
    end
end
if (iii<=1)  % in case no reflector detected
    detected_reflector=0;
    disp('No reflector Detected!!!');
else
%%%%--- merge all peaks from the same refletor
l=1;iii=1;
detected_ID(1)=1;
detected_reflector(1,1)=detected_reflector_x(1);
detected_reflector(1,2)=detected_reflector_y(1);
for ii=(l+1):length(detected_reflector_angle)
               angle_delta=reflector_diameter/Lidar_data(2,ii)/pi*180;
               if (abs(detected_reflector_dist(ii)-detected_reflector_dist(l))<distance_delta) && (abs(detected_reflector_angle(ii)-detected_reflector_angle(l))<angle_delta)
             % check if the detected point is from the same reflector
                11;
                detected_ID(iii)=iii;
                detected_reflector(detected_ID(iii),1)=(detected_reflector_x(ii)+detected_reflector(detected_ID(iii),1))/2;
                detected_reflector(detected_ID(iii),2)=(detected_reflector_y(ii)+detected_reflector(detected_ID(iii),2))/2;
            else %(abs(Lidar_data(1,ii)-Lidar_data(1,ii-1))<angle_delta)
                12;
                iii=iii+1;
                detected_ID(iii)=iii;
                detected_reflector(detected_ID(iii),1)=detected_reflector_x(ii);
                detected_reflector(detected_ID(iii),2)=detected_reflector_y(ii);

                disp('Detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID(iii)));
               end
               l=l+1;
end
% figure(104);plot(detected_reflector(:,1),detected_reflector(:,2),'+')
% detected_reflector
end
if (length(detected_ID)<=1 && detected_ID(1)==0)  % in case no reflector detected
    detected_reflector=0;
    disp('No reflector Detected!!!');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-- Below code tests the algorithm to find and merge conditional detected reflectors 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aa=[   1525
%     1524
%     1523
%     1520
%         1487
%         1484
%         1480
%         1407
%         1314
%         1303
%         12990
%         12380];
%     l=1;
%     k=1;
%     zz(1)=aa(1);
% for i=(l+1):length(aa)
%     if abs(aa(l)-aa(i))<10
%     zz(k)=(aa(l)+aa(i))/2;
%     else 
%         k=k+1;
%         zz(k)=aa(i);
%     end
%     l=l+1;
% end