% created by    Yanuar T. [yanuar@student.chalmers.se]

function prepare_reference(gps_path)

% 1st
gt1 = load(strcat(gps_path, '160222_124022_LocalProcessed.mat'));
% 2nd
gt2 = load(strcat(gps_path, '160303_065851_WindowsProcessed.mat'));
% 3rd
gt3 = load(strcat(gps_path, 'mobileLinuxVerified.mat'));
% 4th
gt4 = load(strcat(gps_path, '160329_062234_Windows.mat'));
% 5th
gt5 = load(strcat(gps_path, '160405_062702_LinuxProcessed.mat'));
% 6th
gt6 = load(strcat(gps_path, '160310_071130_Novatel.mat'));
% 7th
gt7 = load(strcat(gps_path, '160412_105032_Linux.mat'));
% 8th
gt8 = load(strcat(gps_path, '160413_051732_Linux.mat'));
% 9th
gt9 = load(strcat(gps_path, '160413_105912_Linux.mat'));  % vehicle is too slow
% 10th
gt10 = load(strcat(gps_path, '160414_071253_Linux.mat'));

%% extract xyz of 10 runs
xyz1 = [gt1.rtStruct.SwerefEasting; gt1.rtStruct.SwerefNorthing; gt1.rtStruct.PosAlt];
xyz2 = [gt2.rtStruct.SwerefEasting; gt2.rtStruct.SwerefNorthing; gt2.rtStruct.PosAlt];
xyz3 = [gt3.rtStruct.SwerefEasting; gt3.rtStruct.SwerefNorthing; gt3.rtStruct.PosAlt];
xyz4 = [gt4.rtStruct.SwerefEasting; gt4.rtStruct.SwerefNorthing; gt4.rtStruct.PosAlt];
xyz5 = [gt5.rtStruct.SwerefEasting; gt5.rtStruct.SwerefNorthing; gt5.rtStruct.PosAlt];
xyz6 = [gt6.rtStruct.SwerefEasting; gt6.rtStruct.SwerefNorthing; gt6.rtStruct.PosAlt];
xyz7 = [gt7.rtStruct.SwerefEasting; gt7.rtStruct.SwerefNorthing; gt7.rtStruct.PosAlt];
xyz8 = [gt8.rtStruct.SwerefEasting; gt8.rtStruct.SwerefNorthing; gt8.rtStruct.PosAlt];
xyz9 = [gt9.rtStruct.SwerefEasting; gt9.rtStruct.SwerefNorthing; gt9.rtStruct.PosAlt];
xyz10 = [gt10.rtStruct.SwerefEasting; gt10.rtStruct.SwerefNorthing; gt10.rtStruct.PosAlt];

%% downsample the gps runs and synchronization to temporary reference (1st GPS data)

%  1. choose the reference run. given n number of runs, run_i, i=1, i<n, is considered a reference run.
%  2. choose the first point in the reference run, which is closest to the first camera position
%  3. choose the end point in the reference, which is the closest to the last camera position
%  4. count the number of GPS XYZ data, here in this case it gives us 850 samples
%  5. find the other 9's closest starting points from the reference run starting point
%  6. extract 850 samples for other 9's runs starts from the starting points


%% find nearest neighbor code snippet (commented if not necessary)
% downsample for just Gnist?ngtunneln data (each has 850 samples)
% gpsxyz = [xyz10(1,:); xyz10(2,:); xyz10(3,:)];
% gpscloud = pointCloud(gpsxyz.');
% [index, distance] = findNearestNeighbors(gpscloud, [xyz1(1,192750), xyz1(2,192750), xyz1(3,192750)], 1);

%% create a range based on the 'index' founded from above snippet
range1 = 192750:192750+850;     % act as the key run
range2 = 621895:621895+850;
range3 = 463501:463501+850;
range4 = 536433:536433+850;
range5 = 535765:535765+850;
range6 = 356922:356922+850;
range7 = 562508:562508+850;
range8 = 250993:250993+850;
range9 = 194955:194955+850;
range10 = 232254:232254+850;

xyz1_d = [xyz1(1,range1); xyz1(2,range1); xyz1(3,range1)];
xyz2_d = [xyz2(1,range2); xyz2(2,range2); xyz2(3,range2)];
xyz3_d = [xyz3(1,range3); xyz3(2,range3); xyz3(3,range3)]; 
xyz4_d = [xyz4(1,range4); xyz4(2,range4); xyz4(3,range4)];
xyz5_d = [xyz5(1,range5); xyz5(2,range5); xyz5(3,range5)];
xyz6_d = [xyz6(1,range6); xyz6(2,range6); xyz6(3,range6)];
xyz7_d = [xyz7(1,range7); xyz7(2,range7); xyz7(3,range7)];
xyz8_d = [xyz8(1,range8); xyz8(2,range8); xyz8(3,range8)];
xyz9_d = [xyz9(1,range9); xyz9(2,range9); xyz9(3,range9)];
xyz10_d = [xyz10(1,range10); xyz10(2,range10); xyz10(3,range10)];

%% now with longer range
range1l = 192750-1000:192750+1850;     % act as the key run
range2l = 621895-1000:621895+1850;
range3l = 463501-1000:463501+1850;
range4l = 536433-1000:536433+1850;
range5l = 535765-1000:535765+1850;
range6l = 356922-1000:356922+1850;
range7l = 562508-1000:562508+1850;
range8l = 250993-1000:250993+1850;
range9l = 194955-1000:194955+1850;
range10l = 232254-1000:232254+1850;

xyz1_l = [xyz1(1,range1l); xyz1(2,range1l); xyz1(3,range1l)];
xyz2_l = [xyz2(1,range2l); xyz2(2,range2l); xyz2(3,range2l)];
xyz3_l = [xyz3(1,range3l); xyz3(2,range3l); xyz3(3,range3l)]; 
xyz4_l = [xyz4(1,range4l); xyz4(2,range4l); xyz4(3,range4l)];
xyz5_l = [xyz5(1,range5l); xyz5(2,range5l); xyz5(3,range5l)];
xyz6_l = [xyz6(1,range6l); xyz6(2,range6l); xyz6(3,range6l)];
xyz7_l = [xyz7(1,range7l); xyz7(2,range7l); xyz7(3,range7l)];
xyz8_l = [xyz8(1,range8l); xyz8(2,range8l); xyz8(3,range8l)];
xyz9_l = [xyz9(1,range9l); xyz9(2,range9l); xyz9(3,range9l)];
xyz10_l = [xyz10(1,range10l); xyz10(2,range10l); xyz10(3,range10l)];

%% shift all the data relative from (0,0,0) which is now the first XYZ of the reference
% set the shift distance equals to the first XYZ of the reference
xmin = xyz1_d(1,1);
ymin = xyz1_d(2,1);
zmin = xyz1_d(3,1);
xmax = xyz1_d(1,851);
ymax = xyz1_d(2,851);
zmax = xyz1_d(3,851);

%% shift
xyz1_d(1,:) = xyz1_d(1,:)   - xmin; xyz1_d(2,:)  = xyz1_d(2,:)  - ymin; xyz1_d(3,:) = xyz1_d(3,:)   - zmin;
xyz2_d(1,:) = xyz2_d(1,:)   - xmin; xyz2_d(2,:)  = xyz2_d(2,:)  - ymin; xyz2_d(3,:) = xyz2_d(3,:)   - zmin;
xyz3_d(1,:) = xyz3_d(1,:)   - xmin; xyz3_d(2,:)  = xyz3_d(2,:)  - ymin; xyz3_d(3,:) = xyz3_d(3,:)   - zmin;
xyz4_d(1,:) = xyz4_d(1,:)   - xmin; xyz4_d(2,:)  = xyz4_d(2,:)  - ymin; xyz4_d(3,:) = xyz4_d(3,:)   - zmin;
xyz5_d(1,:) = xyz5_d(1,:)   - xmin; xyz5_d(2,:)  = xyz5_d(2,:)  - ymin; xyz5_d(3,:) = xyz5_d(3,:)   - zmin;
xyz6_d(1,:) = xyz6_d(1,:)   - xmin; xyz6_d(2,:)  = xyz6_d(2,:)  - ymin; xyz6_d(3,:) = xyz6_d(3,:)   - zmin;
xyz7_d(1,:) = xyz7_d(1,:)   - xmin; xyz7_d(2,:)  = xyz7_d(2,:)  - ymin; xyz7_d(3,:) = xyz7_d(3,:)   - zmin;
xyz8_d(1,:) = xyz8_d(1,:)   - xmin; xyz8_d(2,:)  = xyz8_d(2,:)  - ymin; xyz8_d(3,:) = xyz8_d(3,:)   - zmin;
xyz9_d(1,:) = xyz9_d(1,:)   - xmin; xyz9_d(2,:)  = xyz9_d(2,:)  - ymin; xyz9_d(3,:) = xyz9_d(3,:)   - zmin;
xyz10_d(1,:) = xyz10_d(1,:) - xmin; xyz10_d(2,:) = xyz10_d(2,:) - ymin; xyz10_d(3,:) = xyz10_d(3,:) - zmin;

%% save to file
save ('downsampled_gps.mat', ...
        'xyz1_d', ...
        'xyz2_d', ...
        'xyz3_d', ...
        'xyz4_d', ...
        'xyz5_d', ...
        'xyz6_d', ...
        'xyz7_d', ...
        'xyz8_d', ...
        'xyz9_d', ...
        'xyz10_d');

%% plotting (commented)
% 
% figure(2); 
% subplot(1,4,1);
% axis ([xmin xmax ymin ymax]);
% scatter(pcl3(:,1), pcl3(:,2), 5, 'oc', 'filled'); hold on;
% plot(xyz1_d(1,:), xyz1_d(2,:)); 
% plot(xyz2_d(1,:), xyz2_d(2,:));
% plot(xyz3_d(1,:), xyz3_d(2,:));
% plot(xyz4_d(1,:), xyz4_d(2,:));
% plot(xyz5_d(1,:), xyz5_d(2,:));
% plot(xyz6_d(1,:), xyz6_d(2,:));
% plot(xyz7_d(1,:), xyz7_d(2,:));
% plot(xyz8_d(1,:), xyz8_d(2,:));
% plot(xyz9_d(1,:), xyz9_d(2,:));
% plot(xyz10_d(1,:), xyz10_d(2,:));
% title('Lateral GPS Trajectory', 'FontSize', 16);
% xlabel('Easting (meter)');
% ylabel('Northing (meter)');
% 
% subplot(1,4,2);
% axis ([xmin xmax ymin ymax]);
% scatter(pcl3(:,1), pcl3(:,2), 5, 'oc', 'filled'); hold on;
% plot(xyz1_d(1,:), xyz1_d(2,:)); 
% plot(xyz2_d(1,:), xyz2_d(2,:));
% plot(xyz3_d(1,:), xyz3_d(2,:));
% plot(xyz4_d(1,:), xyz4_d(2,:));
% plot(xyz5_d(1,:), xyz5_d(2,:));
% plot(xyz6_d(1,:), xyz6_d(2,:));
% plot(xyz7_d(1,:), xyz7_d(2,:));
% plot(xyz8_d(1,:), xyz8_d(2,:));
% plot(xyz9_d(1,:), xyz9_d(2,:));
% plot(xyz10_d(1,:), xyz10_d(2,:));
% title('Outside', 'FontSize', 16);
% xlabel('Easting (meter)');
% ylabel('Northing (meter)');
% 
% subplot(1,4,3);
% axis ([xmin xmax ymin ymax]);
% scatter(pcl3(:,1), pcl3(:,2), 5, 'oc', 'filled'); hold on;
% plot(xyz1_d(1,:), xyz1_d(2,:)); 
% plot(xyz2_d(1,:), xyz2_d(2,:));
% plot(xyz3_d(1,:), xyz3_d(2,:));
% plot(xyz4_d(1,:), xyz4_d(2,:));
% plot(xyz5_d(1,:), xyz5_d(2,:));
% plot(xyz6_d(1,:), xyz6_d(2,:));
% plot(xyz7_d(1,:), xyz7_d(2,:));
% plot(xyz8_d(1,:), xyz8_d(2,:));
% plot(xyz9_d(1,:), xyz9_d(2,:));
% plot(xyz10_d(1,:), xyz10_d(2,:));
% title('Early Part', 'FontSize', 16);
% xlabel('Easting (meter)');
% ylabel('Northing (meter)');
% 
% subplot(1,4,4);
% axis ([xmin xmax ymin ymax]);
% scatter(pcl3(:,1), pcl3(:,2), 5, 'oc', 'filled'); hold on;
% plot(xyz1_d(1,:), xyz1_d(2,:)); 
% plot(xyz2_d(1,:), xyz2_d(2,:));
% plot(xyz3_d(1,:), xyz3_d(2,:));
% plot(xyz4_d(1,:), xyz4_d(2,:));
% plot(xyz5_d(1,:), xyz5_d(2,:));
% plot(xyz6_d(1,:), xyz6_d(2,:));
% plot(xyz7_d(1,:), xyz7_d(2,:));
% plot(xyz8_d(1,:), xyz8_d(2,:));
% plot(xyz9_d(1,:), xyz9_d(2,:));
% plot(xyz10_d(1,:), xyz10_d(2,:));
% title('Inside Part', 'FontSize', 16);
% xlabel('Easting (meter)');
% ylabel('Northing (meter)');
% 
% grid off;