%% all path
gps_log_path = '/Users/januaditya/Thesis/exjobb-data/volvo/gpsReferenceDataGBG/';
mat_path     = '../camera_data/';

%% build the gps reference based on the logs, the output would be downsampled_gps.mat
prepare_reference(gps_log_path);

%% build the reference using manual lane marking detection
manual_lane_marking(mat_path);