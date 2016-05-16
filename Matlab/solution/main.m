%% add path
addpath ('lasdata');

%% load the point cloud (.las) using lasdata
% tunnelData = lasdata ('/Users/januaditya/Thesis/exjobb-data/volvo/pc/47.las');

%% extract the XYZ
% gnistangtunnelnXYZ = tunnelData.get_xyz;

%% build the matlab specific pointcloud
% gnistangtunnelnPC = pointCloud(gnistangtunnelnXYZ);

%% get the entrance scene
% entranceIndices = findNearestNeighbors(gnistangtunnelnPC, [143425.684418894 6394339.57677586 35.9966981618200], 4000000);

%% R matrix -- rotation matrix from PnP
rvec = [0.9374521391997079, -0.3434782435455127, -0.05662316593745959;
        -0.08260335620060169, -0.06147257392411345, -0.994684778308958;
        0.3381718087811107, 0.9371466368002576, -0.08600005162601933];
    
%% T matrix -- translation matrix from PnP
tvec = [2061863.850036867;
        404959.7605953381;
        -6040933.256341981];
    
%% C matrix -- calibration matrix
K = [1432, 0, 640;
     0, 1432, 480;
     0, 0, 1];
 
%% load the lookup table
lutWorld  =    [143430.318, 6394363.127, 39.797;
                143434.166, 6394361.662, 39.842;
                143432.108, 6394362.287, 39.617;
                143432.948, 6394364.927, 37.656;
                143427.658, 6394362.027, 38.376;
                143436.316, 6394359.472, 38.452;
                143427.048, 6394361.520, 33.577;
                143430.465, 6394361.396, 38.098;
                143437.223, 6394361.204, 39.037;
                143432.753, 6394362.541, 39.446];

lutPixels =    [397.210571, 145.146866;
                650.494934, 129.172379;
                519.567688, 131.898239;
                531.834473, 267.480103;
                239.835358, 207.141220;
                834.740051, 174.580566;
                211.190155, 510.402740;
                437.319458, 218.244186;
                845.259948, 160.413910;
                559.729248, 170.678528];

lut37Pixels = [834.7400512695312, 174.58056640625;
               239.8353576660156, 207.1412200927734;
               437.3194580078125, 218.2441864013672;
               531.83447265625, 267.4801025390625;
               559.729248046875, 170.6785278320312;
               845.2599487304688, 160.4139099121094;
               211.1901550292969, 510.4027404785156;
               519.5676879882812, 131.8982391357422;
               650.4949340820312, 129.1723785400391;
               397.2105712890625, 145.1468658447266;
               281.1081237792969, 95.38687133789062;
               166.3629608154297, 116.0634078979492;
               371.4561767578125, 120.8866500854492;
               824.3793334960938, 142.317626953125;
               812.8443603515625, 161.3762054443359;
               337.1262512207031, 162.532470703125;
               696.4364013671875, 165.0917053222656;
               775.031982421875, 167.0222625732422;
               754.8485717773438, 167.3820037841797;
               758.80322265625, 167.3997802734375;
               697.168701171875, 169.4677734375;
               301.3576965332031, 170.3515167236328;
               324.682373046875, 172.1241149902344;
               461.7184448242188, 174.9342193603516;
               301.1141967773438, 177.4057312011719;
               490.6653747558594, 180.9556884765625;
               701.9425659179688, 195.8117370605469;
               439.5482177734375, 242.9078216552734;
               556.896240234375, 245.8140869140625;
               426.1513366699219, 246.6478424072266;
               624.480712890625, 247.3087463378906;
               492.7648620605469, 249.5208892822266;
               460.8605346679688, 252.1749114990234;
               474.0901184082031, 256.2586669921875;
               385.8645324707031, 260.5527648925781;
               385.8645324707031, 260.5527648925781;
               525.922607421875, 278.1383666992188];
            
%% transform from world coordinate into camera coordinate
%
%     u_c = r * w_c + t
%
c_pointsProjection = computeTransformWorld2Camera(rvec, tvec, lutWorld);

%% transform from pixel coordinate into camera coordinate
%
%     u_p = K  *  u_c
%     u_c = K' *  u_p
%
c_pointsReprojection = computeTransformPixel2Camera(K, lut37Pixels);

%% get camera center in world space
%     
%     [0 0 0].' = r * X + t
%            X  = -inv(r) * t
%     
c_camera_center = [0 0 0];
w_camera_center = -inv(rvec)*tvec;

%% transform from camera coordinate into world coordinate
%     
%                      x    = r * X + t
%                  x - t    = r * X
%          inv(r) (x - t)   = X 
% [inv(r) -inv(r)|t] * x    = X
% 
[w_pixelsInWorlds, inverse_T ] = computeTransformCamera2World(rvec, tvec, c_pointsReprojection);

%% build ray
% 
%     rays = projectRay (origin, points)
%     return a 3D vector in any coordinate used by the points
w_rays = projectRays( w_camera_center, w_pixelsInWorlds);

%% draw longer ray through the 3D space
numOfNN = 10;
[w_tracedPoints, w_tracedDists] = computeReprojection2Dto3D (gnistangtunnelnXYZ, gnistangtunnelnPC, w_rays, numOfNN);

%% compute errors
% w_errors = computeErrors (lutWorld.', w_tracedPoints);

%% draw and plot
figure;
w_projectedPoints = zeros(3,2,size(w_tracedPoints,2));
for i=1:size(w_tracedPoints,2)
    w_projectedPoints(:,:,i) = [w_camera_center, w_tracedPoints(:,i)];
end

% draw rays
% drawRays (w_projectedPoints, 'rep');
% drawRays (w_rays, 'p');
% drawImagePlane ( K, inverse_T );
pcshow (gnistangtunnelnXYZ(entranceIndices, 1:3)); hold on;

% draw camera poses
w_poses  =        [143425.5458 6394339.7282 35.4776;
                   143425.4636 6394340.2403 35.7872;
                   143425.7920 6394340.1494 36.0087;
                   143425.0163 6394341.4250 35.4204];

plot3 (w_poses (:,1), w_poses (:,2), w_poses (:,3));               
grid on;