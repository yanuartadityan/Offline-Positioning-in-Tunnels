% the data are chosen by david, matlab by yanuar

function manual_lane_marking (path)

load (strcat(path, 'filteredCloudS.mat'));
load (strcat(path, 'cameraPosition.mat'));

gps_d =[143427.604, 6394346.637, 32.252;
        143430.926, 6394358.220, 32.375;
        143434.570, 6394369.674, 32.500;
        143438.394, 6394381.048, 32.611;
        143442.510, 6394392.247, 32.717;
        143446.770, 6394403.400, 32.814;
        143451.317, 6394414.726, 32.900;
        143455.991, 6394425.780, 32.982;
        143460.843, 6394436.708, 33.067;
        143465.987, 6394447.620, 33.134;
        143471.311, 6394458.432, 33.259;
        143476.831, 6394469.129, 33.315;
        143482.510, 6394479.712, 33.335;
        143488.227, 6394490.223, 33.399;
        143494.154, 6394500.701, 33.448;
        143500.165, 6394511.035, 33.472;
        143506.342, 6394521.400, 33.465;
        143512.675, 6394531.708, 33.487;
        143518.948, 6394541.853, 33.501];
    

lane_dist = 1.75; % meter
f_hand = figure;
set (f_hand, 'Position', [100 100 960 960]);
hold on;
scatter(pcl3(:,1), pcl3(:,2), 10, 'oc', 'filled');
plot(gps_d(:,1), gps_d(:,2), '-r', 'LineWidth', 2); 
plot(cameraPosition(1,:), cameraPosition(2,:), '-xk', 'LineWidth', 1);

rightlane = [];
for i=1:(length(gps_d)-1)
    dx = gps_d(i+1,1) - gps_d(i,1);
    dy = gps_d(i+1,2) - gps_d(i,2);
    
    m1 = dy/dx;
    m  = [1 -1/m1];
    m_norm = m./norm([1 m]);
    
    A = [gps_d(i,1) gps_d(i,2)];
    C = A + [dx/2 dy/2];
    
    D = C + m_norm * lane_dist;

    rightlane = [rightlane; D];
    
    plot([C(1) D(1)], [C(2) D(2)], '-.k', 'LineWidth', 2);
    scatter(D(1), D(2), 40, 'filled');
end
plot (rightlane(:,1), rightlane(:,2), '-b', 'LineWidth', 2);
grid on;
axis equal;
