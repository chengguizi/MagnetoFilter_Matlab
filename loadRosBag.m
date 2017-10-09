function mat_result_div4 = loadRosBag(filePath)

if ~exist('filePath')
    if isunix
        filePath = '/home/dhl/Desktop/mag/2017-10-02-14-26-44.bag';
    elseif ispc 
        filePath = 'D:\TLAB_Work\2017-09-20-10-13-12.bag';
    else
        disp('Not Supported');
    end
end
disp('Loading Logbag...');

bag = rosbag(filePath);

bag.AvailableTopics

bagSelect1 = select(bag,'Topic','/cust_imu0')

%msgs1 = readMessages(bagSelect1);

disp('Generating Timeseries...')
ts1 = timeseries(bagSelect1,'Header.Stamp.Sec','Header.Stamp.Nsec','Header.Seq','Magnetometer.X','Magnetometer.Y','Magnetometer.Z');

disp('Extracting Matrix...');
mat_raw = ts1.data;
disp('Zeroing Start Time...');
initial_time(1:size(mat_raw,1),1) = mat_raw (1,1) + mat_raw(1,2)*1.0e-9;
mat_result= horzcat (mat_raw (:,1) + mat_raw(:,2)*1.0e-9 - initial_time(:,1), mat_raw(:,4), mat_raw(:,5), mat_raw(:,6));



% calibration matrix
U = [2.3291    0.2930    0.0387;
         0    5.5205   -0.7942;
         0         0    2.4020;];
     
c = [-0.2653; 0.1058; 0.0843];

new_size = fix (size(mat_result,1)/4);
mat_result_div4 = zeros(new_size,size(mat_result,2));
for index = 1:new_size
    rawData = mat_result(index*4-3,[2:4]).';
    calibData = U*(rawData-c);
    mat_result_div4 (index, :) =  [ mat_result(index*4-3,1) , calibData.'];
    
end

