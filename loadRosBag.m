function mat_result_div4 = loadRosBag(filePath)


bag = rosbag(filePath);

bag.AvailableTopics;

bagSelect1 = select(bag,'Topic','/cust_imu0');

%msgs1 = readMessages(bagSelect1);

disp('Generating Timeseries...')
ts1 = timeseries(bagSelect1,'Header.Stamp.Sec','Header.Stamp.Nsec','Header.Seq','Magnetometer.X','Magnetometer.Z');

disp('Extracting Matrix...');
mat_raw = ts1.data;
disp('Zeroing Start Time...');
initial_time(1:size(mat_raw,1),1) = mat_raw (1,1) + mat_raw(1,2)*1.0e-9;
mat_result= horzcat (mat_raw (:,1) + mat_raw(:,2)*1.0e-9 - initial_time(:,1), mat_raw(:,4), mat_raw(:,5));

mat_result_div4 = mat_result(1:4:end,:);

disp('Plotting...');

figure
T = mat_result_div4(:,1);
Mx = mat_result_div4(:,2);
Mz = mat_result_div4(:,3);
plot(T,Mx,T,Mz);
