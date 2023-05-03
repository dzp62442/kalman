load('data.csv');

figure;
plot(data(:,[1,4,7]),data(:,[2,5,8]), 'LineWidth', 1);
title('位置预测结果');
legend({'位置真值', '直接计算', 'EKF'});
xlabel('x位置坐标');
ylabel('y位置坐标');

figure;
predict_error = sqrt((data(:,1)-data(:,4)).^2 + (data(:,2)-data(:,5)).^2);
ekf_error = sqrt((data(:,1)-data(:,7)).^2 + (data(:,2)-data(:,8)).^2);
plot([predict_error, ekf_error], 'LineWidth', 1);
title('位置预测误差（欧氏距离）');
legend({'直接计算误差', 'EKF误差'});
xlabel('迭代次数');
ylabel('欧氏距离误差');

figure;
plot(data(:,[3,6,9]), 'LineWidth', 1);
title('朝向角度预测结果');
legend({'角度真值', '直接计算', 'EKF'});
xlabel('迭代次数');
ylabel('朝向角度');

figure;
predict_err = sqrt((data(:,3)-data(:,6)).^2);
ekf_err = sqrt((data(:,3)-data(:,9)).^2);
plot([predict_err, ekf_err], 'LineWidth', 1);
title('朝向角预测误差（角度）');
legend({'直接计算误差', 'EKF误差'});
xlabel('迭代次数');
ylabel('角度误差');