load('data.csv');

figure;
plot(data(:,[1,4,7]),data(:,[2,5,8]), 'LineWidth', 1);
title('λ��Ԥ����');
legend({'λ����ֵ', 'ֱ�Ӽ���', 'EKF'});
xlabel('xλ������');
ylabel('yλ������');

figure;
predict_error = sqrt((data(:,1)-data(:,4)).^2 + (data(:,2)-data(:,5)).^2);
ekf_error = sqrt((data(:,1)-data(:,7)).^2 + (data(:,2)-data(:,8)).^2);
plot([predict_error, ekf_error], 'LineWidth', 1);
title('λ��Ԥ����ŷ�Ͼ��룩');
legend({'ֱ�Ӽ������', 'EKF���'});
xlabel('��������');
ylabel('ŷ�Ͼ������');

figure;
plot(data(:,[3,6,9]), 'LineWidth', 1);
title('����Ƕ�Ԥ����');
legend({'�Ƕ���ֵ', 'ֱ�Ӽ���', 'EKF'});
xlabel('��������');
ylabel('����Ƕ�');

figure;
predict_err = sqrt((data(:,3)-data(:,6)).^2);
ekf_err = sqrt((data(:,3)-data(:,9)).^2);
plot([predict_err, ekf_err], 'LineWidth', 1);
title('�����Ԥ�����Ƕȣ�');
legend({'ֱ�Ӽ������', 'EKF���'});
xlabel('��������');
ylabel('�Ƕ����');