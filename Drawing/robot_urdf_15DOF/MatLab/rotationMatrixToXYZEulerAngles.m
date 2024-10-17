function euler = rotationMatrixToXYZEulerAngles(R)
    % 初始化欧拉角向量为零
    euler = zeros(3, 1);
    
    % 计算 y 轴的欧拉角 (-pi/2 到 pi/2)
    euler(2) = asin(R(1, 3));
    
    % 计算 z 轴的欧拉角 (-pi 到 pi)
    sinz = -R(1, 2) / cos(euler(2));
    cosz = R(1, 1) / cos(euler(2));
    euler(3) = atan2(sinz, cosz);
    
    % 计算 x 轴的欧拉角 (-pi 到 pi)
    sinx = -R(2, 3) / cos(euler(2));
    cosx = R(3, 3) / cos(euler(2));
    euler(1) = atan2(sinx, cosx);

    euler = euler';
end