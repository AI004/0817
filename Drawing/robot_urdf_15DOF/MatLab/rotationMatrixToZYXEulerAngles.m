function eulerAngles = rotationMatrixToZYXEulerAngles(rotationMatrix)
    % 检查旋转矩阵是否有效
    if abs(det(rotationMatrix) - 1) > 0.01
        error('Invalid rotation matrix: determinant is not close to 1.');
    end

    % 提取欧拉角
    sy = sqrt(rotationMatrix(1, 1)^2 + rotationMatrix(2, 1)^2);

    singular = sy < 1e-6; % 如果 sy 太小，则表示奇异情况

    if ~singular
        roll = atan2(rotationMatrix(3, 2), rotationMatrix(3, 3));
        pitch = atan2(-rotationMatrix(3, 1), sy);
        yaw = atan2(rotationMatrix(2, 1), rotationMatrix(1, 1));
    else
        roll = atan2(-rotationMatrix(2, 3), rotationMatrix(2, 2));
        pitch = atan2(-rotationMatrix(3, 1), sy);
        yaw = 0;
    end

    eulerAngles = [yaw, pitch, roll];
end