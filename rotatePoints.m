%% 辅助函数：旋转点集
function rotated_points = rotatePoints(points, angle)
    % 旋转点集
    % points: [x, y] 坐标矩阵
    % angle: 旋转角度（弧度）
    
    % 创建旋转矩阵
    R = [cos(angle), -sin(angle); 
         sin(angle), cos(angle)];
    
    % 旋转点集
    rotated_points = points * R';
end