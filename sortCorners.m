%% ====================== 辅助函数：排序角点 ======================

function sorted_corners = sortCorners(corners)
    % 排序角点为：左上、右上、右下、左下
    % corners: 4x2矩阵，包含4个点的(x,y)坐标
    
    % 计算中心点
    center = mean(corners);
    
    % 计算每个点相对于中心的角度
    angles = atan2(corners(:,2) - center(2), corners(:,1) - center(1));
    
    % 将角度转换为度并排序
    [~, idx] = sort(angles);
    
    % 按角度排序的点
    sorted_by_angle = corners(idx, :);
    
    % 我们需要确保顺序是左上、右上、右下、左下
    % 找到最左上的点作为左上角
    sums = sum(sorted_by_angle, 2);
    [~, top_left_idx] = min(sums);
    
    % 重新排列点
    sorted_corners = circshift(sorted_by_angle, 1-top_left_idx);
    
    % 现在我们需要确保是逆时针顺序
    % 计算叉积来判断方向
    v1 = sorted_corners(2,:) - sorted_corners(1,:);
    v2 = sorted_corners(3,:) - sorted_corners(2,:);
    cross_product = v1(1)*v2(2) - v1(2)*v2(1);
    
    % 如果叉积为负，说明是顺时针，需要反转
    if cross_product < 0
        sorted_corners = sorted_corners([1,4,3,2], :);
    end
end