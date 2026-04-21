clc; clear; close all;

%% 载入并预处理图像---倾斜校正
I_original = imread("倾斜车牌\5.png");  
I = I_original;
if size(I,1)>2000 || size(I,2)>2000, I = imresize(I,0.5); I_original = I; end

%% 车牌分割
rgb = reshape(double(I), [], 3);
rgb(rgb(:,1)+rgb(:,2)+rgb(:,3)<600,:) = 0;
rgb(rgb(:,1)./rgb(:,2)>1.1,:) = 0;
rgb(rgb(:,1)./rgb(:,3)>1.08 & rgb(:,3)<200,:) = 0;
img = reshape(uint8(rgb), size(I));

%% 提取车牌区域
bw = img(:,:,1)>0;
bw_filled = imfill(bw, 'holes');

% 剔除边缘并保留最大区域
boundary_threshold = 10;
[L, num] = bwlabel(bw_filled);
stats = regionprops(L, 'BoundingBox');
keep_mask = false(size(bw_filled));

for i = 1:num
    bbox = stats(i).BoundingBox;
    if bbox(1) > boundary_threshold && bbox(1)+bbox(3) < size(bw_filled,2)-boundary_threshold && ...
       bbox(2) > boundary_threshold && bbox(2)+bbox(4) < size(bw_filled,1)-boundary_threshold
        keep_mask(L == i) = true;
    end
end

bw_clean = bw_filled & keep_mask;
[L_clean, num_clean] = bwlabel(bw_clean);
if num_clean > 0
    [~, max_idx] = max([regionprops(L_clean, 'Area').Area]);
    bw_largest = L_clean == max_idx;
else
    bw_largest = bw_clean;
end

%% 保存最初的车牌掩膜图像（在旋转校正前）
bw_plate_initial = imclose(bw_largest, strel('disk', 50));

%% ===== 新增：基于倾斜角度判断是否需要校正 =====
fprintf('=== 车牌倾斜角度分析 ===\n');
bw_plate = bw_plate_initial; % 从初始掩膜开始

% 获取车牌区域的倾斜角度
stats_plate_angle = regionprops(bw_plate, 'Orientation');
if ~isempty(stats_plate_angle)
    plate_angle = stats_plate_angle.Orientation;
    fprintf('检测到车牌倾斜角度: %.2f度\n', plate_angle);
    
    % === 关键优化：判断倾斜角度是否合理 ===
    if plate_angle > 0
        fprintf('角度过大(%.1f°)，可能是180度倒置，进行180度旋转\n', plate_angle);
        
        % 旋转180度
        bw_plate = imrotate(bw_plate, 180, 'bilinear', 'crop') > 0.5;
        I = imrotate(I, 180, 'bilinear', 'crop');
        
        % 重新获取角度
        stats_plate_angle = regionprops(bw_plate, 'Orientation');
        if ~isempty(stats_plate_angle)
            plate_angle = stats_plate_angle.Orientation;
            fprintf('旋转180度后角度: %.2f度\n', plate_angle);
        end
    end
    
    % 检查角度是否在合理范围内，如果不在，进行适当旋转
    if abs(plate_angle) > 45
        fprintf('角度(%.1f°)超出合理范围，进行校正\n', plate_angle);
        
        % 计算需要校正的角度
        if plate_angle > 0
            correction_angle = plate_angle - 45;
        else
            correction_angle = plate_angle + 45;
        end
        
        % 应用旋转
        bw_plate = imrotate(bw_plate, -correction_angle, 'bilinear', 'crop') > 0.5;
        I = imrotate(I, -correction_angle, 'bilinear', 'crop');
        
        fprintf('校正角度: %.2f度\n', -correction_angle);
    end
end

%% ===== 优化：车牌四点透视校正（确保宽度>高度） =====
fprintf('=== 车牌角点检测开始 ===\n');

% 获取边界
[B, L] = bwboundaries(bw_plate, 'noholes');
if isempty(B)
    fprintf('未能提取车牌边界\n');
    corners = [];
else
    boundary = B{1};
    
    % 简化边界（减少点数）
    if size(boundary, 1) > 100
        step = floor(size(boundary, 1) / 50);
        simplified_boundary = boundary(1:step:end, :);
    else
        simplified_boundary = boundary;
    end
    
    % 提取x,y坐标
    x_points = simplified_boundary(:,2);  % 列坐标
    y_points = simplified_boundary(:,1);  % 行坐标
    
    % 使用霍夫变换检测直线
    [h, w] = size(bw_plate);
    edge_img = edge(bw_plate, 'canny', [0.1 0.2], 1);
    
    % 霍夫变换检测直线
    [H, theta, rho] = hough(edge_img, 'Theta', -90:0.5:89.5);
    peaks = houghpeaks(H, 10, 'threshold', ceil(0.3*max(H(:))));
    lines = houghlines(edge_img, theta, rho, peaks, 'FillGap', 50, 'MinLength', 30);
    
    if ~isempty(lines)
        % 收集所有直线的端点
        all_points = [];
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            all_points = [all_points; xy];
        end
        
        % 聚类直线方向，找到主要的两个方向
        angles = [lines.theta];
        
        % 将角度归一化到[-45, 45]度范围
        normalized_angles = mod(angles + 90, 180) - 90;
        
        % 使用k-means聚类找到两个主要方向
        if length(normalized_angles) >= 2
            [idx, centers] = kmeans(normalized_angles', 2);
            
            % 获取每个方向的直线
            dir1_lines = lines(idx == 1);
            dir2_lines = lines(idx == 2);
            
            % 计算每个方向的平均角度
            avg_angle1 = mean([dir1_lines.theta]);
            avg_angle2 = mean([dir2_lines.theta]);
            
            fprintf('检测到两个主要方向: %.1f度和%.1f度\n', avg_angle1, avg_angle2);
            
            % 寻找四个边：每条直线与其他方向直线的交点
            corners = [];
            
            % 对每个方向的直线按截距排序
            if length(dir1_lines) >= 2 && length(dir2_lines) >= 2
                % 对dir1方向的直线按截距排序
                lines1_sorted = sortLinesByIntercept(dir1_lines);
                % 取第一个和最后一个作为上下边界
                line1_1 = lines1_sorted(1);
                line1_2 = lines1_sorted(end);
                
                % 对dir2方向的直线按截距排序
                lines2_sorted = sortLinesByIntercept(dir2_lines);
                % 取第一个和最后一个作为左右边界
                line2_1 = lines2_sorted(1);
                line2_2 = lines2_sorted(end);
                
                % 计算四个交点
                corners = zeros(4, 2);
                
                % 交点1: line1_1 和 line2_1
                corners(1,:) = lineIntersection(line1_1, line2_1);
                
                % 交点2: line1_1 和 line2_2
                corners(2,:) = lineIntersection(line1_1, line2_2);
                
                % 交点3: line1_2 和 line2_2
                corners(3,:) = lineIntersection(line1_2, line2_2);
                
                % 交点4: line1_2 和 line2_1
                corners(4,:) = lineIntersection(line1_2, line2_1);
                
                fprintf('通过直线交点计算得到角点\n');
            else
                % 直线数量不足，使用边界极值点
                corners = getExtremePoints(boundary);
                fprintf('直线数量不足，使用边界极值点\n');
            end
        else
            % 直线太少，使用边界极值点
            corners = getExtremePoints(boundary);
            fprintf('直线数量不足，使用边界极值点\n');
        end
    else
        % 没有检测到直线，使用边界极值点
        corners = getExtremePoints(boundary);
        fprintf('未检测到直线，使用边界极值点\n');
    end
end

%% 如果角点检测失败，使用最小外接矩形
min_bbox = [];
if isempty(corners) || size(corners, 1) < 4
    fprintf('角点检测失败，使用最小外接矩形\n');
    
    % 获取区域属性
    stats_temp = regionprops(bw_plate, 'BoundingBox', 'Orientation');
    if ~isempty(stats_temp)
        bbox = stats_temp.BoundingBox;
        angle = stats_temp.Orientation;
        
        fprintf('使用最小外接矩形，角度: %.2f度\n', angle);
        
        % === 优化：检查角度是否合理 ===
        if abs(angle) > 80
            fprintf('外接矩形角度过大(%.1f°)，可能是180度倒置\n', angle);
        end
        
        % 创建旋转矩形
        center = [bbox(1)+bbox(3)/2, bbox(2)+bbox(4)/2];
        width = bbox(3);
        height = bbox(4);
        min_bbox = [center(1)-width/2, center(2)-height/2, width, height, angle];
        
        % 计算旋转后的四个角点
        theta = deg2rad(-angle);  % 注意方向
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        
        corners = zeros(4, 2);
        % 相对于中心的四个角点
        local_corners = [-width/2, -height/2;
                          width/2, -height/2;
                          width/2, height/2;
                         -width/2, height/2];
        
        for i = 1:4
            rotated = R * local_corners(i,:)';
            corners(i,:) = center + rotated';
        end
    else
        % 最后的手段：使用图像的四个角点
        [h, w] = size(bw_plate);
        [y, x] = find(bw_plate);
        if ~isempty(x)
            min_x = min(x); max_x = max(x);
            min_y = min(y); max_y = max(y);
            corners = [min_x, min_y;
                      max_x, min_y;
                      max_x, max_y;
                      min_x, max_y];
        else
            corners = [1, 1; w, 1; w, h; 1, h];
        end
    end
end

%% 确保有4个角点并按顺序排列
if size(corners, 1) >= 4
    % 取前4个点
    if size(corners, 1) > 4
        % 计算中心点
        center = mean(corners);
        % 计算每个点到中心的距离
        distances = sqrt((corners(:,1)-center(1)).^2 + (corners(:,2)-center(2)).^2);
        % 取距离最大的4个点
        [~, idx] = sort(distances, 'descend');
        corners = corners(idx(1:4), :);
    end
    
    % 排序角点：[左上, 右上, 右下, 左下]
    sorted_corners = sortRectangleCorners(corners);
else
    fprintf('错误：无法获得4个角点\n');
    sorted_corners = [];
end

%% 透视校正（确保宽度 > 高度）
I_perspective_corrected = I; % 保存透视校正后的图像
if ~isempty(sorted_corners) && size(sorted_corners, 1) == 4
    fprintf('\n=== 开始透视校正 ===\n');
    
    % 计算当前四边形的宽度和高度
    width_top = norm(sorted_corners(2,:) - sorted_corners(1,:));
    width_bottom = norm(sorted_corners(3,:) - sorted_corners(4,:));
    avg_width = (width_top + width_bottom) / 2;
    
    height_left = norm(sorted_corners(4,:) - sorted_corners(1,:));
    height_right = norm(sorted_corners(3,:) - sorted_corners(2,:));
    avg_height = (height_left + height_right) / 2;
    
    % === 关键修复：确保宽度 > 高度 ===
    if avg_height > avg_width
        fprintf('检测到高度(%.1f) > 宽度(%.1f)，重新排序角点\n', avg_height, avg_width);
        
        % 重新排序角点，交换宽高
        temp = sorted_corners;
        sorted_corners = [temp(2,:); temp(3,:); temp(4,:); temp(1,:)];
        
        % 重新计算宽高
        width_top = norm(sorted_corners(2,:) - sorted_corners(1,:));
        width_bottom = norm(sorted_corners(3,:) - sorted_corners(4,:));
        avg_width = (width_top + width_bottom) / 2;
        
        height_left = norm(sorted_corners(4,:) - sorted_corners(1,:));
        height_right = norm(sorted_corners(3,:) - sorted_corners(2,:));
        avg_height = (height_left + height_right) / 2;
        
        fprintf('重新排序后：宽度=%.1f，高度=%.1f\n', avg_width, avg_height);
    end
    
    original_aspect_ratio = avg_width / avg_height;
    fprintf('原始宽高比: %.3f (宽/高)\n', original_aspect_ratio);
    
    % === 关键修复：确保目标尺寸宽度 > 高度 ===
    min_aspect_ratio = 1.5;  % 最小宽高比
    if original_aspect_ratio < min_aspect_ratio
        fprintf('宽高比(%.2f)太小，调整为%.1f\n', original_aspect_ratio, min_aspect_ratio);
        original_aspect_ratio = min_aspect_ratio;
    end
    
    % 设置目标尺寸 - 固定高度，根据宽高比计算宽度
    target_height = 150;  % 固定高度
    target_width = round(target_height * original_aspect_ratio);
    
    % 确保目标宽度 > 高度
    if target_width <= target_height
        target_width = target_height * 2;  % 确保宽度是高度的至少2倍
        fprintf('调整目标尺寸：%d x %d\n', target_width, target_height);
    end
    
    fprintf('目标尺寸: %d x %d (宽/高=%.2f)\n', target_width, target_height, target_width/target_height);
    
    % 目标矩形的四个角点
    destinationPoints = [1, 1; 
                        target_width, 1; 
                        target_width, target_height; 
                        1, target_height];
    
    % 计算透视变换矩阵
    try
        tform = fitgeotrans(sorted_corners, destinationPoints, 'projective');
        
        % 应用透视变换到原始图像
        I_perspective_corrected = imwarp(I, tform, 'OutputView', imref2d([target_height, target_width]));
        
        % 应用透视变换到车牌二值图像
        bw_plate_corrected = imwarp(bw_plate, tform, 'OutputView', imref2d([target_height, target_width]));
        bw_plate_corrected = bw_plate_corrected > 0.5;
        
        % 更新图像和车牌区域
        I = I_perspective_corrected;
        bw_plate = bw_plate_corrected;
        
        fprintf('透视校正成功完成\n');
        
    catch ME
        fprintf('透视变换失败: %s\n', ME.message);
        % 如果透视变换失败，尝试简单旋转校正
        stats_temp = regionprops(bw_plate, 'Orientation');
        if ~isempty(stats_temp)
            angle = stats_temp.Orientation;
            if abs(angle) > 0.5
                fprintf('尝试旋转校正: %.2f度\n', angle);
                I = imrotate(I, -angle, 'bilinear', 'crop');
                bw_plate = imrotate(bw_plate, -angle, 'bilinear', 'crop') > 0.5;
            end
        end
    end
else
    fprintf('无法进行透视校正，使用原始图像\n');
    I_perspective_corrected = I;
end

%% 继续原处理流程
fprintf('\n=== 继续字符分割流程 ===\n');

% 获取校正后的车牌彩色图像
I_char = uint8(double(I).*bw_plate);

%% 提取并缩放车牌
stats_plate = regionprops(bw_plate, 'BoundingBox');
if ~isempty(stats_plate)
    bbox = stats_plate.BoundingBox;
    x1 = max(1, floor(bbox(1))); y1 = max(1, floor(bbox(2)));
    x2 = min(size(I_char,2), ceil(bbox(1)+bbox(3)));
    y2 = min(size(I_char,1), ceil(bbox(2)+bbox(4)));
    
    plate_cropped = I_char(y1:y2, x1:x2, :);
    
    % 如果裁剪区域太小，使用整个车牌区域
    if size(plate_cropped,1) < 10 || size(plate_cropped,2) < 10
        plate_cropped = I_char;
    end
    
    gray_cropped = rgb2gray(plate_cropped);
    [row_indices, col_indices] = find(gray_cropped > 0);
    
    if ~isempty(row_indices)
        min_row = max(1, min(row_indices)-2); 
        max_row = min(size(plate_cropped,1), max(row_indices)+2);
        min_col = max(1, min(col_indices)-2); 
        max_col = min(size(plate_cropped,2), max(col_indices)+2);
        
        % 确保裁剪区域有效
        if min_row < max_row && min_col < max_col
            plate_content = plate_cropped(min_row:max_row, min_col:max_col, :);
        else
            plate_content = plate_cropped;
        end
        
        [h, w, ~] = size(plate_content);
        
        % 计算当前车牌内容的宽高比
        content_ratio = w / h;
        
        % 标准车牌比例约3.14:1，我们以此为目标
        standard_ratio = 3.14;
        
        if content_ratio > 1.5  % 宽高比合理（宽度明显大于高度）
            % 使用标准比例
            if content_ratio > standard_ratio
                % 当前比标准更宽，以高度为基准
                target_h = 200;
                target_w = round(target_h * standard_ratio);
            else
                % 当前没有标准宽，以宽度为基准
                target_w = 400;
                target_h = round(target_w / standard_ratio);
            end
        else
            % 宽高比异常，强制使用标准比例
            target_w = 400;
            target_h = round(target_w / standard_ratio);
            fprintf('车牌宽高比异常(%.2f)，使用标准比例(3.14:1)\n', content_ratio);
        end
        
        % 计算缩放比例
        scale_h = target_h / h;
        scale_w = target_w / w;
        scale = min(scale_h, scale_w);
        
        plate_resized = imresize(plate_content, scale);
        
        final_plate = uint8(zeros(target_h, target_w, 3));
        [new_h, new_w, ~] = size(plate_resized);
        sr = floor((target_h-new_h)/2)+1; 
        sc = floor((target_w-new_w)/2)+1;
        
        % 确保索引有效
        if sr > 0 && sc > 0 && sr+new_h-1 <= target_h && sc+new_w-1 <= target_w
            final_plate(sr:sr+new_h-1, sc:sc+new_w-1, :) = plate_resized;
        else
            % 如果放置位置有问题，直接使用缩放图像
            final_plate = plate_resized;
        end
    else
        final_plate = uint8(zeros(200, 400, 3));
    end
else
    final_plate = uint8(zeros(200, 400, 3));
end

final_plate = imresize(I_char,[200, 400]);

%% 车牌二值化
gray_plate = rgb2gray(final_plate);
threshold = graythresh(gray_plate);
plate_binary = ~imbinarize(gray_plate, threshold-0.06);  

% 删除边界连通区域
[L_bw, num_bw] = bwlabel(plate_binary);
stats_bw = regionprops(L_bw, 'BoundingBox');
delete_mask = false(size(plate_binary));
[h, w] = size(plate_binary);

for i = 1:num_bw
    bbox = stats_bw(i).BoundingBox;
    near_edge = bbox(1) <= 25 || bbox(1)+bbox(3) >= w-25 || ...
                bbox(2) <= 25 || bbox(2)+bbox(4) >= h-25;
    
    if near_edge
        aspect_ratio = bbox(3) / bbox(4);
        if aspect_ratio > 3 || aspect_ratio < 0.23 || ...
           bbox(3)*bbox(4) < 100 || ...
           (bbox(1) <= 25 && bbox(2) <= 25) || ...
           (bbox(1)+bbox(3) >= w-25 && bbox(2) <= 25)
            delete_mask(L_bw == i) = true;
        end
    end
end

plate_bw_clean = plate_binary;
plate_bw_clean(delete_mask) = false;

%% 矩形区域置0
left_rect = [0, 0, 125, 54];
right_rect = [280, 0, w, 90];
rect_mask = false(h, w);

rect_mask(1:left_rect(4), 1:left_rect(3)) = true;
rect_mask(1:right_rect(4), right_rect(1):right_rect(3)) = true;

plate_bw_final = plate_bw_clean;
plate_bw_final(rect_mask) = false;
plate_bw2 = bwareaopen(plate_bw_final, 150);

%% 顶部0-30像素矩形区域置0
top_rect_mask = false(size(plate_bw2));
top_height = 25;
top_rect_mask(1:top_height, :) = true;

plate_bw3 = plate_bw2;
plate_bw3(top_rect_mask) = false;

%% 闭运算
plate_bw4 = imclose(plate_bw3, strel('disk', 3));
plate_bw5 = bwareaopen(plate_bw4, 200);

%% 字符切割
[L_final, num_final] = bwlabel(plate_bw5);
stats_final = regionprops(L_final, 'BoundingBox', 'Area', 'Centroid');

% 获取所有连通区域的中心坐标
centroids = zeros(num_final, 2);
bounding_boxes = zeros(num_final, 4);
for i = 1:num_final
    centroids(i, :) = stats_final(i).Centroid;
    bounding_boxes(i, :) = stats_final(i).BoundingBox;
end

% 根据y坐标将字符分为上下两行
y_coords = centroids(:, 2);
y_mean = mean(y_coords);

% 区分上下行
top_indices = find(y_coords < y_mean);    % 上部字符（地区字符）
bottom_indices = find(y_coords >= y_mean); % 下部字符（字母和数字）

% 对每行字符按x坐标排序（从左到右）
[~, top_sorted] = sort(centroids(top_indices, 1));
[~, bottom_sorted] = sort(centroids(bottom_indices, 1));

% 组合字符顺序：先上排从左到右，再下排从左到右
all_sorted_indices = [top_indices(top_sorted); bottom_indices(bottom_sorted)];

% 切割字符
characters = cell(num_final, 1);
char_bboxes = zeros(num_final, 4);
char_positions = cell(num_final, 1); % 记录字符位置

for i = 1:num_final
    idx = all_sorted_indices(i);
    bbox = stats_final(idx).BoundingBox;
    
    % 判断字符位置
    if ismember(idx, top_indices)
        char_positions{i} = 'top';
    else
        char_positions{i} = 'bottom';
    end
    
    % 切割连通区域（加2像素边界）
    x1 = max(1, floor(bbox(1)) - 2);
    y1 = max(1, floor(bbox(2)) - 2);
    x2 = min(w, ceil(bbox(1) + bbox(3)) + 2);
    y2 = min(h, ceil(bbox(2) + bbox(4)) + 2);
    
    % 提取字符区域
    char_region = plate_bw5(y1:y2, x1:x2);
    
    % 裁剪到最小边界框
    [rows, cols] = find(char_region);
    if ~isempty(rows) && ~isempty(cols)
        min_row = max(1, min(rows) - 1);
        max_row = min(size(char_region,1), max(rows) + 1);
        min_col = max(1, min(cols) - 1);
        max_col = min(size(char_region,2), max(cols) + 1);
        
        char_cropped = char_region(min_row:max_row, min_col:max_col);
    else
        char_cropped = char_region;
    end
    
    % 统一缩放到32x32
    char_resized = imresize(char_cropped, [32, 32]);
    % imwrite(char_resized, ['font\', num2str(i*10003),'.jpg'])
    characters{i} = char_resized;
    char_bboxes(i, :) = [x1, y1, x2, y2];
end

%% ============================================
%% 字符匹配（直接匹配最高相似度）
fprintf('=== 字符匹配开始 ===\n');

% 加载字符库
template_dir = 'font';
if ~exist(template_dir, 'dir')
    error('字符库文件夹不存在: %s', template_dir);
end

% 获取所有子文件夹（字符类别）
char_folders = dir(template_dir);
char_folders = char_folders([char_folders.isdir] & ~ismember({char_folders.name}, {'.', '..'}));

% 加载模板图像
template_chars = struct('name', {}, 'images', {});
for i = 1:length(char_folders)
    folder_name = char_folders(i).name;
    folder_path = fullfile(template_dir, folder_name);
    image_files = dir(fullfile(folder_path, '*.jpg'));
    
    templates = cell(length(image_files), 1);
    for j = 1:length(image_files)
        img_path = fullfile(folder_path, image_files(j).name);
        template_img = imread(img_path);
        if size(template_img, 3) == 3
            template_img = rgb2gray(template_img);
        end
        template_img = logical(template_img);
        template_img = imresize(template_img, [32, 32]);
        templates{j} = template_img;
    end
    
    template_chars(i).name = folder_name;
    template_chars(i).images = templates;
end

% 匹配字符 - 直接匹配最高相似度
recognized_chars = cell(num_final, 1);
scores = zeros(num_final, 1);

for i = 1:num_final
    char_img = characters{i};
    best_score = -inf;
    best_char = '?';
    
    % 遍历所有字符类别
    for j = 1:length(template_chars)
        templates = template_chars(j).images;
        folder_name = template_chars(j).name;
        
        % 遍历当前类别的所有模板
        for k = 1:length(templates)
            template = templates{k};
            
            % 计算相似度（使用相关系数）
            corr_score = corr2(char_img(:), template(:));
            
            % 直接选择最高相似度
            if corr_score > best_score
                best_score = corr_score;
                best_char = folder_name;
            end
        end
    end
    
    recognized_chars{i} = best_char;
    scores(i) = best_score;
end

% 生成车牌号码
top_chars = {};
bottom_chars = {};

for i = 1:num_final
    if strcmp(char_positions{i}, 'top')
        top_chars{end+1} = recognized_chars{i};
    else
        bottom_chars{end+1} = recognized_chars{i};
    end
end

if ~isempty(top_chars)
    top_str = strjoin(top_chars, '');
else
    top_str = '';
end

if ~isempty(bottom_chars)
    bottom_str = strjoin(bottom_chars, '');
else
    bottom_str = '';
end

license_plate = [top_str, ' ', bottom_str];

%% 显示主要处理结果图像
figure('Position', [50, 50, 1400, 600]);
set(gcf, 'Name', '车牌识别处理过程', 'NumberTitle', 'off');

% 图1：显示最开始的原始图像
subplot(2, 3, 1);
imshow(I_original);
title('1. 原始图像', 'FontSize', 12, 'FontWeight', 'bold');

% 图2：显示最后二值图（不是车牌的）
subplot(2, 3, 2);
imshow(bw);
title('2. 最后二值图（示例）', 'FontSize', 12, 'FontWeight', 'bold');

% 图3：显示最开始图像车牌区域的掩膜图像
subplot(2, 3, 3);
imshow(I_original);
hold on;
% 创建掩膜覆盖层
mask_overlay = zeros(size(I_original));
mask_overlay(:,:,1) = 255 * uint8(bw_plate_initial); % 红色通道
mask_overlay(:,:,3) = 128 * uint8(bw_plate_initial); % 蓝色通道
h_mask = imshow(mask_overlay);
set(h_mask, 'AlphaData', 0.3 * bw_plate_initial);
title('3. 车牌区域掩膜', 'FontSize', 12, 'FontWeight', 'bold');
hold off;

% 图4：在最开始原始图上显示透视校正图像，最小外接矩形
subplot(2, 3, 4);
imshow(final_plate);
title('4. 透视校正与增强', 'FontSize', 12, 'FontWeight', 'bold');

% 图5：显示车牌二值图
subplot(2, 3, 5);
imshow(plate_binary);
hold on;
% 标记字符区域
for i = 1:num_final
    idx = all_sorted_indices(i);
    bbox = stats_final(idx).BoundingBox;
    if ismember(idx, top_indices)
        color = 'r';
    else
        color = 'g';
    end
    rectangle('Position', bbox, 'EdgeColor', color, 'LineWidth', 1);
end
title(sprintf('5. 车牌二值图 (%d个字符)', num_final), 'FontSize', 12, 'FontWeight', 'bold');
hold off;

% 图6：在最开始原始图像上标记显示检测位置（移除文字显示）
subplot(2, 3, 6);
imshow(I_original);
hold on;

% 如果检测到车牌区域，绘制边界框
if ~isempty(bw_plate_initial)
    stats_initial = regionprops(bw_plate_initial, 'BoundingBox');
    if ~isempty(stats_initial)
        bbox_initial = stats_initial(1).BoundingBox;
        rectangle('Position', bbox_initial, 'EdgeColor', 'g', 'LineWidth', 3);
        % 已移除图像上的文字显示
    end
end

title('6. 最终识别位置 (结果请看弹窗)', 'FontSize', 12, 'FontWeight', 'bold');
hold off;

%% 输出识别结果
fprintf('\n===== 车牌识别结果 =====\n');
fprintf('车牌号码: %s\n', license_plate);
fprintf('检测到的字符数: %d\n', num_final);
fprintf('上排字符数: %d\n', length(top_indices));
fprintf('下排字符数: %d\n', length(bottom_indices));

msg_content = {
    '【倾斜车牌识别结果】';
    '-----------------------------';
    ['车牌号码: ', license_plate];
    ['总检测字符: ', num2str(num_final)];
    ['上排字符数: ', num2str(length(top_chars))];
    ['下排字符数: ', num2str(length(bottom_chars))];
    '-----------------------------';
    '已完成透视校正与识别'
};
msgbox(msg_content, '识别完成');

%% ===== 辅助函数定义 =====
function sorted = sortLinesByIntercept(lines)
    % 按截距对直线进行排序
    if isempty(lines)
        sorted = lines;
        return;
    end
    
    intercepts = zeros(length(lines), 1);
    for i = 1:length(lines)
        x1 = lines(i).point1(1); y1 = lines(i).point1(2);
        x2 = lines(i).point2(1); y2 = lines(i).point2(2);
        
        if x2 ~= x1
            m = (y2 - y1) / (x2 - x1);
            b = y1 - m * x1;
            intercepts(i) = b;
        else
            intercepts(i) = y1;
        end
    end
    
    [~, idx] = sort(intercepts);
    sorted = lines(idx);
end

function intersection = lineIntersection(line1, line2)
    % 计算两条直线的交点
    x1 = line1.point1(1); y1 = line1.point1(2);
    x2 = line1.point2(1); y2 = line1.point2(2);
    
    x3 = line2.point1(1); y3 = line2.point1(2);
    x4 = line2.point2(1); y4 = line2.point2(2);
    
    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    
    if abs(denom) < 1e-10
        intersection = [(x1+x3)/2, (y1+y3)/2];
    else
        px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom;
        py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom;
        intersection = [px, py];
    end
end

function extreme_points = getExtremePoints(boundary)
    % 获取边界的四个极值点
    x = boundary(:,2);
    y = boundary(:,1);
    
    % 计算凸包
    k = convhull(x, y);
    hull_x = x(k);
    hull_y = y(k);
    
    % 在凸包上找极值点
    [~, top_idx] = min(hull_y);
    [~, bottom_idx] = max(hull_y);
    [~, left_idx] = min(hull_x);
    [~, right_idx] = max(hull_x);
    
    top_idx = min(max(top_idx, 1), length(hull_x));
    bottom_idx = min(max(bottom_idx, 1), length(hull_x));
    left_idx = min(max(left_idx, 1), length(hull_x));
    right_idx = min(max(right_idx, 1), length(hull_x));
    
    extreme_points = [hull_x(top_idx), hull_y(top_idx);
                      hull_x(right_idx), hull_y(right_idx);
                      hull_x(bottom_idx), hull_y(bottom_idx);
                      hull_x(left_idx), hull_y(left_idx)];
end

function sorted_corners = sortRectangleCorners(corners)
    % 对矩形的四个角点进行排序：[左上, 右上, 右下, 左下]
    if size(corners, 1) ~= 4
        sorted_corners = corners;
        return;
    end
    
    % 计算中心点
    center = mean(corners);
    
    % 将角点转换为相对于中心点的向量
    vectors = corners - center;
    
    % 计算每个向量的角度
    angles = atan2(vectors(:,2), vectors(:,1));
    
    % 将角度转换为0到360度范围
    angles_deg = mod(rad2deg(angles), 360);
    
    % 按角度排序
    [~, idx] = sort(angles_deg);
    sorted = corners(idx, :);
    
    % 找到左上角（y值最小的点）
    [~, min_y_idx] = min(sorted(:,2));
    
    % 重新排序，使左上角成为第一个点
    sorted_corners = circshift(sorted, 1-min_y_idx, 1);
    
    % 确保顺序是：左上->右上->右下->左下
    if sorted_corners(2,1) < sorted_corners(1,1)
        temp = sorted_corners(1,:);
        sorted_corners(1,:) = sorted_corners(2,:);
        sorted_corners(2,:) = temp;
    end
    
    if sorted_corners(4,1) > sorted_corners(3,1)
        temp = sorted_corners(3,:);
        sorted_corners(3,:) = sorted_corners(4,:);
        sorted_corners(4,:) = temp;
    end
end