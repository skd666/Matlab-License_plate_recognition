clc; clear; close all;

%% 载入并预处理图像--文字干扰
I = imread("干扰车牌\1.jpg"); 
if size(I,1)>2000 || size(I,2)>2000, I = imresize(I,0.5); end
figure
imshow(I)
title('原始图像')

%% 车牌分割
rgb = reshape(double(I), [], 3);
rgb(rgb(:,1)+rgb(:,2)+rgb(:,3)<550,:) = 0;
img = reshape(uint8(rgb), size(I));
figure
imshow(img)
title('车牌区域分割')

%% 提取车牌区域
bw = img(:,:,1)>0;
bw_filled = imfill(bw, 'holes');

% 剔除边缘并保留最大区域
boundary_threshold = 30;
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

%% ========== 使用最小外接矩形获取车牌区域 ==========
% 获取最大区域的最小外接矩形
stats_largest = regionprops(bw_largest, 'BoundingBox', 'Orientation', 'Image');

if ~isempty(stats_largest)
    % 获取最小外接矩形的信息
    bbox = stats_largest.BoundingBox;
    orientation = stats_largest.Orientation;
    
    % 获取区域的像素坐标
    [rows, cols] = find(bw_largest);
    points = [cols, rows];  % 注意：x对应列，y对应行
    
    % 计算凸包
    if size(points, 1) > 2
        k = convhull(points(:,1), points(:,2));
        hull_points = points(k, :);
        
        % 计算最小面积矩形（旋转卡壳法简化版）
        min_area = inf;
        min_rect = [];
        
        for i = 1:length(hull_points)-1
            % 获取当前边
            p1 = hull_points(i, :);
            p2 = hull_points(i+1, :);
            
            % 计算边的方向向量
            edge_vec = p2 - p1;
            edge_angle = atan2(edge_vec(2), edge_vec(1));
            
            % 旋转所有点使该边水平
            rot_points = rotatePoints(points, -edge_angle);
            
            % 计算旋转后点的边界框
            min_x = min(rot_points(:,1));
            max_x = max(rot_points(:,1));
            min_y = min(rot_points(:,2));
            max_y = max(rot_points(:,2));
            
            % 计算面积
            width = max_x - min_x;
            height = max_y - min_y;
            area = width * height;
            
            % 更新最小矩形
            if area < min_area
                min_area = area;
                
                % 计算矩形中心
                center_x = (min_x + max_x) / 2;
                center_y = (min_y + max_y) / 2;
                
                % 旋转矩形顶点回原始坐标系
                rect_corners_rot = [min_x, min_y; max_x, min_y; 
                                   max_x, max_y; min_x, max_y];
                rect_corners = rotatePoints(rect_corners_rot, edge_angle);
                
                % 计算矩形参数
                rect_center = rotatePoints([center_x, center_y], edge_angle);
                rect_width = width;
                rect_height = height;
                rect_angle = edge_angle * 180 / pi;
                
                min_rect = struct('corners', rect_corners, 'center', rect_center, ...
                                 'width', rect_width, 'height', rect_height, 'angle', rect_angle);
            end
        end
    else
        % 如果点数太少，使用普通边界框
        min_x = min(cols);
        max_x = max(cols);
        min_y = min(rows);
        max_y = max(rows);
        rect_corners = [min_x, min_y; max_x, min_y; max_x, max_y; min_x, max_y];
        min_rect = struct('corners', rect_corners, 'center', [(min_x+max_x)/2, (min_y+max_y)/2], ...
                         'width', max_x-min_x, 'height', max_y-min_y, 'angle', 0);
    end
    
    % ========== 创建车牌区域掩膜 ==========
    % 创建与原始图像相同大小的掩膜
    plate_mask = false(size(I,1), size(I,2));
    
    % 使用多边形填充创建掩膜
    mask_poly = roipoly(plate_mask, min_rect.corners(:,1), min_rect.corners(:,2));
    
    % 应用掩膜到原始图像
    I_char = uint8(double(I) .* repmat(mask_poly, [1, 1, 3]));
    
    % 提取掩膜区域
    stats_mask = regionprops(mask_poly, 'BoundingBox');
    bbox_mask = stats_mask.BoundingBox;
    
    % 调整边界框以确保在图像范围内
    x1 = max(1, floor(bbox_mask(1)));
    y1 = max(1, floor(bbox_mask(2)));
    x2 = min(size(I_char,2), ceil(bbox_mask(1) + bbox_mask(3)));
    y2 = min(size(I_char,1), ceil(bbox_mask(2) + bbox_mask(4)));
    
    % 提取车牌区域
    plate_cropped = I_char(y1:y2, x1:x2, :);
    
    % 可视化最小外接矩形（调试用）
    figure('Name', '最小外接矩形');
    imshow(I);
    hold on;
    plot(min_rect.corners([1:end,1],1), min_rect.corners([1:end,1],2), 'r-', 'LineWidth', 2);
    plot(min_rect.center(1), min_rect.center(2), 'g+', 'MarkerSize', 10);
    title(sprintf('最小外接矩形 (角度: %.1f°)', min_rect.angle));
    hold off;
    
    % 可视化掩膜结果
    figure('Name', '掩膜提取的车牌');
    imshow(I_char);
    title('使用最小外接矩形掩膜提取的车牌区域');
    
else
    % 如果没有找到区域，使用原始方法
    bw_plate = imclose(bw_largest, strel('disk', 50));
    I_char = uint8(double(I).*repmat(bw_plate, [1, 1, 3]));
    
    stats_plate = regionprops(bw_plate, 'BoundingBox');
    if ~isempty(stats_plate)
        bbox = stats_plate.BoundingBox;
        x1 = max(1, floor(bbox(1))); y1 = max(1, floor(bbox(2)));
        x2 = min(size(I_char,2), ceil(bbox(1)+bbox(3)));
        y2 = min(size(I_char,1), ceil(bbox(2)+bbox(4)));
        plate_cropped = I_char(y1:y2, x1:x2, :);
    else
        plate_cropped = I_char;
    end
end

%% 缩放车牌到标准尺寸
gray_cropped = rgb2gray(plate_cropped);
[row_indices, col_indices] = find(gray_cropped > 0);

if ~isempty(row_indices)
    min_row = max(1, min(row_indices)-2); max_row = min(size(plate_cropped,1), max(row_indices)+2);
    min_col = max(1, min(col_indices)-2); max_col = min(size(plate_cropped,2), max(col_indices)+2);
    plate_content = plate_cropped(min_row:max_row, min_col:max_col, :);
    
    [h, w, ~] = size(plate_content);
    scale = min(200/h, 400/w);
    plate_resized = imresize(plate_content, round([h w]*scale));
    
    final_plate = uint8(zeros(200, 400, 3));
    [new_h, new_w, ~] = size(plate_resized);
    sr = floor((200-new_h)/2)+1; sc = floor((400-new_w)/2)+1;
    final_plate(sr:sr+new_h-1, sc:sc+new_w-1, :) = plate_resized;
else
    final_plate = uint8(zeros(200, 400, 3));
end

figure
imshow(final_plate)

%% ========== 后续代码保持不变（字符分割和识别部分）==========
%% 车牌二值化
rgb2 = reshape(double(final_plate), size(final_plate,1)*size(final_plate,2), size(final_plate,3));
rgb2(rgb2(:,3)>80,:) = 0;
img2 = reshape(uint8(rgb2), size(final_plate,1), size(final_plate,2), size(final_plate,3));
plate_bw = img2(:,:,1)>0;
plate_bw = imclose(plate_bw, strel('disk', 3));

%% 删除边界连通区域
[L_bw, num_bw] = bwlabel(plate_bw);
stats_bw = regionprops(L_bw, 'BoundingBox');
delete_mask = false(size(plate_bw));
[h, w] = size(plate_bw);

for i = 1:num_bw
    bbox = stats_bw(i).BoundingBox;
    near_edge = bbox(1) <= 25 || bbox(1)+bbox(3) >= w-25 || ...
                bbox(2) <= 25 || bbox(2)+bbox(4) >= h-25;
    
    if near_edge
        aspect_ratio = bbox(3) / bbox(4);
        if aspect_ratio > 3 || aspect_ratio < 0.33 || ...
           bbox(3)*bbox(4) < 100 || ...
           (bbox(1) <= 25 && bbox(2) <= 25) || ...
           (bbox(1)+bbox(3) >= w-25 && bbox(2) <= 25)
            delete_mask(L_bw == i) = true;
        end
    end
end

plate_bw_clean = plate_bw;
plate_bw_clean(delete_mask) = false;

%% 矩形区域置0
left_rect = [0, 0, 125, 35];
right_rect = [280, 0, w, 73];
rect_mask = false(h, w);

rect_mask(1:left_rect(4), 1:left_rect(3)) = true;
rect_mask(1:right_rect(4), right_rect(1):right_rect(3)) = true;

plate_bw_final = plate_bw_clean;
plate_bw_final(rect_mask) = false;
plate_bw2 = bwareaopen(plate_bw_final, 40);
plate_bw2 = imclose(plate_bw2, strel('disk', 2));

%% 顶部0-30像素矩形区域置0
top_rect_mask = false(size(plate_bw2));
top_height = 20;
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
for i = 1:num_final
    centroids(i, :) = stats_final(i).Centroid;
end

% 根据y坐标将字符分为上下两行
y_coords = centroids(:, 2);
y_mean = mean(y_coords);

% 区分上下行
top_indices = find(y_coords < y_mean);
bottom_indices = find(y_coords >= y_mean);

% 对每行字符按x坐标排序（从左到右）
[~, top_sorted] = sort(centroids(top_indices, 1));
[~, bottom_sorted] = sort(centroids(bottom_indices, 1));

% 组合字符顺序
all_sorted_indices = [top_indices(top_sorted); bottom_indices(bottom_sorted)];

% 切割字符
characters = cell(num_final, 1);
char_positions = cell(num_final, 1);

for i = 1:num_final
    idx = all_sorted_indices(i);
    bbox = stats_final(idx).BoundingBox;
    
    % 判断字符位置
    if ismember(idx, top_indices)
        char_positions{i} = 'top';
    else
        char_positions{i} = 'bottom';
    end
    
    % 切割连通区域
    x1 = max(1, floor(bbox(1)) - 2);
    y1 = max(1, floor(bbox(2)) - 2);
    x2 = min(w, ceil(bbox(1) + bbox(3)) + 2);
    y2 = min(h, ceil(bbox(2) + bbox(4)) + 2);
    
    char_region = plate_bw5(y1:y2, x1:x2);
    
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
    characters{i} = char_resized;
end

%% 字符匹配
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
    image_files = dir(fullfile(folder_path, '*.png'));
    
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

% 匹配字符 - 添加相似度阈值
recognized_chars = cell(num_final, 1);
scores = zeros(num_final, 1);
best_template_indices = zeros(num_final, 1);

% 设置相似度阈值（可以调整这个值，0.6-0.8之间通常较好）
similarity_threshold = 0.65;

for i = 1:num_final
    char_img = characters{i};
    best_score = -inf;
    best_char = '?';  % 用问号表示未识别或不可靠
    best_template_idx = 0;
    
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
                best_template_idx = k; % 记录最佳模板索引
            end
        end
    end
    
    % 检查相似度是否低于阈值
    if best_score < similarity_threshold
        recognized_chars{i} = '';  % 空字符串表示不显示该字符
    else
        recognized_chars{i} = best_char;
    end
    
    scores(i) = best_score;
    best_template_indices(i) = best_template_idx;
end

% 生成车牌号码
top_chars = {};
bottom_chars = {};

for i = 1:num_final
    % 只添加非空（有效识别）的字符
    if ~isempty(recognized_chars{i})
        if strcmp(char_positions{i}, 'top')
            top_chars{end+1} = recognized_chars{i};
        else
            bottom_chars{end+1} = recognized_chars{i};
        end
    end
end

% 生成车牌字符串
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

%% 显示精简的结果图像
figure('Name', '车牌切割图', 'Position', [100, 100, 600, 400]);
imshow(final_plate);
title('车牌切割图', 'FontSize', 14);

figure('Name', '字符分割图', 'Position', [200, 200, 600, 400]);
imshow(plate_bw5);
hold on;

% 用不同颜色标记上下行字符，只显示有效识别字符（已移除图像上的文字标签）
for i = 1:length(top_indices)
    idx = top_indices(i);
    bbox = stats_final(idx).BoundingBox;
    char_idx = find(all_sorted_indices == idx, 1);
    if ~isempty(char_idx) && ~isempty(recognized_chars{char_idx})
        rectangle('Position', bbox, 'EdgeColor', 'r', 'LineWidth', 2);
    end
end

for i = 1:length(bottom_indices)
    idx = bottom_indices(i);
    bbox = stats_final(idx).BoundingBox;
    char_idx = find(all_sorted_indices == idx, 1);
    if ~isempty(char_idx) && ~isempty(recognized_chars{char_idx})
        rectangle('Position', bbox, 'EdgeColor', 'g', 'LineWidth', 2);
    end
end

title('字符分割图 (红:上排, 绿:下排)', 'FontSize', 14);
hold off;

%% 输出识别结果（控制台）
fprintf('\n===== 车牌识别结果 =====\n');
fprintf('车牌号码: %s\n', license_plate);
fprintf('检测到的字符数: %d\n', num_final);
fprintf('有效识别字符数: %d (相似度阈值: %.2f)\n', ...
    sum(~cellfun(@isempty, recognized_chars)), similarity_threshold);
fprintf('上排有效字符: %d\n', length(top_chars));
fprintf('下排有效字符: %d\n', length(bottom_chars));

fprintf('\n字符识别详情（相似度阈值: %.2f）:\n', similarity_threshold);
for i = 1:num_final
    idx = all_sorted_indices(i);
    current_char = recognized_chars{i};
    if ~isempty(current_char)
        % 有效识别
        fprintf('字符%d: %s ✓ (位置: %s, 相似度: %.4f, 阈值: %.2f)\n', ...
            i, current_char, char_positions{i}, scores(i), similarity_threshold);
    else
        % 无效识别（低于阈值）
        fprintf('字符%d: 未识别 ✗ (位置: %s, 相似度: %.4f < 阈值: %.2f)\n', ...
            i, char_positions{i}, scores(i), similarity_threshold);
    end
end

% 显示字符库信息
fprintf('\n字符库统计:\n');
for i = 1:length(template_chars)
    fprintf('  %s: %d个模板\n', template_chars(i).name, length(template_chars(i).images));
end

%% 在原图上标记识别到的车牌 (已移除图像上的文字标签)
figure('Name', '原图上标记的车牌', 'Position', [400, 400, 800, 600]);
imshow(I);
hold on;

% 如果min_rect存在，绘制最小外接矩形
if exist('min_rect', 'var') && ~isempty(min_rect)
    plot(min_rect.corners([1:end,1],1), min_rect.corners([1:end,1],2), 'g-', 'LineWidth', 3);
end

title('原始图像上的车牌识别位置 (结果见弹窗)', 'FontSize', 16);
hold off;

%% ================== 弹窗显示结果 ==================
if exist('license_plate', 'var')
    msg_content = {
        '【抗干扰模式识别结果】';
        '-----------------------------';
        ['车牌号码: ', license_plate];
        ['检测到的字符: ', num2str(num_final)];
        ['有效识别字符: ', num2str(length(top_chars)+length(bottom_chars))];
        '-----------------------------';
        ['相似度阈值: ', num2str(similarity_threshold)]
    };
    msgbox(msg_content, '识别完成');
end