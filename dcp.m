clc
clear
close all

%% 读取图像 --- 多车牌识别
I = imread('多车牌\4.jpg');  
% (1) 展示最开始原始图像
figure(1)
imshow(I)
title('原始图像')

%% 车牌分割（不显示中间图像）
rgb = reshape(double(I), [], 3);
rgb(rgb(:,1)+rgb(:,2)+rgb(:,3)<450,:) = 0;
rgb(rgb(:,2)./rgb(:,1)>1.1,:) = 0;
rgb(rgb(:,1)./rgb(:,3)>1.3,:) = 0;
rgb(rgb(:,1)./rgb(:,2)>1.1,:) = 0;
rgb(rgb(:,1)<180,:) = 0;
rgb(rgb(:,3)./rgb(:,1)>1.05,:) = 0;
img = reshape(uint8(rgb), size(I));

bw = img(:,:,1);
bw2 = bwareaopen(bw, 2000);
bw2 = imfill(bw2, "holes");
bw2 = imclose(bw2, strel('disk', 7));

%% 去除边框连通区域
if ~islogical(bw2)
    bw_binary = imbinarize(bw2);
else
    bw_binary = bw2;
end

[L, num] = bwlabel(bw_binary);
border_mask = false(size(bw_binary));
border_threshold = 5;
border_mask(1:border_threshold, :) = true;
border_mask(end-border_threshold+1:end, :) = true;
border_mask(:, 1:border_threshold) = true;
border_mask(:, end-border_threshold+1:end) = true;

border_objects = unique(L(border_mask));
border_objects(border_objects == 0) = [];

bw4 = bw_binary;
for i = 1:length(border_objects)
    bw4(L == border_objects(i)) = false;
end

%% 删掉小面积
bw6 = bwareaopen(bw4, 2000);

%% ====================== 最小外接矩形处理 ======================

% 设置多种阈值参数
area_ratio_threshold = 0.7;      % 面积比例阈值
max_height_width_ratio = 3.0;    % 最大高宽比
min_width_height_ratio = 1.5;    % 最小宽高比

% 获取连通区域
[L6, num6] = bwlabel(bw6);
stats = regionprops(L6, 'Area', 'Centroid', 'BoundingBox', 'MajorAxisLength', 'MinorAxisLength');

% 创建结果图像
result_bw = false(size(bw6));

% 初始化计数器
valid_objects = 0;

% 存储每个区域的特征信息
region_info = cell(num6, 1);

% 为每个连通区域计算最小外接矩形
for i = 1:num6
    % 获取当前连通区域的像素坐标
    [row, col] = find(L6 == i);
    
    % 计算最小外接矩形
    [rectx, recty, rect_area, rect_perimeter] = minboundrect(col, row, 'a');
    
    % 计算连通区域面积
    region_area = stats(i).Area;
    
    % 计算面积比例
    area_ratio = region_area / rect_area;
    
    % 计算外接矩形的宽度和高度
    rect_width = sqrt((rectx(2)-rectx(1))^2 + (recty(2)-recty(1))^2);
    rect_height = sqrt((rectx(3)-rectx(2))^2 + (recty(3)-recty(2))^2);
    
    % 确保宽度是较大的尺寸（车牌通常是宽大于高）
    if rect_height > rect_width
        temp = rect_width;
        rect_width = rect_height;
        rect_height = temp;
    end
    
    % 计算宽高比和高宽比
    width_height_ratio = rect_width / rect_height;
    height_width_ratio = rect_height / rect_width;
    
    % 计算细长程度（使用主次轴比）
    major_minor_ratio = stats(i).MajorAxisLength / max(stats(i).MinorAxisLength, 1);
    
    % 初始化剔除原因
    reject_reason = '';
    
    % 检查多个剔除条件
    if area_ratio < area_ratio_threshold
        reject_reason = '面积比例过低';
    elseif major_minor_ratio > 5  % 细长区域：主次轴比大于5
        reject_reason = '区域过于细长';
    elseif height_width_ratio > max_height_width_ratio  % 高度大于宽度
        reject_reason = '高度大于宽度';
    elseif width_height_ratio < min_width_height_ratio  % 宽高比太小（不够宽）
        reject_reason = '宽高比不符合车牌特征';
    end
    
    % 存储区域信息
    region_info{i} = struct(...
        'index', i, ...
        'area_ratio', area_ratio, ...
        'width', rect_width, ...
        'height', rect_height, ...
        'width_height_ratio', width_height_ratio, ...
        'height_width_ratio', height_width_ratio, ...
        'major_minor_ratio', major_minor_ratio, ...
        'region_area', region_area, ...
        'rect_area', rect_area, ...
        'rectx', rectx(1:4), ...
        'recty', recty(1:4), ...
        'centroid', stats(i).Centroid, ...
        'is_valid', false, ...
        'reject_reason', reject_reason);
    
    % 检查是否满足所有条件
    if isempty(reject_reason)
        % 保留该区域
        result_bw(L6 == i) = true;
        valid_objects = valid_objects + 1;
        region_info{i}.is_valid = true;
    end
end

% 获取有效区域的索引
valid_indices = find(cellfun(@(x) x.is_valid, region_info));

% (2) 展示原始图像处理后的识别到车牌的二值图
figure(2)
imshow(result_bw)
title('识别到车牌的二值图')

% (3) 展示在最开始原始图像和二值图叠加图像，叠加用红色掩膜叠加
figure(3)
imshow(I)
hold on
red_mask = cat(3, ones(size(result_bw)), zeros(size(result_bw)), zeros(size(result_bw)));
h = imshow(red_mask);
set(h, 'AlphaData', result_bw * 0.5); % 半透明红色掩膜
title('原始图像与车牌区域叠加（红色掩膜）')
hold off

%% ====================== 车牌四点透视校正 ======================

% 如果检测到车牌区域，对每个车牌进行透视校正
if valid_objects > 0
    fprintf('检测到 %d 个车牌区域\n', valid_objects);
    
    % 创建存储校正后结果的单元格数组
    corrected_plates = cell(valid_objects, 1);
    
    % 存储每个车牌的信息
    plate_info = cell(valid_objects, 1);
    
    % 处理每个车牌
    for plate_idx = 1:valid_objects
        idx = valid_indices(plate_idx);
        info = region_info{idx};
        
        % 从原图中裁剪车牌区域（基于最小外接矩形）
        min_x = floor(min(info.rectx));
        max_x = ceil(max(info.rectx));
        min_y = floor(min(info.recty));
        max_y = ceil(max(info.recty));
        
        % 扩展裁剪区域（确保车牌完整）
        expand_pixels = 10;
        min_x = max(min_x - expand_pixels, 1);
        max_x = min(max_x + expand_pixels, size(I, 2));
        min_y = max(min_y - expand_pixels, 1);
        max_y = min(max_y + expand_pixels, size(I, 1));
        
        % 裁剪车牌区域
        plate_region = I(min_y:max_y, min_x:max_x, :);
        
        % 从二值图像中裁剪对应的区域
        plate_bw = result_bw(min_y:max_y, min_x:max_x);
        
        % 调整rectx和recty到裁剪图像的坐标系
        adjusted_rectx = info.rectx - min_x + 1;
        adjusted_recty = info.recty - min_y + 1;
        
        % 直接使用minboundrect的4个顶点进行透视校正
        sorted_corners = sortCorners([adjusted_rectx(:), adjusted_recty(:)]);
        
        % 计算当前四边形的宽度和高度
        width_top = norm(sorted_corners(2,:) - sorted_corners(1,:));
        width_bottom = norm(sorted_corners(3,:) - sorted_corners(4,:));
        avg_width = (width_top + width_bottom) / 2;
        
        height_left = norm(sorted_corners(4,:) - sorted_corners(1,:));
        height_right = norm(sorted_corners(3,:) - sorted_corners(2,:));
        avg_height = (height_left + height_right) / 2;
        
        % 确保宽度 > 高度（车牌通常宽度大于高度）
        if avg_height > avg_width
            sorted_corners = sorted_corners([2,3,4,1], :);
        end
        
        original_aspect_ratio = avg_width / avg_height;
        
        % 设置目标尺寸
        target_height = 150;
        min_aspect_ratio = 1.5;
        
        if original_aspect_ratio < min_aspect_ratio
            original_aspect_ratio = min_aspect_ratio;
        end
        
        target_width = round(target_height * original_aspect_ratio);
        
        % 确保目标宽度 > 高度
        if target_width <= target_height
            target_width = target_height * 2;
        end
        
        % 目标矩形的四个角点（左上、右上、右下、左下）
        destinationPoints = [1, 1; 
                            target_width, 1; 
                            target_width, target_height; 
                            1, target_height];
        
        % 计算透视变换矩阵
        try
            tform = fitgeotrans(sorted_corners, destinationPoints, 'projective');
            
            % 应用透视变换
            plate_corrected = imwarp(plate_region, tform, 'OutputView', imref2d([target_height, target_width]));
            
            % 存储结果
            corrected_plates{plate_idx} = plate_corrected;
            
            % 存储车牌信息
            plate_info{plate_idx} = struct(...
                'index', idx, ...
                'original_size', size(plate_region), ...
                'corrected_size', [target_width, target_height], ...
                'aspect_ratio', original_aspect_ratio, ...
                'success', true);
            
        catch ME
            fprintf('车牌 %d 透视变换失败: %s\n', plate_idx, ME.message);
            
            % 如果失败，使用原始图像
            corrected_plates{plate_idx} = plate_region;
            
            plate_info{plate_idx} = struct(...
                'index', idx, ...
                'original_size', size(plate_region), ...
                'corrected_size', size(plate_region), ...
                'aspect_ratio', original_aspect_ratio, ...
                'success', false, ...
                'error', ME.message);
        end
    end
    
    %% ====================== 字符分割和字符匹配识别 ======================
    
    % 创建存储识别结果的单元格数组
    recognition_results = cell(valid_objects, 1);
    
    for plate_idx = 1:valid_objects
        % 获取校正后的车牌图像
        final_plate = corrected_plates{plate_idx};
        
        % 如果图像是二值的，转换为彩色
        if size(final_plate, 3) == 1
            final_plate = repmat(final_plate, [1, 1, 3]);
        end
        
        % 获取图像尺寸
        [h, w, ~] = size(final_plate);
        
        %% 对比度增强
        scale = min(200/h, 400/w);
        plate_resized = imresize(final_plate, round([h w]*scale));
        final_plate = histeq(final_plate);
        
        %% 车牌二值化
        gray_plate = rgb2gray(final_plate);
        threshold = graythresh(gray_plate);
        plate_bw = ~imbinarize(gray_plate, threshold-0.28);
        plate_bw(1:20,:) = 0;
        plate_bw(end-19:end,:) = 0;
        plate_bw(:,1:20) = 0;
        plate_bw(:, end-19:end) = 0;
        
        plate_bw_clean = plate_bw;
        plate_bw_clean = bwareaopen(plate_bw_clean, 20);
        
        %% 矩形区域置0
        left_rect_width = round(125 * (w/400));
        left_rect_height = round(35 * (h/150));
        right_rect_start = round(280 * (w/400));
        right_rect_height = round(50 * (h/150));
        
        left_rect = [0, 0, left_rect_width, left_rect_height];
        right_rect = [right_rect_start, 0, w, right_rect_height];
        rect_mask = false(h, w);
        
        rect_mask(1:left_rect(4), 1:left_rect(3)) = true;
        rect_mask(1:right_rect(4), right_rect(1):right_rect(3)) = true;
        
        plate_bw_final = plate_bw_clean;
        plate_bw_final(rect_mask) = false;
        plate_bw_final = imclose(plate_bw_final, strel('disk', 3));
        plate_bw2 = bwareaopen(plate_bw_final, 120);
        
        %% 移除宽度大于高度两倍的连通区域
        [L, num] = bwlabel(plate_bw2);
        stats = regionprops(L, 'BoundingBox');
        
        for i = 1:num
            bbox = stats(i).BoundingBox;
            if bbox(3) > 3 * bbox(4)
                plate_bw2(L == i) = false;
            end
        end
        
        plate_bw2 = bwareaopen(plate_bw2, 120);
        
        %% 删除plate_bw2图像边界的连通区域
        [L_bw2, num_bw2] = bwlabel(plate_bw2);
        
        border_mask = false(size(plate_bw2));
        border_threshold = 3;
        
        border_mask(1:border_threshold, :) = true;
        border_mask(end-border_threshold+1:end, :) = true;
        border_mask(:, 1:border_threshold) = true;
        border_mask(:, end-border_threshold+1:end) = true;
        
        border_objects = unique(L_bw2(border_mask));
        border_objects(border_objects == 0) = [];
        
        for obj_idx = 1:length(border_objects)
            plate_bw2(L_bw2 == border_objects(obj_idx)) = false;
        end
        
        %% 顶部0-30像素矩形区域置0
        top_rect_mask = false(size(plate_bw2));
        top_height = 20;
        top_rect_mask(1:top_height, :) = true;
        
        plate_bw3 = plate_bw2;
        plate_bw3(top_rect_mask) = false;
        
        %% 闭运算
        plate_bw4 = imclose(plate_bw3, strel('disk', 3));
        plate_bw5 = bwareaopen(plate_bw4, 20);
        
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
        top_indices = find(y_coords < y_mean);
        bottom_indices = find(y_coords >= y_mean);
        
        % 对每行字符按x坐标排序
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
            % 如果没有字符库，使用简单的字符识别
            recognized_chars = cell(num_final, 1);
            scores = zeros(num_final, 1);
            
            for i = 1:num_final
                recognized_chars{i} = '?';
                scores(i) = 0.5;
            end
        else
            % 获取所有子文件夹
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
            
            % 匹配字符
            recognized_chars = cell(num_final, 1);
            scores = zeros(num_final, 1);
            
            for i = 1:num_final
                char_img = characters{i};
                best_score = -inf;
                best_char = '?';
                
                for j = 1:length(template_chars)
                    templates = template_chars(j).images;
                    folder_name = template_chars(j).name;
                    
                    for k = 1:length(templates)
                        template = templates{k};
                        corr_score = corr2(char_img(:), template(:));
                        
                        if corr_score > best_score
                            best_score = corr_score;
                            best_char = folder_name;
                        end
                    end
                end
                
                recognized_chars{i} = best_char;
                scores(i) = best_score;
            end
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
        
        % 存储识别结果
        recognition_results{plate_idx} = struct(...
            'license_plate', license_plate, ...
            'num_chars', num_final, ...
            'top_chars', {top_chars}, ...
            'bottom_chars', {bottom_chars}, ...
            'recognized_chars', {recognized_chars}, ...
            'scores', scores, ...
            'characters', {characters}, ...
            'plate_bw5', plate_bw5, ...
            'stats_final', stats_final, ...
            'all_sorted_indices', all_sorted_indices);
        
        fprintf('车牌 %d 识别结果: %s\n', plate_idx, license_plate);
    end
    
    %% ====================== 按要求展示图像 ======================
    
    % (4) 所有识别到的车牌（透视校正后的车牌）
    figure(4)
    if valid_objects <= 4
        rows = 1;
        cols = valid_objects;
    else
        rows = ceil(sqrt(valid_objects));
        cols = ceil(valid_objects / rows);
    end
    
    for plate_idx = 1:valid_objects
        subplot(rows, cols, plate_idx)
        imshow(corrected_plates{plate_idx})
        title(sprintf('车牌 %d', plate_idx))
    end
    sgtitle('透视校正后的车牌图像')
    
    % (5) 最后显示所有车牌识别结果
    figure(5)
    imshow(I)
    hold on
    
    for plate_idx = 1:valid_objects
        idx = valid_indices(plate_idx);
        info = region_info{idx};
        result = recognition_results{plate_idx};
        
        % 绘制车牌区域 (绿色框)
        plot(info.rectx(1:4), info.recty(1:4), 'g-', 'LineWidth', 2);
        plot([info.rectx(1:4); info.rectx(1)], [info.recty(1:4); info.recty(1)], 'g-', 'LineWidth', 2);
        
        % text(...) 显示车牌字符的代码已移除
    end
    
    title(sprintf('原始图像上的车牌检测位置 (结果见弹窗)'))
    hold off
    
    %% ================== 弹窗显示结果 ==================
    msg_content = {'【多车牌识别结果汇总】'; '--------------------------------'};
    for k = 1:valid_objects
        plate_str = recognition_results{k}.license_plate;
        msg_content{end+1} = sprintf('车牌 #%d: %s', k, plate_str);
    end
    msgbox(msg_content, '识别完成');
    
else
    fprintf('未检测到有效车牌区域\n');
    msgbox('未检测到有效车牌', '识别结果');
end