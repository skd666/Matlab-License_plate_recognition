clc; clear; close all;

%% 载入并预处理图像---正识别
I = imread("正常车牌\2.jpg"); 
if size(I,1)>2000 || size(I,2)>2000, I = imresize(I,0.5); end

figure
imshow(I)

%% 车牌分割
rgb = reshape(double(I), [], 3);
rgb(rgb(:,1)+rgb(:,2)+rgb(:,3)<500,:) = 0;
rgb(rgb(:,3)./rgb(:,1)>1.1,:) = 0;
rgb(rgb(:,1)./rgb(:,2)>1.05,:) = 0;
rgb(rgb(:,2)./rgb(:,1)>1.05,:) = 0;
rgb(rgb(:,1)./rgb(:,3)>1.08 & rgb(:,3)<200,:) = 0;
img = reshape(uint8(rgb), size(I));

figure
imshow(img)

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

%% 提取并缩放车牌
bw_plate = imclose(bw_largest, strel('disk', 50));
I_char = uint8(double(I).*bw_plate);

stats_plate = regionprops(bw_plate, 'BoundingBox');
if ~isempty(stats_plate)
    bbox = stats_plate.BoundingBox;
    x1 = max(1, floor(bbox(1))); y1 = max(1, floor(bbox(2)));
    x2 = min(size(I_char,2), ceil(bbox(1)+bbox(3)));
    y2 = min(size(I_char,1), ceil(bbox(2)+bbox(4)));
    
    plate_cropped = I_char(y1:y2, x1:x2, :);
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
else
    final_plate = uint8(zeros(200, 400, 3));
end

%% 对比度增强
final_plate = histeq(final_plate);

%% 车牌二值化
gray_plate = rgb2gray(final_plate);
threshold = graythresh(gray_plate);
plate_bw = ~imbinarize(gray_plate, threshold-0.18);

%% 删除边界连通区域
[L_bw, num_bw] = bwlabel(plate_bw);
stats_bw = regionprops(L_bw, 'BoundingBox', 'Area');
delete_mask = false(size(plate_bw));
[h, w] = size(plate_bw);

% 第一步：删除边界区域
for i = 1:num_bw
    bbox = stats_bw(i).BoundingBox;
    near_edge = bbox(1) <= 25 || bbox(1)+bbox(3) >= w-25 || ...
                bbox(2) <= 25 || bbox(2)+bbox(4) >= h-25;
    
    if near_edge
        aspect_ratio = bbox(3) / bbox(4);
        if aspect_ratio > 3 || aspect_ratio < 0.20 || ...
           bbox(3)*bbox(4) < 100 || ...
           (bbox(1) <= 25 && bbox(2) <= 25) || ...
           (bbox(1)+bbox(3) >= w-25 && bbox(2) <= 25)
            delete_mask(L_bw == i) = true;
        end
    end
end

plate_bw_clean = plate_bw;
plate_bw_clean(delete_mask) = false;

%% 第二步：剔除外接矩形面积大于1/8图像面积的连通区域
[L_clean, num_clean] = bwlabel(plate_bw_clean);
stats_clean = regionprops(L_clean, 'BoundingBox');
image_area = h * w;
threshold_bbox_area = image_area / 8;
large_bbox_mask = false(size(plate_bw_clean));

for i = 1:num_clean
    bbox = stats_clean(i).BoundingBox;
    bbox_area = bbox(3) * bbox(4);
    if bbox_area > threshold_bbox_area
        large_bbox_mask(L_clean == i) = true;
    end
end
plate_bw_clean(large_bbox_mask) = false;

%% 第三步：去掉宽高比大于3的连通区域
[L_final1, num_final1] = bwlabel(plate_bw_clean);
stats_final1 = regionprops(L_final1, 'BoundingBox');
large_aspect_mask = false(size(plate_bw_clean));

for i = 1:num_final1
    bbox = stats_final1(i).BoundingBox;
    aspect_ratio = bbox(3) / bbox(4);
    if aspect_ratio > 4 || aspect_ratio < 1/4
        large_aspect_mask(L_final1 == i) = true;
    end
end
plate_bw_clean(large_aspect_mask) = false;

figure
imshow(plate_bw_clean);
title('清理后的二值图像');

%% 矩形区域置0
left_rect = [0, 0, 125, 75];
right_rect = [280, 0, w, 73];
rect_mask = false(h, w);

rect_mask(1:left_rect(4), 1:left_rect(3)) = true;
rect_mask(1:right_rect(4), right_rect(1):right_rect(3)) = true;

plate_bw_final = plate_bw_clean;
plate_bw_final(rect_mask) = false;
plate_bw2 = bwareaopen(plate_bw_final, 100);

%% 顶部0-30像素矩形区域置0
top_rect_mask = false(size(plate_bw2));
top_height = 20;
top_rect_mask(1:top_height, :) = true;

plate_bw3 = plate_bw2;
plate_bw3(top_rect_mask) = false;

%% 闭运算
plate_bw4 = imclose(plate_bw3, strel('disk', 3));
plate_bw5 = bwareaopen(plate_bw4, 100);

%% 字符切割
[L_final, num_final] = bwlabel(plate_bw5);
stats_final = regionprops(L_final, 'BoundingBox', 'Area', 'Centroid');

centroids = zeros(num_final, 2);
for i = 1:num_final
    centroids(i, :) = stats_final(i).Centroid;
end

y_coords = centroids(:, 2);
y_mean = mean(y_coords);

top_indices = find(y_coords < y_mean);
bottom_indices = find(y_coords >= y_mean);

[~, top_sorted] = sort(centroids(top_indices, 1));
[~, bottom_sorted] = sort(centroids(bottom_indices, 1));

all_sorted_indices = [top_indices(top_sorted); bottom_indices(bottom_sorted)];

characters = cell(num_final, 1);
char_positions = cell(num_final, 1);

for i = 1:num_final
    idx = all_sorted_indices(i);
    bbox = stats_final(idx).BoundingBox;
    
    if ismember(idx, top_indices)
        char_positions{i} = 'top';
    else
        char_positions{i} = 'bottom';
    end
    
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
    
    char_resized = imresize(char_cropped, [32, 32]);
    characters{i} = char_resized;
end

%% 字符匹配
template_dir = 'font';
if ~exist(template_dir, 'dir')
    error('字符库文件夹不存在: %s', template_dir);
end

char_folders = dir(template_dir);
char_folders = char_folders([char_folders.isdir] & ~ismember({char_folders.name}, {'.', '..'}));

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

recognized_chars = cell(num_final, 1);
scores = zeros(num_final, 1);
similarity_threshold = 0.65;

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
    
    if best_score < similarity_threshold
        recognized_chars{i} = '';
    else
        recognized_chars{i} = best_char;
    end
    
    scores(i) = best_score;
end

top_chars = {};
bottom_chars = {};

for i = 1:num_final
    if ~isempty(recognized_chars{i})
        if strcmp(char_positions{i}, 'top')
            top_chars{end+1} = recognized_chars{i};
        else
            bottom_chars{end+1} = recognized_chars{i};
        end
    end
end

if ~isempty(top_chars), top_str = strjoin(top_chars, ''); else, top_str = ''; end
if ~isempty(bottom_chars), bottom_str = strjoin(bottom_chars, ''); else, bottom_str = ''; end

license_plate = [top_str, ' ', bottom_str];

figure('Name', '最终识别结果');
imshow(I); hold on;
if ~isempty(stats_plate)
    plate_bbox = stats_plate.BoundingBox;
    % 仅绘制绿色框，不显示文字
    rectangle('Position', plate_bbox, 'EdgeColor', 'g', 'LineWidth', 3);
end
title(['检测位置 (识别结果请查看弹窗)'], 'FontSize', 12);
hold off;

%% 控制台输出
fprintf('\n===== 车牌识别结果 =====\n');
fprintf('车牌号码: %s\n', license_plate);

%% ================== 弹窗显示结果 ==================
msg_content = {
    '【普通车牌识别结果】';
    '-----------------------------';
    ['车牌号码: ', license_plate];
    ['检测字符数: ', num2str(num_final)];
    ['上排字符: ', num2str(length(top_chars))];
    ['下排字符: ', num2str(length(bottom_chars))];
    '-----------------------------';
    ['相似度阈值: ', num2str(similarity_threshold)]
};
msgbox(msg_content, '识别完成');