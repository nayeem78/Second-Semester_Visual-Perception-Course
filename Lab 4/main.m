%% Load image

I = im2double(imread('Image_base_050.jpg'));

% Center coordinates
c = [2300;2300;1];
region_size = [500;750;0];
l = region_size/2;
top_left = c - region_size/2;

% Get 2 windowped images, one smaller and the other larger
window = I(top_left(1):(top_left(1)+region_size(1)),top_left(2):(top_left(2)+region_size(2)),:);
c_window = l + 1;
large_window = I((c(1)-750):(c(1)+750),(c(2)-750):(c(2)+750),:);
c_large = [751;751;1];

%% projectedective Homographies

% projectedection -> 1 = no deformation
z = [1.1  1.2  1.3  1.4  1    1    1    1;
     1    1    1    1    1.1  1.2  1.3  1.4];

% centre offsets for ref points - "8-neighbourhood" of centre
c_os = [l(1)  l(1)  -l(1)  -l(1);
        l(2)  -l(2)  -l(2)  l(2)];

% Reference points in window and large_window
ref_window = ones(3,4);
for i = 1:2
    ref_window(i,:) = c_os(i,:) + c_window(i);
end

% Fix values for X and Y
ref_window = [ref_window(2,:);ref_window(1,:);ref_window(3,:)];

p_window = ones(3,4);
ref = imref2d(size(window));

% For each case
for i = 1:8
    index = i;

    zx1 = z(1,i);
    zx2 = z(2,i);
    zx = (zx1 + zx2)/2;
    zy1 = 1;
    zy2 = 1;
    zy = (zy1 + zy2)/2;

    % Offset
    k = [zy1  zy2  zy2  zy1;
         zx1  zx1  zx2  zx2];
    os = k.*c_os;

    % Deformation points
    for ii = 1:2
        p_window(ii,:) = os(ii,:) + c_window(ii);
    end

    % Fix values for X and Y
    p_window = [p_window(2,:);p_window(1,:);p_window(3,:)];

    % Compute homography
    tform = fitgeotrans(ref_window(1:2,:)',p_window(1:2,:)','projectedective');
    H = tform.T';

    % Apply transformationormation and window new image
    projected_image = imwarp(window,tform,'OutputView',ref);
    figure;imshow(projected_image);size(projected_image)

    % Save images
    name = ['SEQUENCE1/Image_' num2str(index,'%.2u')];
    imwrite(projected_image,[name 'a.png']);
    imwrite(imnoise(projected_image,'gaussian',0,(3/255)^2),[name 'b.png']);
    imwrite(imnoise(projected_image,'gaussian',0,(6/255)^2),[name 'c.png']);
    imwrite(imnoise(projected_image,'gaussian',0,(18/255)^2),[name 'd.png']);

    % Save homography
    Sequence1Homographies(index).H = H;
end

% For each other case
for i = 1:8
    index = i + 8;

    zx1 = 1;
    zx2 = 1;
    zx = (zx1 + zx2)/2;
    zy1 = z(1,i);
    zy2 = z(2,i);
    zy = (zy1 + zy2)/2;

    % Offset
    k = [zy1  zy2  zy2  zy1;
         zx1  zx1  zx2  zx2];
    os = k.*c_os;

    % Deformation points
    for ii = 1:2
        p_window(ii,:) = os(ii,:) + c_window(ii);
    end

    % Fix x and y because reasons
    p_window = [p_window(2,:);p_window(1,:);p_window(3,:)];

    % Compute homography
    tform = fitgeotrans(ref_window(1:2,:)',p_window(1:2,:)','projectedective');
    H = tform.T';

    % Apply transformationormation and window new image
    projected_image = imwarp(window,tform,'OutputView',ref);
    figure;imshow(projected_image);size(projected_image)

    % Save images
    name = ['SEQUENCE1/Image_' num2str(index,'%.2u')];
    imwrite(projected_image,[name 'a.png']);
    imwrite(imnoise(projected_image,'gaussian',0,(3/255)^2),[name 'b.png']);
    imwrite(imnoise(projected_image,'gaussian',0,(6/255)^2),[name 'c.png']);
    imwrite(imnoise(projected_image,'gaussian',0,(18/255)^2),[name 'd.png']);

    % Save homography
    Sequence1Homographies(index).H = H;
end

% Save homographies
save('Sequence1Homographies.mat','Sequence1Homographies');

%% Test

i = 13;

projected_image = imread(['SEQUENCE1/Image_' num2str(i,'%.2u') 'a.png']);


H = Sequence1Homographies(i).H;
p = [320, 360, 1]';

figure(1);
imshow(window);
hold on;
scatter(p(1), p(2), 'ro', 'linewidth', 3);

pause(0.2)

figure(2);
imshow(projected_image);
hold on;
p2 = H * p;
p2 = p2 / p2(end);
scatter(p2(1), p2(2), 'ro', 'linewidth', 3);

%% Zoom
for i = 1:9
    zoom = 1.05 + i * 0.05;
    
    % Define the homographic matrix
    A = [zoom 0 0;0 zoom 0;0 0 1];
    Sequence2Homographies(i).H = [zoom 0 ((1 - zoom) * l(2)); 0 zoom ((1 - zoom) * l(1)); 0 0 1];
    
    %Get the transformationormation
    transformation = affine2d(A);
    [zoomed_image, ~] = imwarp(large_window, transformation);
    s = size(zoomed_image);
    c = round(s(1:2)/2); % center
    
    %get the scaled image
    zoomed_image = zoomed_image((c(1) - l(1)) : (c(1) + l(1)), (c(2) - l(2)) : (c(2) + l(2)), :);

    imwrite(zoomed_image, strcat('SEQUENCE2/Image_', num2str(i,'%.2u'), 'a.png'));
    imwrite(imnoise(zoomed_image, 'gaussian', 0, (3/255)^2), strcat('SEQUENCE2/Image_', num2str(i,'%.2u'), 'b.png'));
    imwrite(imnoise(zoomed_image, 'gaussian', 0, (6/255)^2), strcat('SEQUENCE2/Image_', num2str(i,'%.2u'), 'c.png'));
    imwrite(imnoise(zoomed_image, 'gaussian', 0, (18/255)^2), strcat('SEQUENCE2/Image_', num2str(i,'%.2u'), 'd.png'));
end

save('Sequence2Homographies.mat','Sequence2Homographies');

%% Rotation

image_container = [1500 1500];
image_size = [500 750];
c = [2300 2300 1]';

%smaller windowping region
image_piece = image(c(1)-floor(image_container(1)/2):c(1)+ceil(image_container(1)/2), c(2)-floor(image_container(2)/2):c(2)+ceil(image_container(2)/2), :);

range = [-45 : 5 : -5, 5 : 5 : 45];

%pointer to write images in sequence
trans_idx = 1;
for i = range
    theta = i / 180 * pi;
    
    %Define the homographic matrix
    T = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    tform = affine2d(T);
    [transformationormed_image, ~] = imwarp(image_piece, tform);
    
    %transformationormations for the image
    new_center = round(size(transformationormed_image) / 2);
    range_x = new_center(1)-floor(image_size(1)/2) : new_center(1)+ceil(image_size(1)/2);
    range_y = new_center(2)-floor(image_size(2)/2) : new_center(2)+ceil(image_size(2)/2);
    image = transformationormed_image(range_x, range_y, :);
    
    l1 = size(image, 2) / 2;
    l2 = size(image, 1) / 2;
    
    %Get the final homographic matrix from the computed images
    T = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
    T(1, 3) = -l1*cos(theta) - l2*sin(theta) + l1;
    T(2, 3) = l1*sin(theta) - l2*cos(theta) + l2;
    Sequence3Homographies(trans_idx).H = T;
    
    %saving the images
    imwrite(image, strcat('SEQUENCE3/Image_', num2str(trans_idx,'%.2u'), 'a.png'));
    imwrite(imnoise(image, 'gaussian', 0, (3/255)^2), strcat('SEQUENCE3/Image_', num2str(trans_idx,'%.2u'), 'b.png'));
    imwrite(imnoise(image, 'gaussian', 0, (6/255)^2), strcat('SEQUENCE3/Image_', num2str(trans_idx,'%.2u'), 'c.png'));
    imwrite(imnoise(image, 'gaussian', 0, (18/255)^2), strcat('SEQUENCE3/Image_', num2str(trans_idx,'%.2u'), 'd.png'));
    
    trans_idx = trans_idx + 1;
end

save('Sequence3Homographies.mat', 'Sequence3Homographies');

%% Part 2
noises = {'a', 'b', 'c', 'd'};
colors = {'r', 'b', 'g', 'c'};

%% Evaluate Sequence 1
load Sequence1Homographies

I1 = imread('SEQUENCE1/Image_00a.png');
I1 = single(rgb2gray(I1));

figure(1); hold on; grid on;
title('Accuracy of SIFT in Seq. 1');
xlabel('projectedection'); ylabel('Percentage of correct matches');
xlim([1 size(Sequence1Homographies, 2)]); ylim([0.7 1]);
set(gca, 'xTick', 1:16);
set(gca, 'xTickLabel', {'110%','120%','130', '140%', '110%', '120%', '130%', '140%', '110%', '120%', '130%', '140%', '110%', '120%', '130%', '140%'});

for noise = 1 : length(noises)
    results = zeros(size(Sequence1Homographies, 2), 1);
    for i = 1 : size(Sequence1Homographies, 2)
        I2 = imread(strcat('SEQUENCE1/Image_', num2str(i,'%.2u'), noises{noise},'.png'));
        I2 = single(rgb2gray(I2));

        results(i, :) = evaluate_sift(I1, I2, Sequence1Homographies(i).H, false);
    end

    plot(1:size(Sequence1Homographies, 2), results(:, 1), colors{noise});
end
legend('No noise', 'N(0, 3)', 'N(0, 6)', 'N(0, 18)', 'Location', 'southeast');
plot([4, 4], [0, 1], 'r--')
plot([8, 8], [0, 1], 'r--')
plot([12, 12], [0, 1], 'r--')
plot([16, 16], [0, 1], 'r--')
text(2.3, 0.8, 'UP')
text(5.5, 0.8, 'DOWN')
text(9.5, 0.8, 'LEFT')
text(13.5, 0.8, 'RIGHT')

%% Evaluate Sequence 2
load Sequence2Homographies

I1 = imread('SEQUENCE2/Image_00a.png');
I1 = single(rgb2gray(I1));

figure(2); hold on; grid on;
title('Accuracy of SIFT in Seq. 2');
xlabel('Zoom'); ylabel('Percentage of correct matches');
xlim([1 size(Sequence2Homographies, 2)]); ylim([0.7 1]);
set(gca, 'xTick', 1:9);
set(gca, 'xTickLabel', {'110%','115%','120%','125%', '130%', '135%', '140%', '145%', '150%'});

for noise = 1 : length(noises)
    results = zeros(size(Sequence2Homographies, 2), 1);
    for i = 1 : size(Sequence2Homographies, 2)
        I2 = imread(strcat('SEQUENCE2/Image_', num2str(i,'%.2u'), noises{noise},'.png'));
        I2 = single(rgb2gray(I2));

        results(i, :) = evaluate_sift(I1, I2, Sequence2Homographies(i).H, false);
    end

    plot(1:size(Sequence2Homographies, 2), results(:, 1), colors{noise});
end
legend('No noise', 'N(0, 3)', 'N(0, 6)', 'N(0, 18)', 'Location', 'southeast');

%% Evaluate sequence 3
load Sequence3Homographies

I1 = imread('SEQUENCE3/Image_00a.png');
I1 = single(rgb2gray(I1));

figure(3); hold on; grid on;
title('Accuracy of SIFT in Seq. 3');
xlabel('Rotation angle'); ylabel('Percentage of correct matches');
xlim([1 size(Sequence3Homographies, 2)]); ylim([0.7 1]);
set(gca, 'xTick', 1:18);
set(gca, 'xTickLabel', [-45:5:-5, 5:5:45]);

for noise = 1 : length(noises)
    results = zeros(size(Sequence3Homographies, 2), 1);
    for i = 1 : size(Sequence3Homographies, 2)
        I2 = imread(strcat('SEQUENCE3/Image_', num2str(i,'%.2u'), noises{noise}, '.png'));
        I2 = single(rgb2gray(I2));

        results(i, :) = evaluate_sift(I1, I2, Sequence3Homographies(i).H, false);
    end

    plot(1:size(Sequence3Homographies, 2), results(:, 1), colors{noise});
end
legend('No noise', 'N(0, 3)', 'N(0, 6)', 'N(0, 18)', 'Location', 'southeast');

%% Part 3

noises = {'a', 'b', 'c', 'd'};
colors = {'r', 'b', 'g', 'c'};
forms_1 = {'+', '+', '+', '+'};
forms_2 = {'--', '--', '--', '--'};

%% Evaluate Sequence 1
load Sequence1Homographies

I1 = imread('SEQUENCE1/Image_00a.png');
I1 = single(rgb2gray(I1));

figure(1); hold on; grid on;
title('Accuracy of SIFT in Seq. 1');
xlabel('projectedection'); ylabel('Percentage of correct matches');
xlim([1 size(Sequence1Homographies, 2)]); ylim([0.7 1]);
set(gca, 'xTick', 1:16);
set(gca, 'xTickLabel', {'110%','120%','130', '140%', '110%', '120%', '130%', '140%', '110%', '120%', '130%', '140%', '110%', '120%', '130%', '140%'});

for noise = 1 : length(noises)
    results = zeros(size(Sequence1Homographies, 2), 2);
    for i = 1 : size(Sequence1Homographies, 2)
        I2 = imread(strcat('SEQUENCE1/Image_', num2str(i,'%.2u'), noises{noise},'.png'));
        I2 = single(rgb2gray(I2));

        loc1 = evaluate_sift(I1, I2, Sequence1Homographies(i).H, false);
        loc2 = evaluate_sift(I1, I2, Sequence1Homographies(i).H, true);

        results(i, 1) = loc1;
        results(i, 2) = loc2;
    end
    
    style_1 = strcat(forms_1{noise}, '-', colors{noise});
    style_2 = strcat(forms_2{noise}, colors{noise});
    
    plot(1:size(Sequence1Homographies, 2), results(:, 1), style_1);
    plot(1:size(Sequence1Homographies, 2), results(:, 2), style_2);
end
legend('No noise standard', 'No noise modified', 'N(0, 3) standard', 'N(0, 3) modified', 'N(0, 6) standard', 'N(0, 6) modified', 'N(0, 18) standard', 'N(0, 18) modified', 'Location', 'southeast');
plot([4, 4], [0, 1], 'r--')
plot([8, 8], [0, 1], 'r--')
plot([12, 12], [0, 1], 'r--')
plot([16, 16], [0, 1], 'r--')
text(2.3, 0.8, 'UP')
text(5.5, 0.8, 'DOWN')
text(9.5, 0.8, 'LEFT')
text(13.5, 0.8, 'RIGHT')

%% Evaluate Sequence 2
load Sequence2Homographies

I1 = imread('SEQUENCE2/Image_00a.png');
I1 = single(rgb2gray(I1));

figure(2); hold on; grid on;
title('Accuracy of SIFT in Seq. 2');
xlabel('Zoom'); ylabel('Percentage of correct matches');
xlim([1 size(Sequence2Homographies, 2)]); ylim([0.7 1]);
set(gca, 'xTick', 1:9);
set(gca, 'xTickLabel', {'110%','115%','120%','125%', '130%', '135%', '140%', '145%', '150%'});

for noise = 1 : length(noises)
    results = zeros(size(Sequence2Homographies, 2), 1);
    for i = 1 : size(Sequence2Homographies, 2)
        I2 = imread(strcat('SEQUENCE2/Image_', num2str(i,'%.2u'), noises{noise},'.png'));
        I2 = single(rgb2gray(I2));

        loc1 = evaluate_sift(I1, I2, Sequence2Homographies(i).H, false);
        loc2 = evaluate_sift(I1, I2, Sequence2Homographies(i).H, true);

        results(i, 1) = loc1;
        results(i, 2) = loc2;
    end

    style_1 = strcat(forms_1{noise}, '-', colors{noise});
    style_2 = strcat(forms_2{noise}, colors{noise});

    plot(1:size(Sequence2Homographies, 2), results(:, 1), style_1);
    plot(1:size(Sequence2Homographies, 2), results(:, 2), style_2);
end
legend('No noise standard', 'No noise modified', 'N(0, 3) standard', 'N(0, 3) modified', 'N(0, 6) standard', 'N(0, 6) modified', 'N(0, 18) standard', 'N(0, 18) modified', 'Location', 'southeast');

%% Evaluate sequence 3
load Sequence3Homographies

I1 = imread('SEQUENCE3/Image_00a.png');
I1 = single(rgb2gray(I1));

figure(3); hold on; grid on;
title('Accuracy of SIFT in Seq. 3');
xlabel('Rotation angle'); ylabel('Percentage of correct matches');
xlim([1 size(Sequence3Homographies, 2)]); ylim([0.7 1]);
set(gca, 'xTick', 1:18);
set(gca, 'xTickLabel', [-45:5:-5, 5:5:45]);

for noise = 1 : length(noises)
    results = zeros(size(Sequence3Homographies, 2), 1);
    for i = 1 : size(Sequence3Homographies, 2)
        I2 = imread(strcat('SEQUENCE3/Image_', num2str(i,'%.2u'), noises{noise}, '.png'));
        I2 = single(rgb2gray(I2));

        loc1 = evaluate_sift(I1, I2, Sequence3Homographies(i).H, false);
        loc2 = evaluate_sift(I1, I2, Sequence3Homographies(i).H, true);

        results(i, 1) = loc1;
        results(i, 2) = loc2;
    end

    style_1 = strcat(forms_1{noise}, '-', colors{noise});
    style_2 = strcat(forms_2{noise}, colors{noise});
    
    plot(1:size(Sequence3Homographies, 2), results(:, 1), style_1);
    plot(1:size(Sequence3Homographies, 2), results(:, 2), style_2);
end
legend('No noise standard', 'No noise modified', 'N(0, 3) standard', 'N(0, 3) modified', 'N(0, 6) standard', 'N(0, 6) modified', 'N(0, 18) standard', 'N(0, 18) modified', 'Location', 'southeast');