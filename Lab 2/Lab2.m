clc;

%part 1 and 2
%im = imread('Images-20170316\chessboard03.png');
%im = imread('Images-20170316\chessboard04.png');
im = imread('Images-20170316\chessboard05.png');
% im = imread('Images-20170316\chessboard06.png');
im_orig = im;
figure,
subplot 231
imshow(im_orig);
title('Original Image');

if size(im,3)>1 
    im=rgb2gray(im); 
end

% Derivative masks
dx = [-1 0 1;
-1 0 1;
-1 0 1];
dy = dx';

% Image derivatives
Ix = conv2(double(im), dx, 'same');
Iy = conv2(double(im), dy, 'same');
sigma=2;

% Generate Gaussian filter of size 9x9 and std. dev. sigma.
g = fspecial('gaussian',9, sigma);

% Smoothed squared image derivatives
Ix2 = conv2(Ix.^2, g, 'same');
Iy2 = conv2(Iy.^2, g, 'same');
Ixy = conv2(Ix.*Iy, g, 'same');

[m,n]=size(im);


%sum of every pixels 
k = 0.04;

%Tic and tac initilaize values
RMat_time = 0;
EMat_time = 0;

for i = 2:m-1
    for j = 2:n-1
        
        %auto correlation matrices or sqaure gradiant matrices
        sum_Ix2 = sum(sum(Ix2(i-1:i+1,j-1:j+1)));
        sum_Iy2 = sum(sum(Iy2(i-1:i+1,j-1:j+1)));
        sum_Ixy = sum(sum(Ixy(i-1:i+1,j-1:j+1)));
               
        %Define a M matrix
         
        M = [sum_Ix2,sum_Ixy; sum_Ixy,sum_Iy2];
        %determinant of Matrix M
        %det(M) = (M(1,1) * M(2,2) - M(1,2) * M(2,1));
        %trace of Matrix M
        %trace(M) = M(1,1) + M(2,2);
        tic
        R(i,j) = (M(1,1)*M(2,2) - M(1,2)*M(2,1)) - k*((M(1,1) + M(2,2))^2);
        RMat_time = RMat_time+ toc;
        tic
        E(i,j) = min(eig(M));
        EMat_time = RMat_time+ toc;     
              
    end
end

%printing R and E matrix values
fprintf('R matrix time: %f\n',RMat_time);
fprintf('E matrix time: %f\n',EMat_time);

subplot 232
imshow(mat2gray(E));
title('Eigen Values')

subplot 233
imshow(mat2gray(R));
title('Image of R')

%Part 3
%selecting between E and R matrix
%R and E matrix Selector
R_E_mat = 1;
if(R_E_mat == 1)
    mat = E;
else
    mat = R;
end

%Get 81 salient Points
sal_num = 81;
[E_sort,i] = sort(mat(:), 'descend');
[p_x, p_y] = ind2sub(size(mat), i(1:sal_num));
%Display the image
subplot 234, 
imshow(im_orig);
hold on;
title('Without non-maximal suppression corners, Step 3')
for iterator=1:size(p_x,1)
    plot(p_x(iterator), p_y(iterator), 'r+');
end
hold off;

%% Part 4 - Non-maximal Suppression
 
features(sal_num).p_x = 0;
features(sal_num).p_y = 0;

%[val,idx] = sort(E(:),'descend');
[I,J] = ind2sub(size(mat),i);

i = 0;% Iterator over all pixels
f = 0;% Iterator over feature points feature points
D = mat;

while f < sal_num
    i = i + 1;
    if D(I(i),J(i)) == 0
        continue    
    else
        f = f + 1;
        %get the next largest value
        features(f).p_x = I(i);
        features(f).p_y = J(i);
        
        %set the other pixels in the window to 0
        for ii = -5:5
            if (I(i) + ii) <= 0 || (I(i) + ii) > r
                continue;
            end
            for jj = -5:5
                if (J(i) + jj) <= 0 || (J(i) + jj) > c || (ii == 0 && jj == 0)
                    continue;
                end
                D(I(i)+ii, J(i)+jj) = 0;
            end
        end
               
    end
end

%Plot with non-maximal suppression
subplot(2,3,5);
imshow(im_orig); 
hold on;
title('Corners with non-maximal suppression')
for iterator=1:size(features,2)
    plot(features(iterator).p_x, features(iterator).p_y, 'r+');
end
hold off;

%% Part 5 - Sub-pixel Accuracy
i = 0;
subp = zeros(81,2);
while i < sal_num
    i = i + 1;
    x = features(i).p_x;
    y = features(i).p_y;
    A = [];
    b = [];
    for ii = [-1 0 1]
        for jj = [-1 0 1]
            A = [A;(x+ii)^2, (x+ii)*(y+jj), (y+jj)^2,  x+ii, y+jj, 1]; 
            b = [b;E(x+ii,y+jj)];
        end
    end
    p = A\b; % Least mean square to find all the unknown parameters
    if 4*p(1)*p(3) -p(2)*p(2) <= 0
        subp(i,1) = x;
        subp(i,2) = y;
    else
        subp(i,:) = [2*p(1) p(2);p(2) 2*p(3)]\[-p(4);-p(5)];
        if abs(subp(1) - x) > 0.5 || abs(subp(2)-y) > 0.5
            subp(i,1) = x;
            subp(i,2) = y;
        end
    end
end

%Plot with non-maximal suppression
subplot 236, imshow(im_orig); hold on;
title('Corners subpixel accuracy')
for iterator=1:size(subp,1)
    plot(subp(iterator,2), subp(iterator,1), 'r+');
end
hold off;


