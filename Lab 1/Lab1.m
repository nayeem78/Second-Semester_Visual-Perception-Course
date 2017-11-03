%Lab 1 - Camera Caliberation

%step 1
au=557.0943; av=712.9824; u0=326.3819; v0=298.6679;
f=80;
Tx=100; Ty=0; Tz=1500;
phix=0.8*pi/2; phiy=-1.8*pi/2; phix1=pi/5;

%step 2 Intrinsic and extrinsic

I = [au, 0, u0,0;
      0, av, v0, 0;
       0, 0, 1,0];
   

rotx = [1,0,0;
        0, cos(phix), -sin(phix);
        0, sin(phix),  cos(phix)];
        
roty = [cos(phix), 0, sin(phix);
        0, 1, 0;
        -sin(phix), 0, cos(phix)];

rotx1 = [cos(phix),-sin(phix),0;
         sin(phix), cos(phix),0;
         0,0,1];
            
R = rotx*roty*rotx1;
E = [R(1,:),Tx; R(2,:),Ty; R(3,:),Tz; 0, 0,0, 1];
%step3 

%%rand = [-234:200;-123:182;-45:48];
randmatrix = [randi([-480,480],[6,1]),randi([-480,480],[6,1]),randi([-480,480], [6,1])];
randmatrix = randmatrix';
randmatrix(4,:) = 1
figure
scatter (randmatrix(1,:), randmatrix(2,:),randmatrix(3,:));
%step 4

pmatrix = zeros (2,6);
pmatrix = I*E*randmatrix;
scatter (pmatrix(1,:), pmatrix(2,:));
%step 5



% normalize the projected points
projNorm(1,:) = pmatrix(1,:)./pmatrix(3,:);
projNorm(2,:) = pmatrix(2,:)./pmatrix(3,:);

%figure, scatter3(randmatrix(1,:), randmatrix(2,:), randmatrix(3,:), 'bo'), title('3D points');
%figure, scatter(projNorm(1,:), projNorm(2,:), 'bo', 'MarkerFaceColor', 'b'), title('projected 2D points');



%step 6 
Q=[];B=[];

for i=1:6
%     Q(2*(i-1),:) = [randmatrix(1,i) randmatrix(2,i) randmatrix(3,i) 1 0 0 0 0 -(projNorm(1,i)*randmatrix(1:3,i))'] ;
%     Q(2*i,:) =     [0 0 0 0 randmatrix(1,i) randmatrix(2,i) randmatrix(3,i) 1 -(projNorm(2,i)*randmatrix(1:3,i))' ] ;
%     B(2*(i-1),:) = projNorm(1,i);
%     B(2*i,:) =   projNorm(2,i);
    
    Q = [Q;randmatrix(1,i) randmatrix(2,i) randmatrix(3,i) 1 0 0 0 0 -(projNorm(1,i)*randmatrix(1:3,i))'; 0 0 0 0 randmatrix(1,i) randmatrix(2,i) randmatrix(3,i) 1 -(projNorm(2,i)*randmatrix(1:3,i))' ]
    B = [B; projNorm(1,i); projNorm(2,i)]
    
end

  A = Q\B;

%step 7

tr = I * E;
tr = tr(:,:)/tr(3, 4);

% Hallmat and tr are the same;

%step 8- 9
pN = 6; 
std = 0.5; % standard deviation
noisePoints2d = zeros(2, pN);
% function randn gives normal (Gaussian) distributed random numbers;
noisePoints2d(1,:) = projNorm(1,:)+std*randn(1, pN);
noisePoints2d(2,:) = projNorm(2,:)+std*randn(1, pN);

%get new 2d points using noiseA
A(12) = 1;
D = reshape(A,4,3);
A = D'

noiseProj = A*randmatrix;
noiseProj(1,:) = noiseProj(1,:)./noiseProj(3,:);
noiseProj(2,:) = noiseProj(2,:)./noiseProj(3,:);
noiseProj(3,:) = [];
figure;
projFig = scatter(projNorm(1,:), projNorm(2,:), 'bo');
title('Step 8 with 6 points'), hold on;
noiseFig = scatter(noisePoints2d(1,:), noisePoints2d(2,:), 'r+');
noiseProjFig = scatter(noiseProj(1,:), noiseProj(2,:), 'g.'); 
hold off;
legend([projFig,noiseFig,noiseProjFig],'Initial points without noise','Points with noise',...
    'Projected points','NorthEastOutside');

error = mean(sqrt((noiseProj(1,:)-projNorm(1,:)).^2+((noiseProj(2,:)-projNorm(2,:)).^2)));
fprintf('Step 9: Number of points: %d Mean error: %.16f\n',pN,error);

%step 10 

%Q= [];B= [];

  Q = zeros(2*pN, 11);
  B = zeros(2*pN, 1);
    for i = 1:pN
        %Q(2*i-1, :) 
        Q= [Q;randmatrix(i,:) -(projNorm(1,i)*randmatrix(i,:))' 0 0 0 1 0;0 0 0 -(projNorm(2,i)*randmatrix(i,:))' randmatrix(i,:) 0 1];
       % Q(2*i, :) = [0, 0, 0, -projNorm(2,i)*randmatrix(i,:), randmatrix(i,:), 0., 1.];
       % B(2*i-1) = 
       
        B= [B;projNorm(1,i);projNorm(2,i)];
       
    end;
    X = Q\B
    
    T1 = X(1:3)';
    T2 = X(4:6)';
    T3 = X(7:9)';
    C1 = X(10);
    C2 = X(11);
    %compute intrinsic parameters:
    normT2 = norm(T2,2)^2;
    U0 = (T1*T2')/normT2;
    V0 = (T2*T3')/normT2;
    Au = norm(cross(T1',T2'))/normT2;
    Av = norm(cross(T2',T3'))/normT2;
    intrinsics = [Au, 0, U0, 0; 0, Av, V0, 0; 0, 0, 1, 0];
    % intMat and intMat2 are the same;
    %compute extrinsic parameters:
    r1 = (norm(T2)/norm(cross(T1',T2')))*(T1-(T1*T2'/normT2)*T2);
    r2 = (norm(T2)/norm(cross(T2',T3')))*(T3-(T2*T3'/normT2)*T2);
    r3 = T2/norm(T2);
    Tx = (norm(T2)/norm(cross(T1',T2')))*(C1-(T1*T2'/normT2));
    Ty = (norm(T2)/norm(cross(T2',T3')))*(C2-(T2*T3'/normT2));
    Tz = 1/norm(T2);
    extrinsics = [
        r1, Tx;
        r2, Ty;
        r3, Tz;
        0,0,0,1
        ];
    fougerasMatrix = intrinsics*extrinsics;
    fougerasMatrix = fougerasMatrix./fougerasMatrix(3, 4);
    
    %step 11
    
    
std = [0.5, 1.0, 1.5];
for i = 1:pN
    noisePoints2d = zeros(2, pN);
    noisePoints2d(1,:) = projNorm(1,:)+std(i)*randn(1, pN);
    noisePoints2d(2,:) = projNorm(2,:)+std(i)*randn(1, pN);
    [f1] = X(randmatrix, noisePoints2d);
    noiseProj = f1*randmatrix;
    noiseProj(1,:) = noiseProj(1,:)./noiseProj(3,:);
    noiseProj(2,:) = noiseProj(2,:)./noiseProj(3,:);
    noiseProj(3,:) = [];
    error = sqrt((noiseProj(1,:)-projNorm(1,:)).^2+((noiseProj(2,:)-projNorm(2,:)).^2));
    fprintf('Step 11 (Fougeras): Number of points: %d Mean error %.16f (sigma=%f)\n',pN,mean(error),std(i));
    [f2] = A(randmatrix, noisePoints2d);
    noiseProj = f2*randmatrix;
    noiseProj(1,:) = noiseProj(1,:)./noiseProj(3,:);
    noiseProj(2,:) = noiseProj(2,:)./noiseProj(3,:);
    noiseProj(3,:) = [];
    error = sqrt((noiseProj(1,:)-projNorm(1,:)).^2+((noiseProj(2,:)-projNorm(2,:)).^2));
    fprintf('Step 11 (Hall): Number of points: %d Mean error %.16f (sigma=%f)\n',pN,mean(error),std(i));



end



%Part 3
% Step 12
% camera coordinate system origin at 0 0 0;
figure, scatter3(0,0,0,'b.'), hold on, title('World scene simulation'); 
%axis vis3d; % uncomment this to keep aspect ratio
text(0, 0, 0, 'Camera');
% draw x and y axis of camera coord system;
line([0, 150], [0 0], [0 0]); text(155, 0, 0, 'x');
line([0, 0], [0 150], [0 0]); text(0, 155, 0, 'y');
% define world coordinate system axis definition;
WorldCoord = [100 0 0; 0 100 0; 0 0 100; 1 1 1];
% World coordinate origin with respect to camera coord system;
Wcenter= E*[0;0;0;1];
% world coord system with respect to camera
WorldCoord2Cam = E*WorldCoord;
% draw world coordinate system axis and give labels;
line([Wcenter(1) WorldCoord2Cam(1, 1)],[Wcenter(2) WorldCoord2Cam(2, 1)],[Wcenter(3) WorldCoord2Cam(3, 1)]);
line([Wcenter(1) WorldCoord2Cam(1, 2)],[Wcenter(2) WorldCoord2Cam(2, 2)],[Wcenter(3) WorldCoord2Cam(3, 2)]);
line([Wcenter(1) WorldCoord2Cam(1, 3)],[Wcenter(2) WorldCoord2Cam(2, 3)],[Wcenter(3) WorldCoord2Cam(3, 3)]);
text(Wcenter(1), Wcenter(2), Wcenter(3), 'World Coordinate System');
text(WorldCoord2Cam(1, 1), WorldCoord2Cam(2, 1), WorldCoord2Cam(3, 1), 'X');
text(WorldCoord2Cam(1, 2), WorldCoord2Cam(2, 2), WorldCoord2Cam(3, 2), 'Y');
text(WorldCoord2Cam(1, 3), WorldCoord2Cam(2, 3), WorldCoord2Cam(3, 3), 'Z');

% 3D point coordinates with respect to camera;
World2Cam = E*randmatrix;
% plot them;
scatter3(World2Cam(1,:), World2Cam(2,:), World2Cam(3,:), 'ro');

% define image plane with size 640x480
imagePlane = [
        -320, 320, 320, -320, -320;
         240, 240,-240, -240, 240;
         f, f, f, f, f
    ];
% convert from pixels to mm.
ku = -au/f;
kv = -av/f;
imagePlane(1,:) = imagePlane(1,:)/ku;
imagePlane(2,:) = imagePlane(2,:)/kv;
% plot the image plane.
plot3(imagePlane(1,:), imagePlane(2,:), imagePlane(3,:), 'b-');

% projNorm - are the pixels on the image plane.
RD = zeros(2, pN);
RD(1,:) = u0;
RD(2,:) = v0;
% convert 2d point coordinates on image plane into focal point
RD(1,:) = RD(1,:) - projNorm(1,:);
RD(2,:) = RD(2,:) - projNorm(2,:);
% ku = -au/f;
% kv = -av/f;
CU = zeros(3, pN);
% convert 2d coordinates into 3d points with respect to the camera
CU(1,:) = RD(1,:)/ku;
CU(2,:) = RD(2,:)/kv;
CU(3,:) = f;
% plot them
scatter3(CU(1,:), CU(2,:), CU(3,:), 'r.');

% rays from 3d points to camera center
for i = 1:pN
    line([0 World2Cam(1,i)], [0 World2Cam(2,i)], [0 World2Cam(3,i)]);
end;