
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