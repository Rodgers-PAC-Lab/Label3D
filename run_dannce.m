%% Example setup for Label3D
% Label3D is a GUI for manual labeling of 3D keypoints in multiple cameras. 
% 
% Its main features include:
% 1. Simultaneous viewing of any number of camera views. 
% 2. Multiview triangulation of 3D keypoints.
% 3. Point-and-click and draggable gestures to label keypoints. 
% 4. Zooming, panning, and other default Matlab gestures
% 5. Integration with Animator classes. 
% 6. Support for editing prelabeled data.
% 
% Instructions:
% right: move forward one frameRate
% left: move backward one frameRate
% up: increase the frameRate
% down: decrease the frameRate
% t: triangulate points in current frame that have been labeled in at least two images and reproject into each image
% r: reset gui to the first frame and remove Animator restrictions
% u: reset the current frame to the initial marker positions
% z: Toggle zoom state
% p: Show 3d animation plot of the triangulated points. 
% backspace: reset currently held node (first click and hold, then
%            backspace to delete)
% pageup: Set the selectedNode to the first node
% tab: shift the selected node by 1
% shift+tab: shift the selected node by -1
% h: print help messages for all Animators
% shift+s: Save the data to a .mat file
clear all
close all;
% addpath(genpath('deps'))
% addpath(genpath('skeletons'))
% danncePath = 'Y:/Diego/code/DANNCE';

%% Load in the calibration parameter data
%note--the calibration param files created by the DANNCE example code
%      aren't in the format expected by Label3D. Have to account for this
calibPath = "/home/mouse/mnt/cuttlefish/lucas/calibration_tests/2023-10-25_climbing/cam_params.mat";
numCams = 9;
params = load(calibPath);
cell_params = cell(1,numCams);
for cam = 1:numCams
%     if cam == 3
%         continue
%     elseif cam > 3
%         cam_params = struct;
%         cam_params.K = params.params_individual{cam}.K'; %potential compatibility issue--R2022b expects the transpose;
%         cam_params.RDistort = params.params_individual{cam}.RadialDistortion;
%         cam_params.TDistort = params.params_individual{cam}.TangentialDistortion;
%         cam_params.r = params.rotationMatrix{cam};
%         cam_params.t = params.translationVector{cam};
%         cell_params{cam-1} = cam_params;
% 
%     else
%         cam_params = struct;
%         cam_params.K = params.params_individual{cam}.K'; %potential compatibility issue--R2022b expects the transpose;
%         cam_params.RDistort = params.params_individual{cam}.RadialDistortion;
%         cam_params.TDistort = params.params_individual{cam}.TangentialDistortion;
%         cam_params.r = params.rotationMatrix{cam};
%         cam_params.t = params.translationVector{cam};
%         cell_params{cam} = cam_params;
%     end

    cam_params = struct;
    cam_params.K = params.params_individual{cam}.K'; %potential compatibility issue--R2022b expects the transpose;
    cam_params.RDistort = params.params_individual{cam}.RadialDistortion;
    cam_params.TDistort = params.params_individual{cam}.TangentialDistortion;
    cam_params.r = params.rotationMatrix{cam};
    cam_params.t = params.translationVector{cam};
    cell_params{cam} = cam_params;
end
cell_params = cell_params';

%% Load the videos into memory
vid_dir = "/home/mouse/mnt/cuttlefish/lucas/calibration_tests/2023-10-25_climbing/video_2";
cd(vid_dir)
camera_ids = ["e3v833f" "e3v83e4" "e3v82eb" "e3v83d2" "e3v83d9" "e3v8333" "e3v83c6" "e3v832e" "e3v83d7"];
% camera_ids = ["e3v833f" "e3v83e4" "e3v83d2" "e3v83d9"];
vid_name = "-20231025T101746-103736_transcoded";

% vidPaths = ["e3v833f-20230626T145519-145847.mp4"
%             "e3v83e4-20230626T145519-145847.mp4"
%             "e3v82eb-20230626T145519-145847.mp4"
%             "e3v83d2-20230626T145519-145847.mp4"
%             "e3v83d9-20230626T145519-145847.mp4"];

% vidPaths = ["e3v833f-20230626T145519-145847.mp4"
%             "e3v83e4-20230626T145519-145847.mp4"
%             "e3v83d2-20230626T145519-145847.mp4"
%             "e3v83d9-20230626T145519-145847.mp4"];

height = 720;
width = 1280;
channel = 3;
framesToLabel = 10;
spacing = 1000; %label every 1000 frames
%frame*1000, so frame idx are 1000,2000,3000...20000
videos = cell(1,numCams);

%Read in the number of frames to label and undistort for each view
%very slow code, need to refactor
%video needs to be in format 'uint8' with size (height,width,channel,N), and they must be stored in a
%cell array

for nVid = 1:numel(camera_ids)
    vidPath = camera_ids(nVid)+vid_name+".mp4";
    v = VideoReader(vidPath);
    u_video = zeros(height,width,channel,framesToLabel, 'uint8');
    for f = 1:framesToLabel
        v.CurrentTime = f*spacing/v.FrameRate;
        d_frame = readFrame(v);
        u_video(:,:,:,f) = undistortImage(d_frame, params.params_individual{nVid});
    end
    videos{nVid} = u_video;
end

%% Get the skeleton
label3D_dir = "/home/mouse/dev/Label3D"; %working directory for label3d to save files
cd(label3D_dir)
skeleton = load('skeletons/mouse22.mat');
% skeleton = load('com');

%% Start Label3D
close all
label_wd = "/home/mouse/mnt/cuttlefish/lucas/calibration_tests/2023-10-25_climbing";
cd(label_wd)
labelGui = Label3D(cell_params, videos, skeleton);
% labelGui = Label3D(params, videos, skeleton, 'sync', sync, 'framesToLabel', framesToLabel);
labelGui = Label3D("20231214_Label3D.mat", videos);

%% Export to DANNCE
basePath = '/home/mouse/mnt/cuttlefish/lucas/f2_dannce/session0';
cameraNames = {'Camera1', 'Camera2', 'Camera3', 'Camera4', 'Camera5', 'Camera6', 'Camera7', 'Camera8', 'Camera9'};
filePath = '/home/mouse/mnt/cuttlefish/lucas/f2_dannce/session0/labels/20231214_Label3D.mat';
ftl_vec = linspace(spacing,framesToLabel*spacing,framesToLabel);
labelGui.exportDannce('basePath', basePath,'cameraNames', cameraNames,'framesToLabel',ftl_vec,'saveFolder', basePath)

%% Check the camera positions
labelGui.plotCameras       

%% If you just wish to view labels, use View 3D
close all
viewGui = View3D(params, videos, skeleton);

%% You can load both in different ways
close all;
View3D()