%% 
clear; clc; close all;

%%

%Part 1: estimation of the fundamental matrix with manually selected correspondences

% Load images
img1 = imread('Rubik/Rubik1.pgm');
img2 = imread('Rubik/Rubik2.pgm');

% Load points
P1orig = load('Rubik/Rubik1.points');
P2orig = load('Rubik/Rubik2.points');

%P1orig = P1orig(1:10,:);
%P2orig = P2orig(1:10,:);

n = size(P1orig,1);

% Add the third component to work in homogeneous coordinates
P1 = [P1orig'; ones(1,n)];
P2 = [P2orig'; ones(1,n)];

% Estimate the fundamental matrix
F = EightPointsAlgorithmN(P1, P2);

% Visualize the epipolar lines
visualizeEpipolarLines(img1, img2, F, P1orig, P2orig, 100);
pause 
%close all
visualizeEpipolarLines(img1, img2, F, [], [], 110);

%% CHECK EPIPOLAR CONTRAINT
verified = check_epipolar_constraint(F, P1, P2);
display("VERIFY EPIPOLAR CONDITION: ");
display(verified);

%EPIPOLES
[e1,e2] = epipoles(F);
% Display the computed epipoles
disp("Epipole 1: ");
disp(e1);
disp("Epipole 2: ");
disp(e2);

%CHECKING EPIPOLAR 
ep_verified = check_epipole(F,e1);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %0

ep_verified = check_epipole(F,e2);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %1

ep_verified = check_epipole(F',e1);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %1

ep_verified = check_epipole(F',e2);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %0

%% Part 2: assessing the use of RANSAC 
clc, clear all;

% Load images
img1 = imread('Rubik/Rubik1.pgm');
img2 = imread('Rubik/Rubik2.pgm');

% Load points
P1orig = load('Rubik/Rubik1.points');
P2orig = load('Rubik/Rubik2.points');

% Add random points (to assess RANSAC)
x1r = double(round(size(img1,1)*rand(5,1)));
y1r = double(round(size(img1,2)*rand(5,1)));

x2r = double(round(size(img2,1)*rand(5,1)));
y2r = double(round(size(img2,2)*rand(5,1)));

P1orign = [P1orig; [x1r, y1r]];
P2orign = [P2orig; [x2r, y2r]];

n = size(P1orign,1);

% Add the third component to work in homogeneous coordinates
P1 = [P1orign'; ones(1,n)];
P2 = [P2orign'; ones(1,n)];

% Estimate the fundamental matrix with RANSAC
%prende subset di punti falsi e veri
%fa degli INSIEMI di coppie e si calcola ogni F.
% CALCOLA UNO SCORE E RESTITUISCE LA F MIGLIORE TRA LE COPPIE DI PUNTI

th = 10^(-2);
[F, consensus, outliers] = ransacF(P1, P2, th);

% Visualize the epipolar lines
visualizeEpipolarLines(img1, img2, F, P1orig, P2orig, 120);

%%  check 
%CHECK EPIPOLAR CONTRAINT
verified = check_epipolar_constraint(F, P1, P2);
display("VERIFY EPIPOLAR CONDITION: ");
display(verified);

%EPIPOLES
[e1,e2] = epipoles(F);
% Display the computed epipoles
disp("Epipole 1: ");
disp(e1);
disp("Epipole 2: ");
disp(e2);

%CHECKING EPIPOLAR 
ep_verified = check_epipole(F,e1);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %0

ep_verified = check_epipole(F,e2);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %1

ep_verified = check_epipole(F',e1);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %1

ep_verified = check_epipole(F',e2);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %0



%% Part 3: using image matching+ransac
clc, close all, clear all;
%addpath('../ImageMatching/'); % change the path here if needed
addpath('../../Lab7/Material/ImageMatching/');

% Load images
img1 = rgb2gray(imread('OtherPairs/robot1.jpeg'));
img2 = rgb2gray(imread('OtherPairs/robot2.jpeg'));

img1 = imresize(img1, 0.5);
img2 = imresize(img2, 0.5);

% extraction of keypoints and matching
list = imageMatching(img1, img2, 'POS', 0.85, 0, 10);

n = size(list,1);

% Add the third component to work in homogeneous coordinates
P1 = [list(:,2)'; list(:,1)'; ones(1,n)];
P2 = [list(:,4)'; list(:,3)'; ones(1,n)];

% Estimate the fundamental matrix with RANSAC
th = 10^(-2);
[F, consensus, outliers] = ransacF(P1, P2, th);

% Visualize the epipolar lines
visualizeEpipolarLines(img1, img2, F, P1(1:2,:)', P2(1:2,:)', 130);

%-------------------
% img1 = rgb2gray(imread('OtherPairs/chiavi1.jpeg'));
% img2 = rgb2gray(imread('OtherPairs/chiavi2.jpeg'));
% 
% img1 = imresize(img1, 0.5);
% img2 = imresize(img2, 0.5);
% 
% % extraction of keypoints and matching
% list = imageMatching(img1, img2, 'NCC', 0.85, 0, 100);
% 
% n = size(list,1);
% 
% % Add the third component to work in homogeneous coordinates
% P1 = [list(:,2)'; list(:,1)'; ones(1,n)];
% P2 = [list(:,4)'; list(:,3)'; ones(1,n)];
% 
% % Estimate the fundamental matrix with RANSAC
% th = 10^(-2);
% [F, consensus, outliers] = ransacF(P1, P2, th);
% 
% % Visualize the epipolar lines
% visualizeEpipolarLines(img1, img2, F, P1(1:2,:)', P2(1:2,:)', 130);
% 
% %-------------------
% 
% img1 = rgb2gray(imread('OtherPairs/foto1.jpeg'));
% img2 = rgb2gray(imread('OtherPairs/foto2.jpeg'));
% 
% img1 = imresize(img1, 0.5);
% img2 = imresize(img2, 0.5);
% 
% % extraction of keypoints and matching
% list = imageMatching(img1, img2, 'POSNCC', 0.65, 1, 100);
% 
% n = size(list,1);
% 
% % Add the third component to work in homogeneous coordinates
% P1 = [list(:,2)'; list(:,1)'; ones(1,n)];
% P2 = [list(:,4)'; list(:,3)'; ones(1,n)];
% 
% % Estimate the fundamental matrix with RANSAC
% th = 10^(-2);
% [F, consensus, outliers] = ransacF(P1, P2, th);
% 
% % Visualize the epipolar lines
% visualizeEpipolarLines(img1, img2, F, P1(1:2,:)', P2(1:2,:)', 130);
% 
% %-------------------
% 
% img1 = rgb2gray(imread('OtherPairs/robot1.jpeg'));
% img2 = rgb2gray(imread('OtherPairs/robot2.jpeg'));
% 
% img1 = imresize(img1, 0.5);
% img2 = imresize(img2, 0.5);
% 
% % extraction of keypoints and matching
% list = imageMatching(img1, img2, 'POSNCC', 0.65, 0, 10);
% 
% n = size(list,1);
% 
% % Add the third component to work in homogeneous coordinates
% P1 = [list(:,2)'; list(:,1)'; ones(1,n)];
% P2 = [list(:,4)'; list(:,3)'; ones(1,n)];
% 
% % Estimate the fundamental matrix with RANSAC
% th = 10^(-2);
% [F, consensus, outliers] = ransacF(P1, P2, th);
% 
% % Visualize the epipolar lines
% visualizeEpipolarLines(img1, img2, F, P1(1:2,:)', P2(1:2,:)', 130);
% 
% %-------------------
% 
% img1 = rgb2gray(imread('OtherPairs/chiavi1.jpeg'));
% img2 = rgb2gray(imread('OtherPairs/chiavi2.jpeg'));
% 
% img1 = imresize(img1, 0.5);
% img2 = imresize(img2, 0.5);
% 
% % extraction of keypoints and matching
% list = imageMatching(img1, img2, 'POSNCC', 0.65, 1, 100);
% 
% n = size(list,1);
% 
% % Add the third component to work in homogeneous coordinates
% P1 = [list(:,2)'; list(:,1)'; ones(1,n)];
% P2 = [list(:,4)'; list(:,3)'; ones(1,n)];
% 
% % Estimate the fundamental matrix with RANSAC
% th = 10^(-2);
% [F, consensus, outliers] = ransacF(P1, P2, th);
% 
% % Visualize the epipolar lines
% visualizeEpipolarLines(img1, img2, F, P1(1:2,:)', P2(1:2,:)', 130);

%%  check 
%CHECK EPIPOLAR CONTRAINT
verified = check_epipolar_constraint(F, P1, P2);
display("VERIFY EPIPOLAR CONDITION: ");
display(verified);

%EPIPOLES
[e1,e2] = epipoles(F);
% Display the computed epipoles
disp("Epipole 1: ");
disp(e1);
disp("Epipole 2: ");
disp(e2);

%CHECKING EPIPOLAR 
ep_verified = check_epipole(F,e1);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %0

ep_verified = check_epipole(F,e2);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %1

ep_verified = check_epipole(F',e1);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %1

ep_verified = check_epipole(F',e2);
display("VERIFY EPIPOLE CONDITION: ");
display(ep_verified); %0
