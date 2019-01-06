%% demo.m
%Read OBJ files, Plot the point cloud, run ICP and compare the results.

close all
clear all
clc

%Initialize confiuration
addpath('config/');
config;

%Read the obj files
Neutral = Lecture_fichier(path_neutral,m);
Smile = Lecture_fichier(path_smile,m);


n = size(Neutral,2);
%% 
%Plot the surfaces obtaineed after sampling
Afficher(Smile,Neutral);

%Use ICP
[Ricp Ticp ER t] = icp(Smile, Neutral, k, 'Matching', 'kDtree','Extrapolation', true);
% Transform data-matrix using ICP result
Smileicp = Ricp * Neutral + repmat(Ticp, 1, n);

%Compare the 2 Point cloud
Comparer(Smile,Smileicp,k,ER,t);




