close all
clear all
clc

# Read the trajectory (id, measurements ,ground truth)
fid = fopen("./Dataset/trajectory.dat", 'r');
trajectoryData = textscan(fid, "%*s %d %f %f %f %f %f %f");

trajMeas = cell2mat(trajectoryData(1,2:4))
subplot(2,1,1)
plot(trajMeas(:,1), trajMeas(:,2))
title("Measurements")

trajGT = cell2mat(trajectoryData(1,5:7))
subplot(2,1,2)
plot(trajGT(:,1), trajGT(:,2))
title("Ground Truth")