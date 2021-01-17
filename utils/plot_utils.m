# Plots
1;

# Plot odometry and ground truth
function plotOdometryAndGT(traj_meas, traj_gt)
  subplot(2,1,1)
  plot(traj_meas(:,1), traj_meas(:,2))
  title("Measurements")
  subplot(2,1,2)
  plot(traj_gt(:,1), traj_gt(:,2))
  title("Ground Truth")
endfunction
