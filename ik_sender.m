
joint_angles_r = joint_angles .* [-1,-1,-1,-1,-1];

[rArm, rGoalMsg] = rosactionclient('/arm_1/arm_controller/follow_joint_trajectory');

rGoalMsg.Trajectory.JointNames = {'arm_joint_1', ...
                                   'arm_joint_2', ...
                                   'arm_joint_3', ...
                                   'arm_joint_4',...
                                   'arm_joint_5'};
% Point 1

tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint1.Positions = joint_angles_r;
tjPoint1.Velocities = zeros(1,5);
tjPoint1.TimeFromStart = rosduration(1.0);


rGoalMsg.Trajectory.Points = [tjPoint1];

sendGoalAndWait(rArm,rGoalMsg);
pause(2); % Kısa süre bekle, eklemin hareketi tamamlaması için


% joint_states topic'i oku
jointStateMsg = rossubscriber('/arm_1/arm_controller/state');
feedback = receive(jointStateMsg, 3);  % 3 saniye timeout

% Actual positions'u çek
actual_positions = feedback.Actual.Positions;

% Gönderilen joint_angles ile actual_positions farkı
joint_error = joint_angles - transpose(actual_positions) ;

% Sonuçları konsola yazdır
disp('Joint angle farkları (hedef - actual):');
disp(joint_error);