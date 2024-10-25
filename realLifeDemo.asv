function moveToJointState(q)
    [client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
    goal.Trajectory.JointNames = jointNames;
    goal.Trajectory.Header.Seq = 1;
    goal.Trajectory.Header.Stamp = rostime('Now','system');
    goal.GoalTimeTolerance = rosduration(0.05);
    bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
    durationSeconds = 5; % This is how many seconds the movement will take
    
    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);     
          
    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    nextJointState_123456 = q;
    endJointSend.Positions = nextJointState_123456;
    endJointSend.TimeFromStart = rosduration(durationSeconds);
    
    goal.Trajectory.Points = [startJointSend; endJointSend];

    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    sendGoal(client,goal);
end

rosinit('192.168.27.1'); % If unsure, please ask a tutor
jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');

ur3e = UR3e;

pose1 = transl(0, 0.2, 0.4) * trotx(-90 * pi/180);
q1 = ur3e.model.ikcon(pose1);
ur3e.model.animate(q1);
pause();
moveToJointState(q1);
pause();

pose2 = transl(0.3, 0.2, 0.4) * trotx(-90 * pi/180) * troty(-30 * pi/180);
q2 = ur3e.model.ikcon(pose2);
ur3e.model.animate(q2);
pause();
moveToJointState(q2);
pause();

pose3 = transl(-0.3, 0.2, 0.4) * trotx(-90 * pi/180) * troty(30 * pi/180);
q3 = ur3e.model.ikcon(pose3);
ur3e.model.animate(q3);
moveToJointState(q3);
pause();