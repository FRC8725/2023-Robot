// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoubleSubscriber;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  final private DoubleArraySubscriber robotPoseSubscriber;
  final private DoubleSubscriber tagIDSubscriber;
  final static Limelight INSTANCE = new Limelight();
  public static Limelight getInstance() {
    return INSTANCE;
  }
  public Limelight() {
    robotPoseSubscriber = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
    tagIDSubscriber = NetworkTableInstance.getDefault().getTable("Limelight").getDoubleTopic("tid").subscribe(-1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumberArray("robot Pose from limelight", robotPoseSubscriber.get());
  }

  public Optional<Transform3d> getAprilTagRelative() {
    var robotPoseArray = robotPoseSubscriber.get();
    Optional<Transform3d> CameraToTarget = Optional.empty();
    if (tagIDSubscriber.get() != -1) {
      CameraToTarget = Optional.of(new Transform3d(new Translation3d(robotPoseArray[0], robotPoseArray[1], robotPoseArray[2]), new Rotation3d(robotPoseArray[3], robotPoseArray[4], robotPoseArray[5])));
    }
    return CameraToTarget;
  }

  public Optional<Pair<Pose2d, Double>> getEstimatedGlobalPose() {
    double currentTime = Timer.getFPGATimestamp();
    var robotPoseArray = robotPoseSubscriber.get();
    if(tagIDSubscriber.get() == -1) {
      return Optional.empty();
    } else {
      var robotPose2d = new Pose2d(robotPoseArray[0], robotPoseArray[1], new Rotation2d(robotPoseArray[2], robotPoseArray[3]));
      return Optional.of(new Pair<Pose2d, Double>(robotPose2d, Timer.getFPGATimestamp()));
    }
  }
}
