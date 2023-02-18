// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;


import java.util.Optional;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final DoubleArraySubscriber robotToTagSubscriber;
  private final DoubleArraySubscriber robotPoseSubscriber;
  private final DoubleSubscriber tagIDSubscriber;
  private static final Limelight INSTANCE = new Limelight();

  public Limelight() {
    robotToTagSubscriber = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
    robotPoseSubscriber = NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("botpose").subscribe(new double[6]);
    tagIDSubscriber = NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tid").subscribe(-1);
    if (DriverStation.getAlliance() == Alliance.Blue) {
      NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[6]);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getDoubleArrayTopic("botpose_wpired").subscribe(new double[6]);
    }
  }

  public static Limelight getInstance() {
    return INSTANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumberArray("robot Pose from limelight", robotPoseSubscriber.get());
  }

  public Optional<Pose2d> getAprilTagRelative() {
    var robotPoseArray = robotToTagSubscriber.get();
    Optional<Pose2d> CameraToTarget = Optional.empty();
    if (hasTarget()) {
      CameraToTarget = Optional.of(new Pose2d(robotPoseArray[2], robotPoseArray[0], new Rotation2d(Units.degreesToRadians(robotPoseArray[4]))));
    }
    return CameraToTarget;
  }

  public Optional<Pair<Pose2d, Double>> getEstimatedGlobalPose() {
    var robotPoseArray = robotPoseSubscriber.get();
    if (!hasTarget()) {
      return Optional.empty();
    } else {
      var robotPose2d = new Pose2d(robotPoseArray[0] + FieldConstants.length / 2, robotPoseArray[1] + FieldConstants.width / 2 , new Rotation2d(Units.degreesToRadians(robotPoseArray[5])));
      return Optional.of(new Pair<>(robotPose2d, robotPoseArray[6]));
    }
  }

  public boolean hasTarget() {
    return tagIDSubscriber.get() != -1;
  }
}
