package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;


public class CorrectPositionReflectiveTape2d extends CommandBase {

  final private SwerveSubsystem swerveSubsystem;
  final private ProfiledPIDController xController, yController;


  final private VisionManager visionManager;

  public CorrectPositionReflectiveTape2d(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    // Controller Settings
    xController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, AutoConstants.kDriveControllerConstraints);
    yController = new ProfiledPIDController(AutoConstants.kPYController, 0, 0, AutoConstants.kDriveControllerConstraints);
    xController.setTolerance(.2);
    yController.setTolerance(.2);

    this.visionManager = visionManager;
  }

  @Override
  public void initialize() {
    xController.reset(swerveSubsystem.getPose().getX());
    yController.reset(swerveSubsystem.getPose().getY());
    visionManager.getReflectiveTapeRelative2d();
    visionManager.setDriverMode(false);
  }

  @Override
  public void execute() {
    Transform2d relativePos = visionManager.getReflectiveTapeRelative2d();
    if (!visionManager.hasTarget()) {
      swerveSubsystem.stopModules();
      return;
    }
    var robotPose = new Pose2d(
            swerveSubsystem.getPose().getX(),
            swerveSubsystem.getPose().getY(),
            new Rotation2d(swerveSubsystem.getPose().getRotation().getRadians())
    );
    var camPose = robotPose.transformBy(new Transform2d(VisionConstants.Robot2Photon.getTranslation().toTranslation2d(), new Rotation2d()));
    var targetPose = camPose.transformBy(relativePos);
    Transform2d tag2goal = new Transform2d(VisionConstants.Tag2Goal.getTranslation().toTranslation2d(), new Rotation2d())
            .plus(new Transform2d(new Translation2d(-5, 0), new Rotation2d()));

    var goalPose = targetPose.transformBy(tag2goal);

    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());

    var xSpeed = xController.calculate(robotPose.getX());
    var ySpeed = yController.calculate(robotPose.getY());

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, 0, swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    visionManager.setDriverMode(true);
    swerveSubsystem.stopModules();
  }
}