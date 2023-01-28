package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;


public class CorrectPositionReflectiveTape extends CommandBase {

  final private SwerveSubsystem swerveSubsystem;
  final private PIDController yController;


  final private VisionManager visionManager;

  public CorrectPositionReflectiveTape(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    // Controller Settings
    yController = new PIDController(1.5 * Math.PI, 0, 0);
    yController.setTolerance(.2);
    yController.enableContinuousInput(-Math.PI, Math.PI);

    this.visionManager = visionManager;
  }

  @Override
  public void initialize() {
    yController.reset();
    visionManager.getReflectiveTapeRelativeYawRads();
    visionManager.setDriverMode(false);
    Timer.delay(.8);
  }

  @Override
  public void execute() {
    double relativeRads = visionManager.getReflectiveTapeRelativeYawRads();
    if (!visionManager.hasTarget()) {
      swerveSubsystem.stopModules();
      return;
    }

    yController.setSetpoint(relativeRads);

    var ySpeed = yController.calculate(0);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0, ySpeed, 0, swerveSubsystem.getRotation2d());

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