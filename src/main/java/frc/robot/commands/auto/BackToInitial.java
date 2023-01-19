package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.VisionManager;
import frc.robot.Constants.DriveConstants;


public class BackToInitial extends CommandBase {

    SwerveSubsystem swerveSubsystem;

    ProfiledPIDController xController, yController, thetaController;
    VisionManager visionManager;
    Pose2d initialPose;

    public BackToInitial(SwerveSubsystem swerveSubsystem, Pose2d initialPose) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        this.initialPose = initialPose;

        // Controller Settings
        xController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, AutoConstants.kDriveControllerConstraints);
        yController = new ProfiledPIDController(AutoConstants.kPYController, 0, 0, AutoConstants.kDriveControllerConstraints);
        thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        xController.setTolerance(.2);
        yController.setTolerance(.2);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements();
    }

    @Override
    public void initialize() {
        xController.reset(swerveSubsystem.getPose().getX());
        yController.reset(swerveSubsystem.getPose().getY());
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Transform2d relativePos = visionManager.getReflectiveTapeRelative();
        var robotPose = swerveSubsystem.getPose();
        var goalPose = robotPose.transformBy(
                new Transform2d(initialPose.getTranslation(), new Rotation2d()));

            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            thetaController.setGoal(goalPose.getRotation().getRadians());

        var xSpeed = xController.calculate(robotPose.getX());
        var ySpeed = yController.calculate(robotPose.getY());
        var turningSpeed = thetaController.calculate(swerveSubsystem.getPose().getRotation().getRadians());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());


        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        double distance = swerveSubsystem.getPose().getTranslation().getDistance(initialPose.getTranslation());
        // TODO: Make this return true when this Command no longer needs to run execute()
        return distance < 1;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
