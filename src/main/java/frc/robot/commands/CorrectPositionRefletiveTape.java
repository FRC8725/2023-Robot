package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;
import org.photonvision.targeting.PhotonTrackedTarget;


public class CorrectPositionRefletiveTape extends CommandBase {

    private SwerveSubsystem swerveSubsystem;
    private ProfiledPIDController xController, yController, thetaController;

    private PhotonTrackedTarget lastTarget;

    private VisionManager visionManager;

    public CorrectPositionRefletiveTape(SwerveSubsystem swerveSubsystem, VisionManager visionManager) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        // Controller Settings
        xController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, AutoConstants.kDriveControllerConstraints);
        yController = new ProfiledPIDController(AutoConstants.kPYController, 0, 0, AutoConstants.kDriveControllerConstraints);
        thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        xController.setTolerance(.2);
        yController.setTolerance(.2);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.visionManager = visionManager;
    }

    @Override
    public void initialize() {
        xController.reset(swerveSubsystem.getPose().getX());
        yController.reset(swerveSubsystem.getPose().getY());
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians());
        visionManager.setLED(true);
    }

    @Override
    public void execute() {
        Transform2d relativePos = visionManager.getReflectiveTapeRelative();
        var robotPose = new Pose2d(
                swerveSubsystem.getPose().getX(),
                swerveSubsystem.getPose().getY(),
                new Rotation2d(swerveSubsystem.getPose().getRotation().getRadians())
        );
        var camPose = robotPose.transformBy(
                new Transform2d(VisionConstants.Robot2Photon.getTranslation().toTranslation2d(), new Rotation2d()));
        var targetPose = camPose.transformBy(relativePos);
        Transform2d tag2goal = new Transform2d(VisionConstants.Tag2Goal.getTranslation().toTranslation2d(), new Rotation2d())
                .plus(new Transform2d(new Translation2d(-1, 0), new Rotation2d()));

        var goalPose = targetPose.transformBy(tag2goal);

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
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !visionManager.hasTarget();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        visionManager.setLED(false);
    }
}
