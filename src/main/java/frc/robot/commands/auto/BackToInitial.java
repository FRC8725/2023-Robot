package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class BackToInitial extends CommandBase {

    final SwerveSubsystem swerveSubsystem;

    final ProfiledPIDController xController, yController, thetaController;
    final Pose2d pathInitialPose;

    public BackToInitial(SwerveSubsystem swerveSubsystem, Pose2d pathInitialPose) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        this.pathInitialPose = pathInitialPose;

        // Controller Settings
        xController = new ProfiledPIDController(AutoConstants.PX_CONTROLLER, 0, 0, AutoConstants.DRIVE_CONTROLLER_CONSTRAINTS);
        yController = new ProfiledPIDController(AutoConstants.PY_CONTROLLER, 0, 0, AutoConstants.DRIVE_CONTROLLER_CONSTRAINTS);
        thetaController = new ProfiledPIDController(
                AutoConstants.PTHETA_CONTROLLER *.6, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
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
        var robotPose = swerveSubsystem.getPose();

            xController.setGoal(pathInitialPose.getX());
            yController.setGoal(pathInitialPose.getY());
            thetaController.setGoal(pathInitialPose.getRotation().getRadians());

        var xSpeed = xController.calculate(robotPose.getX());
        var ySpeed = yController.calculate(robotPose.getY());
        var turningSpeed = thetaController.calculate(robotPose.getRotation().getRadians());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());


        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return swerveSubsystem.getPose().getTranslation().getDistance(pathInitialPose.getTranslation()) < 0.5;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
