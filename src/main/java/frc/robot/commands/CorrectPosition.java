package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;


public class CorrectPosition extends CommandBase {

    private SwerveSubsystem swerveSubsystem;
    private ProfiledPIDController xController, yController, thetaController;
    private Limelight limelight;
    private final int whereChase;
    private double socialDistanceM;

    private Pose2d lastTarget;

    public CorrectPosition(int whereChase, double socialDistanceM) {
        // 0 stand for left side
        // 1 stand for middle
        // 2 stand for right side
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.whereChase = whereChase;
        this.socialDistanceM = socialDistanceM;
        addRequirements(swerveSubsystem);

        // Controller Settings
        xController = new ProfiledPIDController(AutoConstants.kCorrectPositionXController, 0, 0, AutoConstants.kDriveControllerConstraints);
        yController = new ProfiledPIDController(AutoConstants.kCorrectPositionYController, 0, 0, AutoConstants.kDriveControllerConstraints);
        thetaController = new ProfiledPIDController(
                AutoConstants.kCorrectPositionThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        xController.setTolerance(.1);
        yController.setTolerance(.1);
        thetaController.setTolerance(2);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.limelight = Limelight.getInstance();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) {swerveSubsystem.stopModules();return;}
        lastTarget = limelight.getAprilTagRelative().get();
        xController.setGoal(-socialDistanceM-DriveConstants.kTrackWidth/2);
        yController.setGoal(0);
        thetaController.setGoal(0);
        var xSpeed = xController.calculate(-lastTarget.getX());
        double ySpeed;
        switch (whereChase) {
            case 0:
                ySpeed = yController.calculate(lastTarget.getY()-VisionConstants.yoffset);
                break;
            case 2:
                ySpeed = yController.calculate(lastTarget.getY()+VisionConstants.yoffset);
                break;
            default:
                ySpeed = yController.calculate(lastTarget.getY());
        }

        var turningSpeed = thetaController.calculate(lastTarget.getRotation().getRadians());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());


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
        swerveSubsystem.stopModules();
    }
}
