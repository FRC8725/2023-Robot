package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;


public class CorrectPosition extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final ProfiledPIDController xController, yController;
    private final ProfiledPIDController thetaController;
    private final Limelight limelight;
    private final int whereChase;
    private final double socialDistanceM;
    private double ydistance;

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
        xController = new ProfiledPIDController(AutoConstants.CORRECT_POSITION_X_CONTROLLER, 0, 0, AutoConstants.DRIVE_CONTROLLER_CONSTRAINTS);
        yController = new ProfiledPIDController(AutoConstants.CORRECT_POSITION_Y_CONTROLLER, 0, 0, AutoConstants.DRIVE_CONTROLLER_CONSTRAINTS);
        thetaController = new ProfiledPIDController(
                AutoConstants.CORRECT_POSITION_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        xController.setTolerance(.1);
        yController.setTolerance(.1);
        thetaController.setTolerance(2);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.limelight = Limelight.getInstance();
    }

    @Override
    public void initialize() {
        if (limelight.getAprilTagRelative().isPresent()) lastTarget = limelight.getAprilTagRelative().get();
    }

    @Override
    public void execute() {
        xController.setGoal(-socialDistanceM-DriveConstants.TRACK_WIDTH / 2 + swerveSubsystem.getPose().getX());
        yController.setGoal(swerveSubsystem.getPose().getY());
        thetaController.setGoal(swerveSubsystem.getRotation2d().getRadians());
        var xSpeed = xController.calculate(-lastTarget.getX() + swerveSubsystem.getPose().getX());
        var turningSpeed = .0;
        double ySpeed;
        switch (whereChase) {
            case 0:
                ydistance = lastTarget.getY()-VisionConstants.Y_OFFSET + swerveSubsystem.getPose().getY();
                ySpeed = yController.calculate(ydistance);
                break;
            case 2:
                ydistance = lastTarget.getY()+VisionConstants.Y_OFFSET + swerveSubsystem.getPose().getY();
                ySpeed = yController.calculate(ydistance);
                break;
            case 3 :
                xSpeed = 0;
                ySpeed = 0;
                turningSpeed = thetaController.calculate(lastTarget.getRotation().getRadians() + swerveSubsystem.getRotation2d().getRadians());
            default:
                ydistance = lastTarget.getY() + swerveSubsystem.getPose().getY();
                ySpeed = yController.calculate(ydistance);
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());


        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Math.abs(xController.getGoal().position + lastTarget.getX()) < 0.2 && Math.abs(yController.getGoal().position - ydistance) < 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}
