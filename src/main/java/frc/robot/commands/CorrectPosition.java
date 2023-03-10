package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;


public class CorrectPosition extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    final private ProfiledPIDController xController, yController, thetaController;

    final private VisionManager visionManager;
    private final String whereChase;

    private Pose2d lastGoalPose;

    NetworkTable led_nt = NetworkTableInstance.getDefault().getTable("LEDs");
    BooleanPublisher isLimelightPub = led_nt.getBooleanTopic("isLimelight").publish();
    // "left" stand for left side
    // "middle" stand for middle
    // "right" stand for right side

    public CorrectPosition(String whereChase, VisionManager visionManager) {
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.whereChase = whereChase;
        addRequirements(swerveSubsystem);

        // Controller Settings
        xController = new ProfiledPIDController(AutoConstants.CORRECT_POSITION_X_CONTROLLER, 0, 0, AutoConstants.DRIVE_CONTROLLER_CONSTRAINTS);
        yController = new ProfiledPIDController(AutoConstants.CORRECT_POSITION_Y_CONTROLLER, 0, 0, AutoConstants.DRIVE_CONTROLLER_CONSTRAINTS);
        thetaController = new ProfiledPIDController(
                AutoConstants.CORRECT_POSITION_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
//        xController.setTolerance(.2);
//        yController.setTolerance(.2);
//        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.visionManager = visionManager;
    }

    @Override
    public void initialize() {
        xController.reset(swerveSubsystem.getPose().getX());
        yController.reset(swerveSubsystem.getPose().getY());
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians());
        lastGoalPose = new Pose2d();
        isLimelightPub.set(true);
//        led_nt.putValue("isLimelight", NetworkTableValue.makeBoolean(true));
    }

    @Override
    public void execute() {
        var relativePos = visionManager.getAprilTagRelative();

        var robotPose = new Pose3d(
                swerveSubsystem.getPose().getX(),
                swerveSubsystem.getPose().getY(),
                0.0,
                new Rotation3d(0, 0, swerveSubsystem.getPose().getRotation().getRadians())
        );
//        var camPose = robotPose.transformBy(VisionConstants.Robot2Photon);
        var targetPose = robotPose.transformBy(relativePos);
        Transform3d tag2goal;

        // Change the place we want to go.
        switch (whereChase.toLowerCase()) {
            case "left":
                tag2goal = VisionConstants.Tag2Goal.plus(VisionConstants.GoalMid2Left);
                break;
            case "right":
                tag2goal = VisionConstants.Tag2Goal.plus(VisionConstants.GoalMid2Right);
                break;
            case "single":
                tag2goal = DriverStation.getAlliance() == DriverStation.Alliance.Red? VisionConstants.Tag2Single_Red: VisionConstants.Tag2Single_Blue;
                break;
            default:
                tag2goal = VisionConstants.Tag2Goal;
        }

        Pose2d goalPose;
        if (visionManager.hasTarget()) goalPose = targetPose.transformBy(tag2goal).toPose2d();
        else goalPose = lastGoalPose;

        if (goalPose.getX() == 0 && goalPose.getY() == 0) return;
        else lastGoalPose = goalPose;

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        thetaController.setGoal(goalPose.getRotation().getRadians());

        var xSpeed = xController.calculate(robotPose.getX());
        var ySpeed = yController.calculate(robotPose.getY());
        var turningSpeed = thetaController.calculate(swerveSubsystem.getPose().getRotation().getRadians());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());


        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        isLimelightPub.set(false);
        swerveSubsystem.stopModules();
    }
}
