package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
    private final int whereChase;

    private Transform3d lastTarget;
    // 0 stand for left side
    // 1 stand for middle
    // 2 stand for right side

    public CorrectPosition(int whereChase, VisionManager visionManager) {
        // 0 stand for left side
        // 1 stand for middle
        // 2 stand for right side
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.whereChase = whereChase;
        addRequirements(swerveSubsystem);

        // Controller Settings
        xController = new ProfiledPIDController(AutoConstants.P_X_CONTROLLER, 0, 0, AutoConstants.DRIVE_CONTROLLER_CONSTRAINTS);
        yController = new ProfiledPIDController(AutoConstants.P_Y_CONTROLLER, 0, 0, AutoConstants.DRIVE_CONTROLLER_CONSTRAINTS);
        thetaController = new ProfiledPIDController(
                AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
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
        visionManager.getAprilTagRelative();
        lastTarget = new Transform3d();
        Timer.delay(.8);
    }

    @Override
    public void execute() {
        var relativePos = visionManager.getAprilTagRelative();
        if (!visionManager.hasTarget()) relativePos = lastTarget;
        else lastTarget = relativePos;

        if (relativePos.getX() == 0 && relativePos.getY() == 0 && relativePos.getZ() == 0) return;

        var robotPose = new Pose3d(
                swerveSubsystem.getPose().getX(),
                swerveSubsystem.getPose().getY(),
                0.0,
                new Rotation3d(0, 0, swerveSubsystem.getPose().getRotation().getRadians())
        );
        var camPose = robotPose.transformBy(VisionConstants.Robot2Photon);
        var targetPose = camPose.transformBy(relativePos);
        Transform3d tag2goal = new Transform3d();

        // Change the place we want to go.
        switch (whereChase) {
            case 0:
                tag2goal = VisionConstants.Tag2Goal.plus(VisionConstants.GoalMid2Left);
                break;
            case 2:
                tag2goal = VisionConstants.Tag2Goal.plus(VisionConstants.GoalMid2Right);
                break;
            default:
                tag2goal = VisionConstants.Tag2Goal;
        }

        var goalPose = targetPose.transformBy(tag2goal).toPose2d();

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
        swerveSubsystem.stopModules();
    }
}
