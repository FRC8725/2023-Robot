package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnOpposite extends CommandBase {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private double targetPosition;
    ProfiledPIDController controller;
    public TurnOpposite() {
        addRequirements(swerveSubsystem);
        controller = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    }
    @Override
    public void initialize() {
        controller.setGoal(swerveSubsystem.getHeading()+Math.PI);
    }

    @Override
    public void execute() {
        var turningSpeed = controller.calculate(swerveSubsystem.getHeading());
        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(controller.getGoal().position - swerveSubsystem.getRotation2d().getRadians()) < Units.degreesToRadians(3);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}
