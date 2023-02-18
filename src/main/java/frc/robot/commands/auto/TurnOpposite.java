package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnOpposite extends CommandBase {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final ProfiledPIDController controller;

    public TurnOpposite() {
        this.addRequirements(this.swerveSubsystem);
        this.controller = new ProfiledPIDController(Constants.AutoConstants.PTHETA_CONTROLLER, 0, 0, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
    }

    @Override
    public void initialize() {
        this.controller.setGoal(this.swerveSubsystem.getHeading()+Math.PI);
    }

    @Override
    public void execute() {
        double turningSpeed = this.controller.calculate(this.swerveSubsystem.getHeading());
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-.7, 0, turningSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        this.swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.controller.getGoal().position - this.swerveSubsystem.getRotation2d().getRadians()) < Units.degreesToRadians(3);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}
