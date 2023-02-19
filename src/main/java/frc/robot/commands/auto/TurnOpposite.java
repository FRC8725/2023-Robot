package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnOpposite extends CommandBase {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

    public TurnOpposite() {
        this.addRequirements(this.swerveSubsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 1.5, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        this.swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerveSubsystem.getHeading() - 180) < 3;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}