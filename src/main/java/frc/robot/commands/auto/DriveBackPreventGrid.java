package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveBackPreventGrid extends CommandBase {
    SwerveSubsystem swerveSubsystem;
    Timer timer;
    public DriveBackPreventGrid() {
        swerveSubsystem = SwerveSubsystem.getInstance();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-1.5, 0, 0, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        this.swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.4;
    }
    public void end() {
        swerveSubsystem.stopModules();
    }
}
