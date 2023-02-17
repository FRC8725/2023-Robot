// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;



public class DriveUntilDocked extends CommandBase {
    /**
     * Creates a new balance.
     */
    SwerveSubsystem swerveSubsystem;
    PIDController controller = new PIDController(BalanceConstants.kPBalance, BalanceConstants.kIBalance, BalanceConstants.kDBalance);

    public DriveUntilDocked() {
        // Use addRequirements() here to declare subsystem dependencies.
        swerveSubsystem = SwerveSubsystem.getInstance();
        controller.setTolerance(6);
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerveSubsystem.stopModules();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(swerveSubsystem.getRoll()) > BalanceConstants.halfOnStageTheta)
            swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(Math.copySign(0.6, -swerveSubsystem.getRoll()), 0, 0)));
        else swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(Math.min(BalanceConstants.xSpeedMax, -controller.calculate(swerveSubsystem.getRoll(), 0)), 0, 0)));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(swerveSubsystem.getRoll()) > BalanceConstants.xSpeedThreshold && swerveSubsystem.getZAcc() > 1.2;
    }
}
