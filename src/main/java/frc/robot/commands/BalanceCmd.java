// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Balance;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

;

public class BalanceCmd extends CommandBase {
    /**
     * Creates a new balance.
     */
    SwerveSubsystem m_swerveSubsystem;
    PIDController controller = new PIDController(Balance.kPBalance, Balance.kIBalance, Balance.kDBalance);

    public BalanceCmd(SwerveSubsystem swerveSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_swerveSubsystem = swerveSubsystem;
        controller.setTolerance(6);
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_swerveSubsystem.stopModules();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_swerveSubsystem.getPitch() > 8)m_swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(-0.7, 0, 0)));
        else if (m_swerveSubsystem.getPitch() < -8)m_swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.7, 0, 0)));
        else m_swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(-controller.calculate(m_swerveSubsystem.getPitch(), 0), 0, 0)));
        // if (Math.abs(m_swerveSubsystem.getRoll()) > 10.) {
        //     m_swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates( new ChassisSpeeds(0.05, 0, 0)));
        //     passed = true;
        // } else if (Math.abs(m_swerveSubsystem.getRoll()) < 10. && passed) {
        //     m_swerveSubsystem.stopModules();
        // } else {
        //     m_swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.2, 0, 0)));
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
