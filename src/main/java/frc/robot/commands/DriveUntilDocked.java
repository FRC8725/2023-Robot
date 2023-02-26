// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;



public class DriveUntilDocked extends CommandBase {
    /**
     * Creates a new balance.
     */
    final SwerveSubsystem swerveSubsystem;
    private boolean on = false;
    Timer timer = new Timer();
    private final boolean reverse;
    final PIDController controller = new PIDController(BalanceConstants.P_BALANCE*.4, BalanceConstants.I_BALANCE, BalanceConstants.D_BALANCE);

    public DriveUntilDocked(boolean direction) {
        // Use addRequirements() here to declare subsystem dependencies.
        swerveSubsystem = SwerveSubsystem.getInstance();
        this.reverse = direction;
        //controller.setTolerance(6);
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerveSubsystem.stopModules();
        timer.reset();
        on = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (reverse) {
            if (Math.abs(swerveSubsystem.getPitch()) < -BalanceConstants.pitchThreshold){
                on = true;
            }
            if (!on) {
                swerveSubsystem.setModuleStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(-BalanceConstants.xSpeedMax, .0, .0)));
            } else {
                swerveSubsystem.setModuleStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(Math.min(BalanceConstants.xSpeedMax, controller.calculate(swerveSubsystem.getPitch(), 0)), 0, 0)));
            }
        } else {
            if (Math.abs(swerveSubsystem.getPitch()) > BalanceConstants.pitchThreshold){
                on = true;
            }
            if (!on) {
                swerveSubsystem.setModuleStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(BalanceConstants.xSpeedMax*1.3, .0, .0)));
            } else {
                swerveSubsystem.setModuleStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(-Math.min(BalanceConstants.xSpeedMax, controller.calculate(swerveSubsystem.getPitch(), 0)), 0, 0)));
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (timer.get() > 2.5) return true;
        if (reverse) {
            return Math.abs(swerveSubsystem.getPitch()) > -BalanceConstants.pitchThreshold && on;
        } else {
            return Math.abs(swerveSubsystem.getPitch()) < BalanceConstants.pitchThreshold && on;
        }

    }
}
