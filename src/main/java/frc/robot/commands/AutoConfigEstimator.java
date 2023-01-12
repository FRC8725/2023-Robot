// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

import java.util.ArrayList;

public class AutoConfigEstimator extends CommandBase {
    /**
     * Creates a new AutoUpdateEstimator.
     */
    ArrayList<AprilTag> atList;
    SwerveSubsystem m_swerveSubsystem;

    public AutoConfigEstimator(SwerveSubsystem swervesubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_swerveSubsystem = swervesubsystem;
        addRequirements(m_swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (VisionManager.hasTarget()) {
            m_swerveSubsystem.updateWithVision(VisionManager.getEstimatedGlobalPose(m_swerveSubsystem.getPose()).getFirst(),
                    VisionManager.getEstimatedGlobalPose(m_swerveSubsystem.getPose()).getSecond());

        }
    }
}
