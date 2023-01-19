// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.commands.auto.Barrel;
import frc.robot.commands.auto.RightOneGamePieceAndBalance;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

import org.photonvision.PhotonCamera;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Some default constants
    private final PhotonCamera photonCamera = new PhotonCamera("");

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem m_swerveSubsystem = SwerveSubsystem.getInstance();
    private final ElevatorSubsystem m_elevatorSubsystem = ElevatorSubsystem.getInstance();
    private final Pneumatics m_pneumatics = Pneumatics.getInstance();
    private final GamepadJoystick m_swerveJoystick = new GamepadJoystick(0);
    private final GamepadJoystick m_elevatorJoystick = new GamepadJoystick(1);
    private final VisionManager m_visionManager = new VisionManager();

    private final SendableChooser<Command> autoCommand = new SendableChooser<>();


    public RobotContainer() {
        m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                m_swerveSubsystem,
                () -> m_swerveJoystick.get_LStickY(),
                () -> -m_swerveJoystick.get_LStickX(),
                () -> -m_swerveJoystick.get_RStickX(),
                () -> !m_swerveJoystick.btn_topL.getAsBoolean(),
                () -> m_swerveJoystick.btn_topR.getAsBoolean()
        ));
        m_elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(
                m_elevatorSubsystem,
                () -> m_elevatorJoystick.get_LStickY(),
                () -> m_elevatorJoystick.get_RStickY(),
                () -> m_elevatorJoystick.get_RStickX()
        ));
        autoCommand.addOption("Nothing", new InstantCommand(m_swerveSubsystem::stopModules));
        autoCommand.addOption("Barrel", new Barrel(m_swerveSubsystem));
        autoCommand.addOption("RightOneGamePieceAndBalance", new RightOneGamePieceAndBalance(m_swerveSubsystem, m_visionManager));
        configureButtonBindings();
        putToDashboard();
    }

    private void configureButtonBindings() {
        m_swerveJoystick.btn_triggerR.onTrue(new InstantCommand(m_swerveSubsystem::zeroHeading));
        m_swerveJoystick.btn_A.whileTrue(new CorrectPositionReflectiveTape(m_swerveSubsystem, m_visionManager));
        m_swerveJoystick.btn_X.whileTrue(new CorrectPosition(m_swerveSubsystem, 0, m_visionManager));
        m_swerveJoystick.btn_Y.whileTrue(new CorrectPosition(m_swerveSubsystem, 1, m_visionManager));
        m_swerveJoystick.btn_B.whileTrue(new CorrectPosition(m_swerveSubsystem, 2, m_visionManager));
        m_swerveJoystick.btn_triggerL.whileTrue(new BalanceCmd(m_swerveSubsystem));

        m_elevatorJoystick.btn_triggerL.toggleOnTrue(new RunGripper(m_elevatorSubsystem));
        m_elevatorJoystick.btn_topL.onTrue(new ToggleGripperOpen(m_pneumatics));
        m_elevatorJoystick.btn_topR.onTrue(new ToggleArm(m_pneumatics));
        m_elevatorJoystick.btn_A.onTrue(new InstantCommand(m_elevatorSubsystem::reset));
    }

    private void putToDashboard() {
        autoCommand.addOption("Nothing", new InstantCommand(m_swerveSubsystem::stopModules));
        SmartDashboard.putData(autoCommand);
    }

    public Command getAutonomousCommand() {
        SmartDashboard.putData(m_swerveSubsystem.getfield2d());
        m_swerveSubsystem.resetEncoders();
        return autoCommand.getSelected();
    }
}
