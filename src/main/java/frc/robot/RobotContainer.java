// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.BalanceCmd;
import frc.robot.commands.CorrectPosition;
import frc.robot.commands.CorrectPositionRefletiveTape;
import frc.robot.commands.SwerveJoystickCmd;
<<<<<<< HEAD
<<<<<<< HEAD
import frc.robot.commands.auto.Barrel;
=======
>>>>>>> 637b9602b97092e1f4b52eb9c053ac32cba35d75
=======
>>>>>>> 637b9602b97092e1f4b52eb9c053ac32cba35d75
import frc.robot.commands.auto.RightOneGamePieceAndBalance;
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
    private final GamepadJoystick m_swerveJoystick = new GamepadJoystick(0);

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
        autoCommand.addOption("Nothing", new InstantCommand(m_swerveSubsystem::stopModules));
        autoCommand.addOption("Barrel", new Barrel(m_swerveSubsystem));
        autoCommand.addOption("RightOneGamePieceAndBalance", new RightOneGamePieceAndBalance(m_swerveSubsystem));
        configureButtonBindings();
        putToDashboard();
    }

    private void configureButtonBindings() {
        m_swerveJoystick.btn_triggerR.onTrue(new InstantCommand(m_swerveSubsystem::zeroHeading));
        m_swerveJoystick.btn_A.whileTrue(new CorrectPositionRefletiveTape(m_swerveSubsystem));
        m_swerveJoystick.btn_X.whileTrue(new CorrectPosition(m_swerveSubsystem, 0));
        m_swerveJoystick.btn_Y.whileTrue(new CorrectPosition(m_swerveSubsystem, 1));
        m_swerveJoystick.btn_B.whileTrue(new CorrectPosition(m_swerveSubsystem, 2));
        m_swerveJoystick.btn_triggerL.whileTrue(new BalanceCmd(m_swerveSubsystem));
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
