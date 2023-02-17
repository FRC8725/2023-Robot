// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.auto.RightOneGamePieceAndBalance;
import frc.robot.commands.auto.TestMove;
import frc.robot.commands.auto.TurnOpposite;
import frc.robot.subsystems.*;
import frc.robot.Constants.PoseConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Some default constants

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final ElevatorSubsystem m_elevatorSubsystem = ElevatorSubsystem.getInstance();
    private final GripperSubsystem m_gripperSubsystem = GripperSubsystem.getInstance();
    private final Pneumatics m_pneumatics = Pneumatics.getInstance();
    private final GamepadJoystick swerveJoystick = new GamepadJoystick(0);
    private final GamepadJoystick elevatorJoystick = new GamepadJoystick(1);
    private final VisionManager m_visionManager = new VisionManager();
    private final SendableChooser<Command> autoCommand = new SendableChooser<>();


    public RobotContainer() {
        this.swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                this.swerveSubsystem,
                () -> +this.swerveJoystick.get_LStickY(),
                () -> -this.swerveJoystick.get_LStickX(),
                () -> -this.swerveJoystick.get_RStickX(),
                () -> !this.swerveJoystick.btn_topL.getAsBoolean(),
                this.swerveJoystick.btn_topR::getAsBoolean
        ));
        m_elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(
                m_elevatorSubsystem,
                this.elevatorJoystick::get_LStickY,
                this.elevatorJoystick::get_RStickY,
                this.elevatorJoystick::get_RStickX,
                this.elevatorJoystick::get_LStickX,
                () -> !this.elevatorJoystick.btn_triggerR.getAsBoolean())
        );
        configureButtonBindings();
        putToDashboard();
    }

    private void configureButtonBindings() {
        swerveJoystick.btn_triggerR.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
        // m_swerveJoystick.btn_A.whileTrue(new CorrectPositionReflectiveTape(m_swerveSubsystem, m_visionManager));
        swerveJoystick.btn_X.whileTrue(new CorrectPosition(0, Units.inchesToMeters(18)));
        swerveJoystick.btn_Y.whileTrue(new CorrectPosition(1, Units.inchesToMeters(18)));
        swerveJoystick.btn_B.whileTrue(new CorrectPosition(2, Units.inchesToMeters(18)));
        swerveJoystick.btn_triggerL.whileTrue(new DriveUntilDocked());

        elevatorJoystick.btn_triggerL.whileTrue(new RunGripper(m_gripperSubsystem, m_visionManager, m_pneumatics));
        elevatorJoystick.btn_topR.onTrue(new InstantCommand(m_elevatorSubsystem::reset));
        elevatorJoystick.btn_Y.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kHighElevatorPose));
        elevatorJoystick.btn_X.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kMidElevatorPose));
        elevatorJoystick.btn_A.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kLowElevatorPose));
        elevatorJoystick.btn_B.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kLoadingZoneElevatorPose));
        elevatorJoystick.btn_Start.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kInitElevatorPose));
    }

    private void putToDashboard() {
        autoCommand.addOption("Nothing", new InstantCommand(swerveSubsystem::stopModules));
        autoCommand.addOption("turn and docking", new SequentialCommandGroup(new CorrectPosition(1, 18), new TurnOpposite(), new DriveUntilDocked()));
        autoCommand.addOption("RightOneGamePieceAndBalance", new RightOneGamePieceAndBalance(swerveSubsystem));
        autoCommand.addOption("[test]Red path", new TestMove(swerveSubsystem));
        SmartDashboard.putData(autoCommand);
    }

    public Command getAutonomousCommand() {
        swerveSubsystem.setRobotPoseWithVision();
        swerveSubsystem.zeroHeading();
        return autoCommand.getSelected();
    }
}
