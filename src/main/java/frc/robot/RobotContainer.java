// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
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
    private final SwerveSubsystem m_swerveSubsystem = SwerveSubsystem.getInstance();
    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final GripperSubsystem m_gripperSubsystem = GripperSubsystem.getInstance();
    private final Pneumatics m_pneumatics = Pneumatics.getInstance();
    private final GamepadJoystick swerveJoystick = new GamepadJoystick(0);
    private final GamepadJoystick m_elevatorJoystick = new GamepadJoystick(1);
    private final VisionManager m_visionManager = new VisionManager();
    private final SendableChooser<Command> autoCommand = new SendableChooser<>();


    public RobotContainer() {
        this.m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                this.m_swerveSubsystem,
                () -> +this.swerveJoystick.get_LStickY(),
                () -> -this.swerveJoystick.get_LStickX(),
                () -> -this.swerveJoystick.get_RStickX(),
                () -> !this.swerveJoystick.btn_topL.getAsBoolean(),
                this.swerveJoystick.btn_topR::getAsBoolean
        ));
        m_armSubsystem.setDefaultCommand(new ArmJoystickCmd(
                m_armSubsystem,
                () -> m_elevatorJoystick.get_LStickY(),
                () -> m_elevatorJoystick.get_RStickY()
        ));
        configureButtonBindings();
        putToDashboard();
    }

    private void configureButtonBindings() {
        swerveJoystick.btn_triggerR.onTrue(new InstantCommand(m_swerveSubsystem::zeroHeading));
        // m_swerveJoystick.btn_A.whileTrue(new CorrectPositionReflectiveTape(m_swerveSubsystem, m_visionManager));
        swerveJoystick.btn_X.whileTrue(new CorrectPosition(0, Units.inchesToMeters(18)));
        swerveJoystick.btn_Y.whileTrue(new CorrectPosition(1, Units.inchesToMeters(18)));
        swerveJoystick.btn_B.whileTrue(new CorrectPosition(2, Units.inchesToMeters(18)));
        swerveJoystick.btn_triggerL.whileTrue(new DriveUntilDocked());

//        m_elevatorJoystick.btn_triggerL.whileTrue(new RunGripper(m_gripperSubsystem, m_visionManager, m_pneumatics));
        m_elevatorJoystick.btn_topL.onTrue(new GrabPieces(m_armSubsystem, m_gripperSubsystem, m_pneumatics, m_visionManager));
        m_elevatorJoystick.btn_triggerL.onTrue(new IdentifyAndGrabPieces(m_armSubsystem, m_gripperSubsystem, m_pneumatics, m_visionManager));
        m_elevatorJoystick.btn_topR.onTrue(new ResetArm(m_armSubsystem, m_gripperSubsystem, m_pneumatics));
        m_elevatorJoystick.btn_triggerR.onTrue(new ReleaseGripper(m_gripperSubsystem, m_pneumatics));
//        m_elevatorJoystick.btn_Y.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kHighElevatorPose));
        m_elevatorJoystick.btn_Y.onTrue(new RunArmToPosition(m_armSubsystem, m_gripperSubsystem, PoseConstants.HIGH_ARM_POSE, true, true));
        m_elevatorJoystick.btn_X.onTrue(new RunArmToPosition(m_armSubsystem, m_gripperSubsystem, PoseConstants.MID_ARM_POSE, true, true));
        m_elevatorJoystick.btn_A.onTrue(new RunArmToPosition(m_armSubsystem, m_gripperSubsystem, PoseConstants.LOW_ARM_POSE, true, true));
        m_elevatorJoystick.btn_B.toggleOnTrue(new GrabPiecesFromDouble(m_armSubsystem, m_gripperSubsystem, m_pneumatics, m_visionManager));
        m_elevatorJoystick.btn_Back.whileTrue(new AlignGripper(m_gripperSubsystem, m_visionManager));
    }

    private void putToDashboard() {
        autoCommand.addOption("Nothing", new InstantCommand(m_swerveSubsystem::stopModules));
        autoCommand.addOption("turn and docking", new SequentialCommandGroup(new CorrectPosition(1, 18), new TurnOpposite(), new DriveUntilDocked()));
        autoCommand.addOption("RightOneGamePieceAndBalance", new RightOneGamePieceAndBalance(m_swerveSubsystem));
        autoCommand.addOption("[test]Red path", new TestMove(m_swerveSubsystem));
        SmartDashboard.putData(autoCommand);
    }

    public Command getAutonomousCommand() {
        m_swerveSubsystem.setRobotPoseWithVision();
        m_swerveSubsystem.zeroHeading();
        return autoCommand.getSelected();
    }
}
