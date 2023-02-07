// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.Barrel;
import frc.robot.commands.auto.RightOneGamePieceAndBalance;
import frc.robot.subsystems.*;
import frc.robot.Constants.PoseConstants;

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
    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final GripperSubsystem m_gripperSubsystem = GripperSubsystem.getInstance();
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
        m_armSubsystem.setDefaultCommand(new ArmJoystickCmd(
                m_armSubsystem,
                () -> m_elevatorJoystick.get_LStickY(),
                () -> m_elevatorJoystick.get_RStickY()
        ));
        m_gripperSubsystem.setDefaultCommand(new SetGripperHorizontal(m_gripperSubsystem, true));
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

//        m_elevatorJoystick.btn_triggerL.whileTrue(new RunGripper(m_gripperSubsystem, m_visionManager, m_pneumatics));
        m_elevatorJoystick.btn_topL.onTrue(new GrabPieces(m_armSubsystem, m_gripperSubsystem, m_pneumatics));
        m_elevatorJoystick.btn_triggerL.onTrue(new IdentifyAndGrabPieces(m_armSubsystem, m_gripperSubsystem, m_pneumatics, m_visionManager));
        m_elevatorJoystick.btn_topR.onTrue(new ResetArm(m_armSubsystem, m_gripperSubsystem, m_pneumatics));
        m_elevatorJoystick.btn_triggerR.onTrue(new ReleaseGripper(m_pneumatics));
//        m_elevatorJoystick.btn_Y.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kHighElevatorPose));
        m_elevatorJoystick.btn_Y.onTrue(new RunArmToPosition(m_armSubsystem, m_gripperSubsystem, PoseConstants.kHighArmPose, true));
        m_elevatorJoystick.btn_X.onTrue(new RunArmToPosition(m_armSubsystem, m_gripperSubsystem, PoseConstants.kMidArmPose, true));
        m_elevatorJoystick.btn_A.onTrue(new RunArmToPosition(m_armSubsystem, m_gripperSubsystem, PoseConstants.kLowArmPose, true));
        m_elevatorJoystick.btn_B.toggleOnTrue(new GrabPiecesFromDouble(m_armSubsystem, m_gripperSubsystem, m_pneumatics));
        m_elevatorJoystick.btn_Back.whileTrue(new AlignGripper(m_gripperSubsystem, m_visionManager));
        m_elevatorJoystick.btn_Start.whileTrue(new SetGripperHorizontal(m_gripperSubsystem,false));
    }

    private void putToDashboard() {
        autoCommand.addOption("Nothing", new InstantCommand(m_swerveSubsystem::stopModules));
        autoCommand.addOption("Barrel", new Barrel(m_swerveSubsystem));
        autoCommand.addOption("RightOneGamePieceAndBalance", new RightOneGamePieceAndBalance(m_swerveSubsystem, m_visionManager));
        SmartDashboard.putData(autoCommand);
    }

    public Command getAutonomousCommand() {
        SmartDashboard.putData(m_swerveSubsystem.getfield2d());
        m_swerveSubsystem.resetEncoders();
        return autoCommand.getSelected();
    }
}
