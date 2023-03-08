// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
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
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final GripperSubsystem gripperSubsystem = GripperSubsystem.getInstance();
    private final Pneumatics pneumatics = Pneumatics.getInstance();
    private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
    private final GamepadJoystick swerveJoystick = new GamepadJoystick(0);
    private final GamepadJoystick elevatorJoystick = new GamepadJoystick(1);
    private final VisionManager visionManager = new VisionManager();
    NetworkTable led_nt = NetworkTableInstance.getDefault().getTable("LEDs");
    private final SendableChooser<Command> autoCommand = new SendableChooser<>();
    private final SendableChooser<Command> loadingChooser = new SendableChooser<>();


    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                this.swerveSubsystem,
                () -> +this.swerveJoystick.get_LStickY(),
                () -> -this.swerveJoystick.get_LStickX(),
                () -> -this.swerveJoystick.get_RStickX(),
                () -> !this.swerveJoystick.btn_topL.getAsBoolean(),
                this.swerveJoystick.btn_topR::getAsBoolean
        ));
        armSubsystem.setDefaultCommand(new ArmJoystickCmd(
                armSubsystem,
                () -> elevatorJoystick.get_LStickY(),
                () -> elevatorJoystick.get_RStickY(),
                () -> elevatorJoystick.POV_North.getAsBoolean(),
                () -> elevatorJoystick.POV_South.getAsBoolean()
        ));
        ledSubsystem.setDefaultCommand(new AutoLEDs(ledSubsystem));
        configureButtonBindings();
        putToDashboard();
    }

    private void configureButtonBindings() {
        swerveJoystick.btn_triggerR.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
        // swerveJoystick.btn_X.whileTrue(new CorrectPosition(0, visionManager));
        // swerveJoystick.btn_Y.whileTrue(new CorrectPosition(1, visionManager));
        swerveJoystick.btn_B.onTrue(new LockChassis(swerveSubsystem));
        swerveJoystick.POV_West.whileTrue(new CorrectPosition("left", visionManager)).debounce(.1, Debouncer.DebounceType.kBoth);
        swerveJoystick.POV_North.whileTrue(new CorrectPosition("middle", visionManager)).debounce(.1, Debouncer.DebounceType.kBoth);
        swerveJoystick.POV_East.whileTrue(new CorrectPosition("right", visionManager)).debounce(.1, Debouncer.DebounceType.kBoth);
        swerveJoystick.POV_South.whileTrue(new CorrectPosition("single", visionManager)).debounce(.1, Debouncer.DebounceType.kBoth);
        swerveJoystick.btn_triggerL.whileTrue(new DriveUntilDocked(false, swerveSubsystem));

//        elevatorJoystick.btn_triggerL.whileTrue(new RunGripper(gripperSubsystem, visionManager, pneumatics));
        elevatorJoystick.btn_topL.onTrue(new GrabPiecesFromDouble(armSubsystem, gripperSubsystem, pneumatics));
        elevatorJoystick.btn_triggerL.onTrue(new GrabPieces(armSubsystem, gripperSubsystem, pneumatics));
        elevatorJoystick.btn_topR.onTrue(new ResetArm(armSubsystem, gripperSubsystem, pneumatics));
        elevatorJoystick.btn_triggerR.onTrue(new ReleaseGripper(pneumatics));
//        elevatorJoystick.btn_Y.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kHighElevatorPose));
        elevatorJoystick.btn_Y.onTrue(new RunArmToPosition(armSubsystem, gripperSubsystem, PoseConstants.HIGH_ARM_POSE, true, true));
        elevatorJoystick.btn_B.onTrue(new RunArmToPosition(armSubsystem, gripperSubsystem, PoseConstants.MID_ARM_POSE, true, true));
        elevatorJoystick.btn_A.onTrue(new RunArmToPosition(armSubsystem, gripperSubsystem, PoseConstants.LOW_ARM_POSE, true, true));
        elevatorJoystick.btn_X.onTrue(new GrabPiecesFromSingle(armSubsystem, gripperSubsystem, pneumatics));
    }

    private void putToDashboard() {
        autoCommand.addOption("(0)Nothing", new InstantCommand(swerveSubsystem::stopModules));
        autoCommand.addOption("(1)Wide", new WidePath(swerveSubsystem, armSubsystem, gripperSubsystem, pneumatics));
        autoCommand.addOption("(2)Middle", new MiddlePath(swerveSubsystem, armSubsystem, gripperSubsystem, pneumatics));
        autoCommand.addOption("(3)Narrow", new NarrowPath(swerveSubsystem, armSubsystem, gripperSubsystem, pneumatics));
        SmartDashboard.putData(autoCommand);
        SmartDashboard.putData(new PowerDistribution(RobotMap.PDMPort, PowerDistribution.ModuleType.kRev));
        loadingChooser.addOption("Double-Cone",
                new ParallelCommandGroup(
                        new InstantCommand(() -> led_nt.getIntegerTopic("what2grab").publish().set(1)),
                        new InstantCommand(() -> led_nt.getIntegerTopic("where2go").publish().set(2))
                ));
        loadingChooser.addOption("Double-Cube",
                new ParallelCommandGroup(
                        new InstantCommand(() -> led_nt.getIntegerTopic("what2grab").publish().set(0)),
                        new InstantCommand(() -> led_nt.getIntegerTopic("where2go").publish().set(2))
                ));
        loadingChooser.addOption("Single-Cone",
                new ParallelCommandGroup(
                        new InstantCommand(() -> led_nt.getIntegerTopic("what2grab").publish().set(1)),
                        new InstantCommand(() -> led_nt.getIntegerTopic("where2go").publish().set(1))
                ));
        loadingChooser.addOption("Single-Cube",
                new ParallelCommandGroup(
                        new InstantCommand(() -> led_nt.getIntegerTopic("what2grab").publish().set(0)),
                        new InstantCommand(() -> led_nt.getIntegerTopic("where2go").publish().set(1))
                ));
        loadingChooser.addOption("Ground", new InstantCommand(() -> led_nt.getIntegerTopic("where2go").publish().set(0)));
        SmartDashboard.putData(loadingChooser);
    }

    public Command getAutonomousCommand() {
        // swerveSubsystem.setRobotPoseWithVision();
        swerveSubsystem.zeroHeading();
        return autoCommand.getSelected();
    }
}
