// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.PoseConstants;

import java.nio.channels.Pipe;


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
    IntegerPublisher what2grabPub =  led_nt.getIntegerTopic("what2grab").publish();
    IntegerPublisher where2goPub =  led_nt.getIntegerTopic("where2go").publish();
    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

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
//        SmartDashboard.putData(autoCommand);
        SmartDashboard.putData(new PowerDistribution(RobotMap.PDMPort, PowerDistribution.ModuleType.kRev));
        Shuffleboard.getTab("Driver Mode").add(autoCommand).withSize(3, 1).withPosition(0, 3);
        VideoSource ll = CameraServer.getVideo("limelight").getSource();
        Shuffleboard.getTab("Driver Mode").add(ll).withSize(4, 3).withPosition(0, 0);
        VideoSource rpi = CameraServer.getVideo("Processed").getSource();
        Shuffleboard.getTab("Driver Mode").add(rpi).withSize(4, 3).withPosition(4, 0);
        ShuffleboardLayout loadingChooser = Shuffleboard.getTab("Driver Mode")
                .getLayout("Loading", BuiltInLayouts.kList)
                .withSize(5, 1).withPosition(3, 3);
        loadingChooser.add("Double-Cone",
                new ParallelCommandGroup(
                        new InstantCommand(() -> what2grabPub.set(1)),
                        new InstantCommand(() -> where2goPub.set(2))
                ));
        loadingChooser.add("Double-Cube",
                new ParallelCommandGroup(
                        new InstantCommand(() -> what2grabPub.set(0)),
                        new InstantCommand(() -> where2goPub.set(2))
                ));
        loadingChooser.add("Single-Cone",
                new ParallelCommandGroup(
                        new InstantCommand(() -> what2grabPub.set(1)),
                        new InstantCommand(() -> where2goPub.set(1))
                ));
        loadingChooser.add("Single-Cube",
                new ParallelCommandGroup(
                        new InstantCommand(() -> what2grabPub.set(0)),
                        new InstantCommand(() -> where2goPub.set(1))
                ));
        loadingChooser.add("Ground", new InstantCommand(() -> led_nt.getIntegerTopic("where2go").publish().set(0)));
        Shuffleboard.selectTab("Driver Mode");
    }

    public Command getAutonomousCommand() {
        // swerveSubsystem.setRobotPoseWithVision();
        swerveSubsystem.zeroHeading();
        return autoCommand.getSelected();
    }
}
