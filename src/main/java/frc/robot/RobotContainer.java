// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.PoseConstants;

import java.util.Map;


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
    private final XboxJoystick swerveJoystick = new XboxJoystick(0);
    private final XboxJoystick armJoystick = new XboxJoystick(1);
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
                () -> armJoystick.get_RStickY(),
                () -> armJoystick.get_LStickY(),
                () -> armJoystick.btn_RStick.getAsBoolean(),
                () -> armJoystick.btn_LStick.getAsBoolean()
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
        swerveJoystick.btn_X.onTrue(new InstantCommand(() -> where2goPub.set(0)));
        swerveJoystick.btn_Y.onTrue(new SequentialCommandGroup(new InstantCommand(gripperSubsystem::killDistance))).debounce(1, Debouncer.DebounceType.kRising);
        swerveJoystick.POV_West.whileTrue(new CorrectPosition("left", visionManager)).debounce(.1, Debouncer.DebounceType.kBoth);
        swerveJoystick.POV_North.whileTrue(new CorrectPosition("middle", visionManager)).debounce(.1, Debouncer.DebounceType.kBoth);
        swerveJoystick.POV_East.whileTrue(new CorrectPosition("right", visionManager)).debounce(.1, Debouncer.DebounceType.kBoth);
        swerveJoystick.POV_South.whileTrue(new CorrectPosition("single", visionManager)).debounce(.1, Debouncer.DebounceType.kBoth);
        swerveJoystick.btn_triggerL.whileTrue(new DriveUntilDocked(false, swerveSubsystem));

//        elevatorJoystick.btn_triggerL.whileTrue(new RunGripper(gripperSubsystem, visionManager, pneumatics));
        armJoystick.btn_topL.onTrue(new GrabPiecesFromDouble(armSubsystem, gripperSubsystem, pneumatics));
        armJoystick.btn_triggerL.onTrue(new GrabPieces(armSubsystem, gripperSubsystem, pneumatics));
        armJoystick.btn_topR.onTrue(new ResetArm(armSubsystem, gripperSubsystem, pneumatics));
        armJoystick.btn_triggerR.onTrue(new ReleaseGripper(pneumatics));
//        elevatorJoystick.btn_Y.onTrue(new RunElevatorToPosition(m_elevatorSubsystem, PoseConstants.kHighElevatorPose));
        armJoystick.btn_Y.onTrue(new RunArmToPosition(armSubsystem, gripperSubsystem, PoseConstants.HIGH_ARM_POSE, true, true));
        armJoystick.btn_B.onTrue(new RunArmToPosition(armSubsystem, gripperSubsystem, PoseConstants.MID_ARM_POSE, true, true));
        armJoystick.btn_A.onTrue(new RunArmToPosition(armSubsystem, gripperSubsystem, PoseConstants.LOW_ARM_POSE, true, true));
        armJoystick.btn_X.onTrue(new GrabPiecesFromSingle(armSubsystem, gripperSubsystem, pneumatics));
        armJoystick.POV_North.onTrue(new InstantCommand(() -> where2goPub.set(2)));
        armJoystick.POV_South.onTrue(new InstantCommand(() -> where2goPub.set(1)));
        armJoystick.POV_East.onTrue(new InstantCommand(() -> what2grabPub.set(1)));
        armJoystick.POV_West.onTrue(new InstantCommand(() -> what2grabPub.set(0)));
    }

    private void putToDashboard() {
        autoCommand.addOption("(0)Nothing", new InstantCommand(swerveSubsystem::stopModules));
        autoCommand.addOption("(1)Wide", new WidePath(swerveSubsystem, armSubsystem, gripperSubsystem, pneumatics));
        autoCommand.addOption("(2)Middle", new MiddlePath(swerveSubsystem, armSubsystem, gripperSubsystem, pneumatics));
        autoCommand.addOption("(2)Middlewithiut dock", new MiddlePathWithoutDock(swerveSubsystem, armSubsystem, gripperSubsystem, pneumatics));
        autoCommand.addOption("(3)Narrow", new NarrowPath(swerveSubsystem, armSubsystem, gripperSubsystem, pneumatics));
//        SmartDashboard.putData(autoCommand);
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Mode");
        try {
//            VideoSource ll = CameraServer.getVideo("limelight").getSource();
            tab.addCamera("limelight", "limelight")
                    .withSize(3, 3)
                    .withWidget(BuiltInWidgets.kCameraStream);
//            VideoSource rpi = CameraServer.getVideo("Processed").getSource();
//            tab.addCamera("rpi", "Processed")
//                    .withSize(3, 3)
//                    .withWidget(BuiltInWidgets.kCameraStream);
        }
        catch (Exception e) {
            System.out.println("Error: couldn't get the cameras");
        }
        tab.add("Auto Chooser", autoCommand)
                .withSize(2, 1);
//        ShuffleboardLayout loadingChooser = tab
//                .getLayout("Loading", BuiltInLayouts.kGrid)
//                .withSize(2, 2)
//                .withProperties(Map.of("Label position", "HIDDEN"));
//        loadingChooser.add(
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> what2grabPub.set(1)),
//                        new InstantCommand(() -> where2goPub.set(2))
//                ).withName("Double-Cone"))
//                .withWidget(BuiltInWidgets.kToggleButton);
//        loadingChooser.add(
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> what2grabPub.set(0)),
//                        new InstantCommand(() -> where2goPub.set(2))
//                ).withName("Double-Cube"))
//                .withWidget(BuiltInWidgets.kToggleButton);
//        loadingChooser.add(
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> what2grabPub.set(1)),
//                        new InstantCommand(() -> where2goPub.set(1))
//                ).withName("Single-Cone"))
//                .withWidget(BuiltInWidgets.kToggleButton);
//        loadingChooser.add(
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> what2grabPub.set(0)),
//                        new InstantCommand(() -> where2goPub.set(1))
//                ).withName("Single-Cube"))
//                .withWidget(BuiltInWidgets.kToggleButton);
//        loadingChooser.add(
//                new InstantCommand(() -> where2goPub.set(0))
//                    .withName("Ground"))
//                    .withWidget(BuiltInWidgets.kToggleButton);
        Shuffleboard.update();
    }

    public Command getAutonomousCommand() {
        // swerveSubsystem.setRobotPoseWithVision();
        swerveSubsystem.zeroHeading();
        return autoCommand.getSelected();
    }
}
