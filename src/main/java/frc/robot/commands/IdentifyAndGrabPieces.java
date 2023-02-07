package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class IdentifyAndGrabPieces extends SequentialCommandGroup {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    Pneumatics pneumatics;

    public IdentifyAndGrabPieces(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics, VisionManager visionManager) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        addCommands(
                new InstantCommand(() -> pneumatics.setGripper(true)),
                new RunArmToPosition(armSubsystem, gripperSubsystem, 1, .5, false),
                new InstantCommand(() -> pneumatics.setGripper(false)),
                new ResetArm(armSubsystem, gripperSubsystem, pneumatics)
                );
    }
}
