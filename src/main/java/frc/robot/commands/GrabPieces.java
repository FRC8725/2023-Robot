package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class GrabPieces extends SequentialCommandGroup {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    Pneumatics pneumatics;

    public GrabPieces(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics, VisionManager visionManager) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        addRequirements(armSubsystem, gripperSubsystem);
        addCommands(
                new RunArmToPosition(armSubsystem, gripperSubsystem, 1, .5, false),
                new InstantCommand(() -> pneumatics.setGripper(true)),
                new AlignGripper(gripperSubsystem, visionManager),
                new RunArmToPosition(armSubsystem, gripperSubsystem, 1, .3, false),
                new InstantCommand(() -> pneumatics.setGripper(false)),
                new ResetArm(armSubsystem, gripperSubsystem, pneumatics)
                );
    }
}
