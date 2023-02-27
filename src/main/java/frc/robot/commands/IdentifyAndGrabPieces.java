package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.PoseConstants;
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
        addRequirements(armSubsystem, gripperSubsystem, pneumatics, visionManager);
//        if (armSubsystem.getIsTransporting()) return;
        addCommands(
                new RunArmToPosition(armSubsystem, gripperSubsystem, Constants.PoseConstants.VISION_ARM_POSE, false, false),
                new InstantCommand(() -> pneumatics.setGripper(true)),
                new AlignGripper(gripperSubsystem, visionManager),
                new ParallelCommandGroup(
                        new WaitUntilCommand(gripperSubsystem::isPiecesInRange),
                        new InstantCommand(() -> armSubsystem.setSetpoint(PoseConstants.VERTICAL_GRAB_ARM_POSE.getFirst(), PoseConstants.VERTICAL_GRAB_ARM_POSE.getSecond()))
                ),
                new InstantCommand(() -> pneumatics.setGripper(false)),
                new ResetArm(armSubsystem, gripperSubsystem, pneumatics)
        );
    }
}
