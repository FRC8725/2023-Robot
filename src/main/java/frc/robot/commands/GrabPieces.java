package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class GrabPieces extends SequentialCommandGroup {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    Pneumatics pneumatics;
    VisionManager visionManager;

    public GrabPieces(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics, VisionManager visionManager) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        this.visionManager = visionManager;
        addCommands(
                new InstantCommand(() -> pneumatics.setGripper(true, false)),
                new RunArmToPosition(armSubsystem, gripperSubsystem, Constants.PoseConstants.kLowArmPose, true),
                new InstantCommand(() -> pneumatics.setGripper(false, visionManager.isCone())),
                new ResetArm(armSubsystem, gripperSubsystem, pneumatics)
                );
    }
}
