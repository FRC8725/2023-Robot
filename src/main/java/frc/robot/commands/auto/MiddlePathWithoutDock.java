package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.ReleaseGripper;
import frc.robot.commands.ResetArm;
import frc.robot.commands.RunArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class MiddlePathWithoutDock extends SequentialCommandGroup {
    public MiddlePathWithoutDock(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {
        addCommands(
            new RunArmToPosition(armSubsystem, gripperSubsystem, Constants.PoseConstants.HIGH_ARM_POSE, true, true),
            new ReleaseGripper(pneumatics),
            new WaitCommand(0.5),
            new ResetArm(armSubsystem, gripperSubsystem, pneumatics)
            // new WaitCommand(0.5),
            //     new ParallelDeadlineGroup(
            //             new WaitCommand(3),
            //             new RepeatCommand(new InstantCommand(() -> swerveSubsystem.setModuleStates(Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-2., .0, .0), swerveSubsystem.getRotation2d())))))
            //     ),
        );
    }
}
