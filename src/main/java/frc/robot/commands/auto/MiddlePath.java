package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveUntilDocked;
import frc.robot.commands.ReleaseGripper;
import frc.robot.commands.ResetArm;
import frc.robot.commands.RunArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

public class MiddlePath extends SequentialCommandGroup {
    public MiddlePath(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {
        addCommands(
            new RunArmToPosition(armSubsystem, gripperSubsystem, Constants.PoseConstants.HIGH_ARM_POSE, true, true),
            new ReleaseGripper(pneumatics),
            new WaitCommand(0.5),
            new ResetArm(armSubsystem, gripperSubsystem, pneumatics),
            new WaitCommand(3),
            new DriveUntilDocked(true, swerveSubsystem)
        );
    }
}
