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
import frc.robot.commands.ResetArm;
import frc.robot.commands.RunArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

public class WidePath extends SequentialCommandGroup {
    public WidePath(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
                "WidePath", new PathConstraints(AutoConstants.MAX_SPEED_METERS_PER_SECOND*.5, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*.5));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putHigh", new RunArmToPosition(armSubsystem, gripperSubsystem, Constants.PoseConstants.HIGH_ARM_POSE, true, true));
        eventMap.put("resetArm", new ResetArm(armSubsystem, gripperSubsystem, pneumatics));
        eventMap.put("release", new SequentialCommandGroup(new InstantCommand(() -> pneumatics.setGripper(true)), new WaitCommand(0.5)));
        eventMap.put("slowMove",new ParallelDeadlineGroup(
                new WaitCommand(5),
                new RepeatCommand(new InstantCommand(() -> swerveSubsystem.setModuleStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(-1, .0, .0), swerveSubsystem.getRotation2d())))))
        ));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerveSubsystem::getPose,
                swerveSubsystem::resetOdometry,
                DriveConstants.DRIVE_KINEMATICS,
                new PIDConstants(AutoConstants.P_X_CONTROLLER, 0, 0),
                new PIDConstants(AutoConstants.P_THETA_CONTROLLER, 0, 0),
                swerveSubsystem::setModuleStates,
                eventMap,
                true,
                swerveSubsystem
        );
        Command fullAuto = autoBuilder.fullAuto(pathGroup);
        addCommands(fullAuto, new InstantCommand(swerveSubsystem::stopModules));
    }
}
