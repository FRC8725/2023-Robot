package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveUntilDocked;
import frc.robot.commands.GrabPieces;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

public class NarrowPathDock extends SequentialCommandGroup {
    public NarrowPathDock(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
                "NarrowPath", new PathConstraints(AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putItem", new putItem(swerveSubsystem, armSubsystem, gripperSubsystem, pneumatics));
        eventMap.put("pickItem", new GrabPieces(armSubsystem, gripperSubsystem, pneumatics));
        eventMap.put("dock", new DriveUntilDocked(true));

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
