package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.GrabPieces;
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
                "WidePath", new PathConstraints(AutoConstants.MAX_SPEED_METERS_PER_SECOND*0.8, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*0.8));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putItem", new RunArmToPosition(armSubsystem, gripperSubsystem, Constants.PoseConstants.HIGH_ARM_POSE, true, true));
        eventMap.put("pickItem", new GrabPieces(armSubsystem, gripperSubsystem, pneumatics));

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
