package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveUntilDocked;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

public class RightOneGamePieceAndBalance extends SequentialCommandGroup {
    public RightOneGamePieceAndBalance(SwerveSubsystem m_swerveSubsystem) {

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
                "Right2Middle", new PathConstraints(AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("balance", new DriveUntilDocked());

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_swerveSubsystem::getPose,
                m_swerveSubsystem::resetOdometry,
                DriveConstants.DRIVE_KINEMATICS,
                new PIDConstants(AutoConstants.P_X_CONTROLLER, 0, 0),
                new PIDConstants(AutoConstants.P_THETA_CONTROLLER, 0, 0),
                m_swerveSubsystem::setModuleStates,
                eventMap,
                true,
                m_swerveSubsystem
        );
        Command fullAuto = autoBuilder.fullAuto(pathGroup);
        addCommands(fullAuto, new InstantCommand(m_swerveSubsystem::stopModules));
    }
}
