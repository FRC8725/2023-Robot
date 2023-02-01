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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

public class Barrel extends SequentialCommandGroup {

    public Barrel(SwerveSubsystem m_swerveSubsystem) {

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
                "Barrel", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_swerveSubsystem::getPose,
                m_swerveSubsystem::resetOdometry,
                DriveConstants.kDriveKinematics,
                new PIDConstants(AutoConstants.kPXController, 0, 0),
                new PIDConstants(AutoConstants.kPThetaController, 0, 0),
                m_swerveSubsystem::setModuleStates,
                eventMap,
                m_swerveSubsystem
        );

        PathPlannerTrajectory firstTraj = pathGroup.get(0);
        Pose2d initialPose = firstTraj.getInitialPose();
        Command fullAuto = autoBuilder.fullAuto(pathGroup);
        addCommands(
//                new BackToInitial(m_swerveSubsystem, initialPose),
                fullAuto,
                new InstantCommand(m_swerveSubsystem::stopModules));
    }

    /*
    PIDController xController, yController, thetaController;

    private SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if(isFirstPath){
                        m_swerveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        m_swerveSubsystem::getPose, // Pose supplier
                        DriveConstants.kDriveKinematics,
                        xController,yController, thetaController,
                        m_swerveSubsystem::setModuleStates, // Module states consumer
                        m_swerveSubsystem // Requires this drive subsystem
                )
        );
    }

    */
}
