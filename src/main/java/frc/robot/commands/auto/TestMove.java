// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestMove extends SequentialCommandGroup {
  /** Creates a new Testmove. */
  public TestMove(SwerveSubsystem m_swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "RedPath", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

     HashMap<String, Command> eventMap = new HashMap<>();

     SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
             m_swerveSubsystem::getPose,
             m_swerveSubsystem::resetOdometry,
             Constants.DriveConstants.kDriveKinematics,
             new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
             new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
             m_swerveSubsystem::setModuleStates,
             eventMap,
             m_swerveSubsystem
     );
    Pose2d pathInitialPose = pathGroup.get(0).getInitialPose();
    Command fullAuto = autoBuilder.fullAuto(pathGroup);
    addCommands(new BackToInitial(m_swerveSubsystem, pathInitialPose), fullAuto, new InstantCommand(m_swerveSubsystem::stopModules));
  }
}
