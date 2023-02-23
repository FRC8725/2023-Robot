// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ResetArm;
import frc.robot.commands.RunArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;

public class pickItem extends SequentialCommandGroup {
  public pickItem(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

      addCommands(
          new InstantCommand(() -> pneumatics.setGripper(true)),
          new RunArmToPosition(armSubsystem, gripperSubsystem, Constants.PoseConstants.LOW_ARM_POSE, true, false),
          new ResetArm(armSubsystem, gripperSubsystem, pneumatics)
      );
  }
}
