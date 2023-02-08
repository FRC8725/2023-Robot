package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;


public class CloseGripper extends CommandBase {

    GripperSubsystem gripperSubsystem;
    Pneumatics pneumatics;

    public CloseGripper(GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        addRequirements(gripperSubsystem, pneumatics);
    }

    @Override
    public void initialize() {
        gripperSubsystem.runIntake(true, false);
        Timer.delay(1);
        pneumatics.setGripper(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.runIntake(false, false);
    }
}
