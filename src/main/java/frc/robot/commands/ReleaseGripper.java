package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class ReleaseGripper extends CommandBase {
    Pneumatics pneumatics;

    public ReleaseGripper(GripperSubsystem gripperSubsystem,Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        addRequirements(pneumatics);
    }

    @Override
    public void initialize() {
        pneumatics.setGripper(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
