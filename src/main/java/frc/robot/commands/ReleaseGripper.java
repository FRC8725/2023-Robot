package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class ReleaseGripper extends CommandBase {
    Pneumatics pneumatics;

    public ReleaseGripper(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        addRequirements();
    }

    @Override
    public void initialize() {
        pneumatics.setGripper(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
