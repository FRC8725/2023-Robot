package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;


public class ToggleArm extends CommandBase {

    Pneumatics pneumatics;

    public ToggleArm(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
    }

    @Override
    public void initialize() {
        pneumatics.toggleArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
