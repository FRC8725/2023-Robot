package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;


public class ToggleGripperOpen extends CommandBase {

    Pneumatics pneumatics;

    public ToggleGripperOpen(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
    }

    @Override
    public void initialize() {
        pneumatics.toggleGripperOpen();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
