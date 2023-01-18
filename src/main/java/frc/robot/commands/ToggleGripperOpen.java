package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;


public class ToggleGripperOpen extends CommandBase {

    public ToggleGripperOpen() {}

    @Override
    public void initialize() {
        Pneumatics.getInstance().toggleGripperOpen();
    }
}
