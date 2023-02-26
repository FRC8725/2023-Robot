package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;


public class AutoLEDs extends CommandBase {

    LEDSubsystem ledSubsystem;

    public AutoLEDs(LEDSubsystem ledSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.ledSubsystem = ledSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ledSubsystem.rainbow();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setColor(Color.kOrangeRed);
    }
}
