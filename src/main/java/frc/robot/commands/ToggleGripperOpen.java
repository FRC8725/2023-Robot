package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Pneumatics;


public class ToggleGripperOpen extends CommandBase {

    Pneumatics pneumatics;
    ElevatorSubsystem elevatorSubsystem;

    public ToggleGripperOpen(Pneumatics pneumatics, ElevatorSubsystem elevatorSubsystem) {
        this.pneumatics = pneumatics;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        pneumatics.toggleGripperOpen();
    }
    @Override
    public void execute() {
        if(pneumatics.getGripperStatus()) {
            elevatorSubsystem.runIntake(true, false);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.runIntake(false, false);
    }
}
