package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;


public class RunElevatorToPosition extends CommandBase {

    ElevatorSubsystem elevatorSubsystem;
    double position;

    public RunElevatorToPosition(ElevatorSubsystem elevatorSubsystem, double position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorSetpoint(position);
    }
}
