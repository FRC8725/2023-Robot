package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class RunGripper extends CommandBase {

    ElevatorSubsystem elevatorSubsystem;
    VisionManager visionManager;
    Pneumatics pneumatics;
    boolean hasObject;

    public RunGripper(ElevatorSubsystem elevatorSubsystem, VisionManager visionManager, Pneumatics pneumatics) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.visionManager = visionManager;
        this.pneumatics = pneumatics;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        hasObject = !elevatorSubsystem.getIntakeSwitch();
        elevatorSubsystem.runIntake(false, false);
        pneumatics.setGripper(!visionManager.isCone() || hasObject);
    }

    @Override
    public void execute() {
        elevatorSubsystem.runIntake(true, hasObject);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !hasObject && !elevatorSubsystem.getIntakeSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.runIntake(false, false);
    }
}
