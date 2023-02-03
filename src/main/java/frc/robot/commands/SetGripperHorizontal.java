package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;


public class SetGripperHorizontal extends CommandBase {

    GripperSubsystem gripperSubsystem;

    boolean isHorizontal;

    public SetGripperHorizontal(GripperSubsystem gripperSubsystem, boolean isHorizontal) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.gripperSubsystem = gripperSubsystem;
        this.isHorizontal = isHorizontal;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        gripperSubsystem.setHorizontal(isHorizontal);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
