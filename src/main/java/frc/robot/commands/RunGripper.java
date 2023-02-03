package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class RunGripper extends CommandBase {

    GripperSubsystem gripperSubsystem;
    VisionManager visionManager;
    Pneumatics pneumatics;
    boolean hasObject;

    public RunGripper(GripperSubsystem gripperSubsystem, VisionManager visionManager, Pneumatics pneumatics) {
        this.gripperSubsystem = gripperSubsystem;
        this.visionManager = visionManager;
        this.pneumatics = pneumatics;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        hasObject = !gripperSubsystem.getIntakeSwitch();
        gripperSubsystem.runIntake(false, false);
    }

    @Override
    public void execute() {
        gripperSubsystem.runIntake(true, hasObject);
        pneumatics.setGripper(!visionManager.isCone() || hasObject);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !hasObject && !gripperSubsystem.getIntakeSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.runIntake(false, false);
    }
}
