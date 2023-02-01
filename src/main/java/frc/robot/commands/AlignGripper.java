package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Gripper;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionManager;


public class AlignGripper extends CommandBase {

    GripperSubsystem gripperSubsystem;
    VisionManager visionManager;
    boolean isVision;

    public AlignGripper(GripperSubsystem gripperSubsystem , VisionManager visionManager) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.gripperSubsystem = gripperSubsystem;
        this.visionManager = visionManager;
        isVision = true;
        addRequirements(gripperSubsystem, visionManager);
    }

    public AlignGripper(GripperSubsystem gripperSubsystem, double rollSetpoint) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.gripperSubsystem = gripperSubsystem;
        gripperSubsystem.setRollSetpoint(rollSetpoint);
        isVision = false;
        addRequirements(gripperSubsystem, visionManager);
    }

    @Override
    public void initialize() {
        if(!isVision) return;
        gripperSubsystem.addRollSetpoint(visionManager.getConeAngleRads());
    }
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
