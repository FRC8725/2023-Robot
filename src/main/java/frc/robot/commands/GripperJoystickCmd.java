package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

import java.util.function.Supplier;


public class GripperJoystickCmd extends CommandBase {

    GripperSubsystem gripperSubsystem;
    Supplier<Boolean> plusRoll, minusRoll;


    public GripperJoystickCmd(GripperSubsystem gripperSubsystem,
                              Supplier<Boolean> plusRoll,
                              Supplier<Boolean> minusRoll
                              ) {
        this.gripperSubsystem = gripperSubsystem;
        this.plusRoll = plusRoll;
        this.minusRoll = minusRoll;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        gripperSubsystem.addRollSetpoint((plusRoll.get()? 0.5: 0) + (minusRoll.get()? -0.5: 0));
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
