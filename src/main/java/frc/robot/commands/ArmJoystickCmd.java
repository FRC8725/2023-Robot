package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.Supplier;


public class ArmJoystickCmd extends CommandBase {

    ArmSubsystem armSubsystem;
    Supplier<Double> xSpdFunction, ySpdFunction;
    Supplier<Boolean> plusWrist, minusWrist;
    boolean firstPressed = false;

    public ArmJoystickCmd(ArmSubsystem armSubsystem,
                          Supplier<Double> xSpdFunction,
                          Supplier<Double> ySpdFunction,
                          Supplier<Boolean> plusWrist,
                          Supplier<Boolean> minusWrist
                          ) {
        this.armSubsystem = armSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.plusWrist = plusWrist;
        this.minusWrist = minusWrist;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        var xSpeed = this.xSpdFunction.get();
        xSpeed = Math.abs(xSpeed) > 0.5 ? xSpeed: 0;
        var ySpeed = this.ySpdFunction.get();
        ySpeed = Math.abs(ySpeed) > 0.5 ? ySpeed: 0;
        armSubsystem.setSpeed(xSpeed, ySpeed);

        if (!firstPressed) {
            if (plusWrist.get()) {
                armSubsystem.addWristStage(1);
                firstPressed = true;
            }
            else if (minusWrist.get()) {
                armSubsystem.addWristStage(-1);
                firstPressed = true;
            }
        } else if(!plusWrist.get() && !minusWrist.get()) {
            firstPressed = false;
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.stop();
    }
}
