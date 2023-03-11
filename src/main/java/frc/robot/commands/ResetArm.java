package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class ResetArm extends CommandBase {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    Pneumatics pneumatics;

    double startTime;

    public ResetArm(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        addRequirements(armSubsystem, gripperSubsystem, pneumatics);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        armSubsystem.reset();
//        gripperSubsystem.reset();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Timer.getFPGATimestamp() - startTime > 1;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setGripper(false);
    }
}
