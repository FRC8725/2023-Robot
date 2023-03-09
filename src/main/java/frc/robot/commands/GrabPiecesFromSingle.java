package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;


public class GrabPiecesFromSingle extends CommandBase {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    Pneumatics pneumatics;
    double startTime;

    public GrabPiecesFromSingle(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        addRequirements(armSubsystem, gripperSubsystem, pneumatics);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(Constants.PoseConstants.LOADING_ZONE_SINGLE_POSE.getFirst(), Constants.PoseConstants.LOADING_ZONE_SINGLE_POSE.getSecond());
        armSubsystem.setTransporting(false);
        armSubsystem.setPlacing(false);
        armSubsystem.setHorizontal(false);
        pneumatics.setGripper(true);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - startTime < 1) return false;
        return gripperSubsystem.isPiecesInRange(true);
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setGripper(false);
        armSubsystem.reset();
    }
}
