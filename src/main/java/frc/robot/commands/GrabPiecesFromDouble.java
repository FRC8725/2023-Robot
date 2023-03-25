
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;


public class GrabPiecesFromDouble extends CommandBase {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    Pneumatics pneumatics;
    double startTime;
    boolean firstLoop;

    public GrabPiecesFromDouble(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        addRequirements(armSubsystem, gripperSubsystem, pneumatics);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(Constants.PoseConstants.LOADING_ZONE_DOUBLE_POSE.getFirst(), Constants.PoseConstants.LOADING_ZONE_DOUBLE_POSE.getSecond());
        armSubsystem.setTransporting(false);
        armSubsystem.setPlacing(false);
        armSubsystem.setHorizontal(true);
        pneumatics.setGripper(true);
        startTime = Timer.getFPGATimestamp();
        firstLoop = true;
    }

    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - startTime < 1.5) return;
        gripperSubsystem.isPiecesInRange(false);
        // if (gripperSubsystem.isPiecesInRange(false) && firstLoop) {
        //     pneumatics.setGripper(false);
        //     armSubsystem.setTransporting(true);
        //     firstLoop = false;
        //     startTime = Timer.getFPGATimestamp();
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setGripper(false);
        if (!interrupted) armSubsystem.reset();
    }
}
