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
        Timer.delay(.2);
    }

    @Override
    public boolean isFinished() {
        return gripperSubsystem.isPiecesInRange(false);
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setGripper(false);
        armSubsystem.reset();
    }
}
