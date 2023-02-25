package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


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
        armSubsystem.setSetpoint(Constants.PoseConstants.LOADING_ZONE_ARM_POSE.getFirst(), Constants.PoseConstants.LOADING_ZONE_ARM_POSE.getSecond());
        armSubsystem.setTransporting(false);
        armSubsystem.setPlacing(false);
        armSubsystem.setHorizontal(true);
        pneumatics.setGripper(true);
    }

    @Override
    public boolean isFinished() {
        return gripperSubsystem.isPiecesInRange();
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setGripper(false);
    }
}
