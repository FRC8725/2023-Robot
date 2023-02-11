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
    VisionManager visionManager;

    public GrabPiecesFromDouble(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics, VisionManager visionManager) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        this.visionManager = visionManager;
        addRequirements(armSubsystem, gripperSubsystem, pneumatics);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(Constants.PoseConstants.kLoadingZoneArmPose.getFirst(), Constants.PoseConstants.kLoadingZoneArmPose.getSecond());
        gripperSubsystem.setHorizontal(true);
        pneumatics.setGripper(true, false);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setGripper(false, visionManager.isCone());

        // reset Arm
        armSubsystem.reset();
        gripperSubsystem.reset();
        gripperSubsystem.setRollSetpoint(Math.PI);
    }
}
