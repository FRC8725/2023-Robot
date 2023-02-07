package frc.robot.commands;

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
        armSubsystem.setSetpoint(Constants.PoseConstants.kLoadingZoneArmPose.getFirst(), Constants.PoseConstants.kLoadingZoneArmPose.getSecond());
        gripperSubsystem.setHorizontal(true);
        pneumatics.setGripper(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setGripper(false);

        // reset Arm
        armSubsystem.reset();
        gripperSubsystem.reset();
    }
}
