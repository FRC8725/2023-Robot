package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;


public class GrabPieces extends CommandBase {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    Pneumatics pneumatics;
    VisionManager visionManager;
    double startTime;

    public GrabPieces(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem, Pneumatics pneumatics) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.pneumatics = pneumatics;
        addRequirements(armSubsystem, gripperSubsystem, pneumatics);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(Constants.PoseConstants.GROUND_ARM_POSE.getFirst(), Constants.PoseConstants.GROUND_ARM_POSE.getSecond());
        armSubsystem.setDesiredWinchAngle(0);
        armSubsystem.setTransporting(false);
        armSubsystem.setPlacing(false);
        armSubsystem.setHorizontal(true);
        pneumatics.setGripper(true);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (armSubsystem.atElbowSetpoint()) armSubsystem.setSetpoint(Constants.PoseConstants.GROUND_ARM_POSE.getFirst(), Constants.PoseConstants.GROUND_ARM_POSE.getSecond());
        if (Timer.getFPGATimestamp() - startTime > 1.5)gripperSubsystem.isPiecesInRange(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setGripper(false);
        
    }
}
