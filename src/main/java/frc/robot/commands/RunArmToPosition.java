package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;


public class RunArmToPosition extends CommandBase {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    double elevatorPosition;
    double xAxis;
    double yAxis;
    boolean isHorizontal;

    public RunArmToPosition(ArmSubsystem armSubsystem,
                            GripperSubsystem gripperSubsystem,
                            double xAxis,
                            double yAxis,
                            boolean isHorizontal) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.isHorizontal = isHorizontal;
        addRequirements(armSubsystem);
    }
    public RunArmToPosition(ArmSubsystem armSubsystem,
                            GripperSubsystem gripperSubsystem,
                            Pair<Double, Double> armPose,
                            boolean isHorizontal) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.xAxis = armPose.getFirst();
        this.yAxis = armPose.getSecond();
        this.isHorizontal = isHorizontal;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(xAxis, yAxis);
        gripperSubsystem.setHorizontal(isHorizontal);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.atSetpoint();
    }
}
