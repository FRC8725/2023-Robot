package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;


public class RunArmToPosition extends CommandBase {

    ArmSubsystem armSubsystem;
    GripperSubsystem gripperSubsystem;
    double elevatorPosition;
    double xAxis;
    double yAxis;
    boolean isHorizontal, isPlacing;

    public RunArmToPosition(ArmSubsystem armSubsystem,
                            GripperSubsystem gripperSubsystem,
                            double xAxis,
                            double yAxis,
                            boolean isHorizontal,
                            boolean isPlacing) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.isHorizontal = isHorizontal;
        this.isPlacing = isPlacing;
        addRequirements(armSubsystem);
    }
    public RunArmToPosition(ArmSubsystem armSubsystem,
                            GripperSubsystem gripperSubsystem,
                            Pair<Double, Double> armPose,
                            boolean isHorizontal,
                            boolean isPlacing) {
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.xAxis = armPose.getFirst();
        this.yAxis = armPose.getSecond();
        this.isHorizontal = isHorizontal;
        this.isPlacing = isPlacing;
        addRequirements(armSubsystem, gripperSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setSetpoint(xAxis, yAxis);
        armSubsystem.setTransporting(false);
        armSubsystem.setHorizontal(isHorizontal);
        armSubsystem.setPlacing(isPlacing);

    }

    @Override
    public boolean isFinished() {
        return armSubsystem.atSetpoint();
    }
}
