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
    boolean isFirstLoop;

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
        this.isFirstLoop = true;
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
        this.isFirstLoop = true;
        addRequirements(armSubsystem, gripperSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.moveTwice(xAxis, yAxis);
        armSubsystem.setTransporting(true);
        armSubsystem.setHorizontal(isHorizontal);
        armSubsystem.setPlacing(isPlacing);
    }

    @Override
    public void execute() {
        if (isFirstLoop && armSubsystem.atSetpoint()) {
            armSubsystem.setTransporting(false);
            armSubsystem.moveTwice(xAxis, yAxis);
            isFirstLoop = false;
        }
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.atSetpoint() && !isFirstLoop;
    }

    @Override
    public void end(boolean interrupted) {
        isFirstLoop = true;
    }
}
