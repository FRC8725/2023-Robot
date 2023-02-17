package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnOpposite extends CommandBase {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private double targetPosition;
    ProfiledPIDController controller;
    public TurnOpposite() {
        addRequirements(swerveSubsystem);
        controller = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    }
    @Override
    public void initialize() {
        controller.setGoal(swerveSubsystem.getHeading());
    }

    @Override
    public void execute() {
        var turningSpeed = controller.calculate(swerveSubsystem.getHeading());
    }

}
