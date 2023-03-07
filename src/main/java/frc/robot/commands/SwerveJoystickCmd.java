package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction, decreaseSpeedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final PIDController thetaPIDController;
    private final Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                             Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
                             Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> decreaseSpeedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.decreaseSpeedFunction = decreaseSpeedFunction;
        this.thetaPIDController = new PIDController(DriveConstants.P_JOYSTICK_TURNING, 0, 0);
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.resetEncoders();
        this.thetaPIDController.setTolerance(1);
        this.thetaPIDController.setSetpoint(Math.PI);
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        var xSpeed = xSpdFunction.get();
        var ySpeed = ySpdFunction.get();
        var turningSpeed = turningSpdFunction.get();
        var decreaseSpeed = debouncer.calculate(decreaseSpeedFunction.get());

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.Joystick.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.Joystick.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.Joystick.DEADBAND ? turningSpeed : 0.0;


//        if (turntoGridaFunction.get()) {
//            turningSpeed = Math.max(-1, Math.min(1, thetaPIDController.calculate(swerveSubsystem.getRotation2d().getRadians())));
//        }

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND * (decreaseSpeed ? DriveConstants.DECREASE_DRIVING_SPEED_FACTOR : 1.);
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND * (decreaseSpeed ? DriveConstants.DECREASE_DRIVING_SPEED_FACTOR : 1.);
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * (decreaseSpeed ? DriveConstants.DECREASE_TURNING_SPEED_FACTOR : 1.);


        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            SmartDashboard.putString("chassis moving type", "field");
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            SmartDashboard.putString("chassis moving type", "robot");
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}