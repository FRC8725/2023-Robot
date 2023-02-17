package frc.robot.subsystems.Elevator;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMap.ElevatorPort;

public class Winch extends SubsystemBase {

    private final static Winch INSTANCE = new Winch();

    @SuppressWarnings("WeakerAccess")
    public static Winch getInstance() {
        return INSTANCE;
    }

    LazySparkMax winchMotor;

    ProfiledPIDController winchProfiledPIDController;
    DutyCycleEncoder absoluteEncoder;

    private Winch() {
        winchMotor = new LazySparkMax(ElevatorPort.WINCH_MOTOR, ElevatorConstants.kWinchGearRatio);
        winchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        winchProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWinch, ElevatorConstants.kIWinch, ElevatorConstants.kDWinch, ElevatorConstants.kWinchControllerConstraints);
        winchProfiledPIDController.setTolerance(ElevatorConstants.kPIDWinchAngularToleranceRads);
        winchProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.WINCH_ABS_ENCODER);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWinchAbsoluteEncoderOffset);
        winchMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    @Override
    public void periodic() {
        winchMotor.set(winchProfiledPIDController.calculate(winchMotor.getPositionAsRad()));
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition();
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinWinchAngle, ElevatorConstants.kMaxWinchAngle);
        winchProfiledPIDController.setGoal(setpoint);
    }

    public double getSetpoint() {
        return winchProfiledPIDController.getSetpoint().position;
    }

    public double getEncoder() {
        return winchMotor.getPositionAsRad();
    }

    public void stop() {
        winchMotor.set(0);
        winchProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

