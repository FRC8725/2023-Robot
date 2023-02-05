package frc.robot.subsystems.Elevator;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    double setpoint;

    private Winch() {
        winchMotor = new LazySparkMax(ElevatorPort.kWinchMotor, ElevatorConstants.kWinchGearRatio);
        winchMotor.setCurrent(true);
        winchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        winchMotor.setInverted(true);
        winchProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWinch, ElevatorConstants.kIWinch, ElevatorConstants.kDWinch, ElevatorConstants.kWinchControllerConstraints);
        winchProfiledPIDController.setTolerance(ElevatorConstants.kPIDWinchAngularToleranceRads);
        winchProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kWinchAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWinchAbsoluteEncoderOffset);
        resetEncoder();
    }

    @Override
    public void periodic() {
        if(atSetpoint()) {
            if (Math.abs(winchMotor.getVelocityAsRad()) > ElevatorConstants.kVelocityToleranceRads) {
                winchMotor.set(ElevatorConstants.kSpdBrake * winchMotor.getVelocityAsRad() > 0 ? -1: 1);
            } else {
                winchMotor.set(0);
            }
        }
        else winchMotor.set(MathUtil.clamp(winchProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ElevatorConstants.kMaxWinchSpeed, ElevatorConstants.kMaxWinchSpeed));
        SmartDashboard.putNumber("Winch Absolute", absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Winch Encoder", getAbsoluteEncoderRad());
    }

    public void resetEncoder() {
        winchMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinWinchAngle, ElevatorConstants.kMaxWinchAngle);
        SmartDashboard.putNumber("Winch Setpoint", setpoint);
        winchProfiledPIDController.setGoal(setpoint);
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getAbsoluteEncoderRad()) < ElevatorConstants.kPIDWinchAngularToleranceRads;
    }

    public double getEncoder() {
        return winchMotor.getPositionAsRad();
    }

    public void stop() {
        winchMotor.set(0);
        winchProfiledPIDController.setGoal(winchMotor.getPositionAsRad());
    }
}

