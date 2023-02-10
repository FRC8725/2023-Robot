package frc.robot.subsystems.Arm;


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

    LazySparkMax rightWinchMotor, leftWinchMotor;

    ProfiledPIDController winchProfiledPIDController;
    DutyCycleEncoder absoluteEncoder;

    double setpoint;

    private Winch() {
        rightWinchMotor = new LazySparkMax(ElevatorPort.kRightWinchMotor, ElevatorConstants.kRightWinchGearRatio);
        rightWinchMotor.setCurrent(true);
        rightWinchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightWinchMotor.setInverted(ElevatorConstants.kWinchMotorInverted);
        leftWinchMotor = new LazySparkMax(ElevatorPort.kLeftWinchMotor, ElevatorConstants.kLeftWinchGearRatio);
        leftWinchMotor.setCurrent(true);
        leftWinchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftWinchMotor.setInverted(!rightWinchMotor.getInverted());
        leftWinchMotor.setGearRatioLeader(ElevatorConstants.kRightWinchGearRatio);

        winchProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWinch, ElevatorConstants.kIWinch, ElevatorConstants.kDWinch, ElevatorConstants.kWinchControllerConstraints);
        winchProfiledPIDController.setTolerance(ElevatorConstants.kPIDWinchAngularToleranceRads);
        winchProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kWinchAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWinchAbsoluteEncoderOffset);
        resetEncoder();
    }

    @Override
    public void periodic() {
        if (atSetpoint()) {
            winchProfiledPIDController.setP(ElevatorConstants.kPBrake);
        } else {
            winchProfiledPIDController.setP(ElevatorConstants.kPWinch);
        }
        double speed = MathUtil.clamp(winchProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ElevatorConstants.kMaxWinchSpeed, ElevatorConstants.kMaxWinchSpeed);
        rightWinchMotor.set(speed);
        leftWinchMotor.setSpeedFollowGearRatio(speed);
        SmartDashboard.putNumber("Winch Absolute", absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Winch Encoder", getAbsoluteEncoderRad());
    }

    public void resetEncoder() {
        rightWinchMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI*(ElevatorConstants.kWinchAbosoluteEncoderInverted? -1: 1);
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
        return rightWinchMotor.getPositionAsRad();
    }

    public void stop() {
        rightWinchMotor.set(0);
        winchProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

