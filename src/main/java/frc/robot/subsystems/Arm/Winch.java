package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMap.ArmPort;

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
        rightWinchMotor = new LazySparkMax(ArmPort.kRightWinchMotor, ArmConstants.kRightWinchGearRatio);
        rightWinchMotor.setCurrent(true);
        rightWinchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightWinchMotor.setInverted(ArmConstants.kWinchMotorInverted);
        leftWinchMotor = new LazySparkMax(ArmPort.kLeftWinchMotor, ArmConstants.kLeftWinchGearRatio);
        leftWinchMotor.setCurrent(true);
        leftWinchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftWinchMotor.follow(rightWinchMotor, true);

        winchProfiledPIDController = new ProfiledPIDController(ArmConstants.kPWinch, ArmConstants.kIWinch, ArmConstants.kDWinch, ArmConstants.kWinchControllerConstraints);
        winchProfiledPIDController.setTolerance(ArmConstants.kPIDWinchAngularToleranceRads);
        winchProfiledPIDController.disableContinuousInput();

        absoluteEncoder = new DutyCycleEncoder(ArmPort.kWinchAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ArmConstants.kWinchAbsoluteEncoderOffset);
        resetEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Winch Absolute", absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Winch Encoder", getAbsoluteEncoderRad());
//        SmartDashboard.putNumber("Winch Encoder", getAbsoluteEncoderRad());
        if (atSetpoint()) winchProfiledPIDController.setP(ArmConstants.kPBrake);
        double speed = MathUtil.clamp(winchProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ArmConstants.kMaxWinchSpeed, ArmConstants.kMaxWinchSpeed);
//        SmartDashboard.putNumber("Winch Speed", speed);
        rightWinchMotor.set(speed);
    }

    public void resetEncoder() {
        rightWinchMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        if (!absoluteEncoder.isConnected()) return rightWinchMotor.getPositionAsRad();
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        measurement *= (ArmConstants.kWinchAbosoluteEncoderInverted? -1: 1);
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ArmConstants.kMinWinchAngle, ArmConstants.kMaxWinchAngle);
        SmartDashboard.putNumber("Winch Setpoint", setpoint);
        winchProfiledPIDController.setP(ArmConstants.kPWinch);
        winchProfiledPIDController.setGoal(setpoint);
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getAbsoluteEncoderRad()) < ArmConstants.kPIDWinchAngularToleranceRads;
    }

    public void stop() {
        rightWinchMotor.set(0);
        winchProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

