package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LazySparkMax;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMap.ArmPort;

public class Winch {

    private final static Winch INSTANCE = new Winch();

    @SuppressWarnings("WeakerAccess")
    public static Winch getInstance() {
        return INSTANCE;
    }

    LazySparkMax rightWinchMotor, leftWinchMotor;

    ProfiledPIDController winchProfiledPIDController;
    DutyCycleEncoder absoluteEncoder;

    double setpoint = Integer.MAX_VALUE;

    private Winch() {
        rightWinchMotor = new LazySparkMax(ArmPort.RIGHT_WINCH_MOTOR, ArmConstants.RIGHT_WINCH_GEAR_RATIO);
        rightWinchMotor.setCurrent(true);
        rightWinchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightWinchMotor.setInverted(ArmConstants.WINCH_MOTOR_INVERTED);
        leftWinchMotor = new LazySparkMax(ArmPort.LEFT_WINCH_MOTOR, ArmConstants.LEFT_WINCH_GEAR_RATIO);
        leftWinchMotor.setCurrent(true);
        leftWinchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        winchProfiledPIDController = new ProfiledPIDController(ArmConstants.P_WINCH, ArmConstants.I_WINCH, ArmConstants.D_WINCH, ArmConstants.WINCH_CONTROLLER_CONSTRAINTS);
        winchProfiledPIDController.setTolerance(ArmConstants.PID_WINCH_ANGULAR_TOLERANCE_RADS);
        winchProfiledPIDController.disableContinuousInput();
//        winchProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
//        winchProfiledPIDController.disableContinuousInput();

        absoluteEncoder = new DutyCycleEncoder(ArmPort.WINCH_ABSOLUTE_ENCODER);
        absoluteEncoder.setPositionOffset(ArmConstants.WINCH_ABSOLUTE_ENCODER_OFFSET);
        resetEncoder();
    }

    public void calculate() {
        SmartDashboard.putNumber("Winch Absolute", absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Winch Encoder", getAbsoluteEncoderRad());
//        SmartDashboard.putNumber("Winch Encoder", getAbsoluteEncoderRad());
        if (atSetpoint()) winchProfiledPIDController.setP(ArmConstants.P_WINCH_BRAKE);
        double speed = MathUtil.clamp(winchProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ArmConstants.MAX_WINCH_SPEED, ArmConstants.MAX_WINCH_SPEED);
        SmartDashboard.putNumber("Winch Speed", speed);
//        SmartDashboard.putNumber("Winch Speed", speed);
        rightWinchMotor.set(speed);
        leftWinchMotor.follow(rightWinchMotor, true);
    }

    public void setWinchMotor(double speed) {
        speed = MathUtil.clamp(speed, -ArmConstants.MAX_WINCH_SPEED, ArmConstants.MAX_WINCH_SPEED);
        leftWinchMotor.set(speed);
        leftWinchMotor.follow(rightWinchMotor, true);
    }

    public void resetEncoder() {
        rightWinchMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        measurement *= (ArmConstants.WINCH_ABOSOLUTE_ENCODER_INVERTED ? -1: 1);
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ArmConstants.MIN_WINCH_ANGLE, ArmConstants.MAX_WINCH_ANGLE);
        SmartDashboard.putNumber("Winch Setpoint", setpoint);
        winchProfiledPIDController.setP(ArmConstants.P_WINCH);
        winchProfiledPIDController.setGoal(setpoint);
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getAbsoluteEncoderRad()) < ArmConstants.PID_WINCH_ANGULAR_TOLERANCE_RADS;
    }

    public void stop() {
        rightWinchMotor.set(0);
        winchProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

