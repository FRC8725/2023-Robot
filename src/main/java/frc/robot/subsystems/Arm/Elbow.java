package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMap.ArmPort;
import frc.lib.LazySparkMax;

public class Elbow {
    private static final Elbow INSTANCE = new Elbow();
    private final LazySparkMax elbowMotor;
    private final ProfiledPIDController elbowProfiledPIDController;
    private final DutyCycleEncoder absoluteEncoder;
    private double setpoint;

    private Elbow() {
        this.elbowMotor = new LazySparkMax(ArmPort.ELBOW_MOTOR, ArmConstants.ELBOW_GEAR_RATIO);
        this.elbowMotor.setCurrent(true);
        this.elbowMotor.setInverted(ArmConstants.ELBOW_MOTOR_INVERTED);
        this.elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.elbowProfiledPIDController = new ProfiledPIDController(ArmConstants.P_ELBOW, ArmConstants.I_ELBOW, ArmConstants.D_ELBOW, ArmConstants.ELBOW_CONTROLLER_CONSTRAINTS);
        this.elbowProfiledPIDController.setTolerance(ArmConstants.PID_ELBOW_ANGULAR_TOLERANCE_RADS);
        this.elbowProfiledPIDController.disableContinuousInput();
//        this.elbowProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
//        elbowProfiledPIDController.disableContinuousInput();
        this.absoluteEncoder = new DutyCycleEncoder(ArmPort.ELBOW_ABSOLUTE_ENCODER);
        this.absoluteEncoder.setPositionOffset(ArmConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET);
        this.resetEncoder();
    }

    @SuppressWarnings("WeakerAccess")
    public static Elbow getInstance() {
        return INSTANCE;
    }

    public void calculate() {
        double speed;
        if (setpoint < ArmConstants.MIN_ELBOW_ANGLE || setpoint > ArmConstants.MAX_ELBOW_ANGLE) speed = 0;
        else speed = MathUtil.clamp(elbowProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ArmConstants.MAX_ELBOW_SPEED, ArmConstants.MAX_ELBOW_SPEED);
        elbowMotor.set(speed);
        SmartDashboard.putNumber("Elbow Absolute", absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Elbow Encoder", getAbsoluteEncoderRad());
//        SmartDashboard.putNumber("Elbow Encoder", getAbsoluteEncoderRad());
    }

    public void resetEncoder() {
        elbowMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        measurement *= ArmConstants.ELBOW_ABSOLUTE_ENCODER_INVERTED ? -1: 1;
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ArmConstants.MIN_ELBOW_ANGLE, ArmConstants.MAX_ELBOW_ANGLE);
        SmartDashboard.putNumber("Elbow Setpoint", setpoint);
        elbowProfiledPIDController.setGoal(setpoint);
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getAbsoluteEncoderRad()) < ArmConstants.PID_ELBOW_ANGULAR_TOLERANCE_RADS;
    }

    public void stop() {
        elbowMotor.set(0);
        elbowProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

