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

    private final static Elbow INSTANCE = new Elbow();

    @SuppressWarnings("WeakerAccess")
    public static Elbow getInstance() {
        return INSTANCE;
    }

    LazySparkMax elbowMotor;


    ProfiledPIDController elbowProfiledPIDController;
    DutyCycleEncoder absoluteEncoder;

    double setpoint;

    private Elbow() {
        elbowMotor = new LazySparkMax(ArmPort.kElbowMotor, ArmConstants.kElbowGearRatio);
        elbowMotor.setCurrent(true);
        elbowMotor.setInverted(ArmConstants.kElbowMotorInverted);
        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        elbowProfiledPIDController = new ProfiledPIDController(ArmConstants.kPElbow, ArmConstants.kIElbow, ArmConstants.kDElbow, ArmConstants.kElbowControllerConstraints);
        elbowProfiledPIDController.setTolerance(ArmConstants.kPIDElbowAngularToleranceRads);
        elbowProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
//        elbowProfiledPIDController.disableContinuousInput();

        absoluteEncoder = new DutyCycleEncoder(ArmPort.kElbowAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ArmConstants.kElbowAbsoluteEncoderOffset);

        setpoint = Integer.MAX_VALUE;

        resetEncoder();
    }

    public void calculate() {
        double speed;
        if (setpoint < ArmConstants.kMinElbowAngle || setpoint > ArmConstants.kMaxElbowAngle) speed = 0;
        else speed = MathUtil.clamp(elbowProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ArmConstants.kMaxElbowSpeed, ArmConstants.kMaxElbowSpeed);
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
        measurement *= ArmConstants.kElbowAbosoluteEncoderInverted? -1: 1;
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ArmConstants.kMinElbowAngle, ArmConstants.kMaxElbowAngle);
        SmartDashboard.putNumber("Elbow Setpoint", setpoint);
        elbowProfiledPIDController.setGoal(setpoint);
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getAbsoluteEncoderRad()) < ArmConstants.kPIDWinchAngularToleranceRads;
    }

    public void stop() {
        elbowMotor.set(0);
        elbowProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

