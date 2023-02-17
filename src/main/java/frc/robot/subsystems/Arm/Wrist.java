package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LazySparkMax;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMap.ArmPort;

public class Wrist {

    private final static Wrist INSTANCE = new Wrist();

    @SuppressWarnings("WeakerAccess")
    public static Wrist getInstance() {
        return INSTANCE;
    }

    LazySparkMax wristMotor;

    ProfiledPIDController wristProfiledPIDController;

    DutyCycleEncoder absoluteEncoder;

    private Wrist() {

        wristMotor = new LazySparkMax(ArmPort.kWristMotor, ArmConstants.kWristGearRatio);
        wristMotor.setInverted(ArmConstants.kWristMotorInverted);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        absoluteEncoder = new DutyCycleEncoder(ArmPort.kWristAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ArmConstants.kWristAbsoluteEncoderOffset);

        wristProfiledPIDController = new ProfiledPIDController(ArmConstants.kPWrist, ArmConstants.kIWrist, ArmConstants.kDWrist, ArmConstants.kWristControllerConstraints);
        wristProfiledPIDController.setTolerance(ArmConstants.kPIDGripperAngularToleranceRads);
        wristProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoder();
    }

    public void calculate() {
//        wristMotor.set(MathUtil.clamp(wristProfiledPIDController.calculate(getAbsoluteEncoderRad() + Units.degreesToRadians(gyro.getPitch())), -ElevatorConstants.kMaxWristSpeed, ElevatorConstants.kMaxWristSpeed));
        wristMotor.set(MathUtil.clamp(wristProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ArmConstants.kMaxWristSpeed, ArmConstants.kMaxWristSpeed));
        SmartDashboard.putNumber("Wrist Absolute", absoluteEncoder.getAbsolutePosition());
//        SmartDashboard.putNumber("Wrist Encoder", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Wrist Setpoint Degrees", Units.radiansToDegrees(wristProfiledPIDController.getGoal().position));
    }

    public void resetEncoder() {
        wristMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        measurement *= (ArmConstants.kWristAbosoluteEncoderInverted? -1: 1);
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI;
    }

    public void setWristSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ArmConstants.kMinWristAngle, ArmConstants.kMaxWristAngle);
        wristProfiledPIDController.setGoal(setpoint);
    }

}

