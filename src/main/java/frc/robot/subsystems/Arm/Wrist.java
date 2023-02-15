package frc.robot.subsystems.Arm;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMap.ElevatorPort;

public class Wrist extends SubsystemBase {

    private final static Wrist INSTANCE = new Wrist();

    @SuppressWarnings("WeakerAccess")
    public static Wrist getInstance() {
        return INSTANCE;
    }

    LazySparkMax wristMotor;

    ProfiledPIDController wristProfiledPIDController;

    DutyCycleEncoder absoluteEncoder;

    private Wrist() {

        wristMotor = new LazySparkMax(ElevatorPort.kWristMotor, ElevatorConstants.kWristGearRatio);
        wristMotor.setInverted(ElevatorConstants.kWristMotorInverted);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kWristAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWristAbsoluteEncoderOffset);

        wristProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWrist, ElevatorConstants.kIWrist, ElevatorConstants.kDWrist, ElevatorConstants.kWristControllerConstraints);
        wristProfiledPIDController.setTolerance(ElevatorConstants.kPIDGripperAngularToleranceRads);
        wristProfiledPIDController.disableContinuousInput();
        resetEncoder();
    }

    @Override
    public void periodic() {
//        wristMotor.set(MathUtil.clamp(wristProfiledPIDController.calculate(getAbsoluteEncoderRad() + Units.degreesToRadians(gyro.getPitch())), -ElevatorConstants.kMaxWristSpeed, ElevatorConstants.kMaxWristSpeed));
        wristMotor.set(MathUtil.clamp(wristProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ElevatorConstants.kMaxWristSpeed, ElevatorConstants.kMaxWristSpeed));
        SmartDashboard.putNumber("Wrist Absolute", absoluteEncoder.getAbsolutePosition());
//        SmartDashboard.putNumber("Wrist Encoder", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Wrist Setpoint Degrees", Units.radiansToDegrees(wristProfiledPIDController.getGoal().position));
    }

    public void resetEncoder() {
        wristMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        if (!absoluteEncoder.isConnected()) return wristMotor.getPositionAsRad();
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        measurement *= (ElevatorConstants.kWristAbosoluteEncoderInverted? -1: 1);
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI;
    }

    public void setWristSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinWristAngle, ElevatorConstants.kMaxWristAngle);
        wristProfiledPIDController.setGoal(setpoint);
    }

}

