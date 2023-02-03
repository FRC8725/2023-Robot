package frc.robot.subsystems.Elevator;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.lib.LazyTalonFX;
import frc.robot.RobotMap.ElevatorPort;
import frc.robot.Constants.ElevatorConstants;

public class Gripper extends SubsystemBase {

    private final static Gripper INSTANCE = new Gripper();

    @SuppressWarnings("WeakerAccess")
    public static Gripper getInstance() {
        return INSTANCE;
    }

    LazySparkMax wristMotor;
    LazyTalonFX rollMotor;

    ProfiledPIDController wristProfiledPIDController, rollProfiledPIDController;

    DutyCycleEncoder absoluteEncoder;

    private Gripper() {
        rollMotor = new LazyTalonFX(ElevatorPort.kRollMotor, ElevatorConstants.kRollMotorGearRatio);
//        rollMotor = new LazySparkMax(ElevatorPort.kRollMotor, ElevatorConstants.kRollMotorGearRatio);
        rollMotor.setNeutralMode(NeutralMode.Coast);

        wristMotor = new LazySparkMax(ElevatorPort.kWristMotor, ElevatorConstants.kWristGearRatio);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kWristAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWristAbsoluteEncoderOffset);

        wristProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWrist, ElevatorConstants.kIWrist, ElevatorConstants.kDWrist, ElevatorConstants.kWristControllerConstraints);
        wristProfiledPIDController.setTolerance(ElevatorConstants.kPIDGripperAngularToleranceRads);
        wristProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        rollProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPRoll, ElevatorConstants.kIRoll, ElevatorConstants.kDRoll, ElevatorConstants.kRollControllerConstraints);
        rollProfiledPIDController.setTolerance(ElevatorConstants.kPIDRollAngularToleranceRads);
        rollProfiledPIDController.disableContinuousInput();
        rollMotor.setRadPosition(0);
        resetWristEncoder();
    }

    @Override
    public void periodic() {
        wristMotor.set(wristProfiledPIDController.calculate(absoluteEncoder.getAbsolutePosition()));
        rollMotor.set(rollProfiledPIDController.calculate(rollMotor.getPositionAsRad()));
        SmartDashboard.putNumber("Wrist Absolute", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Wrist Encoder", getWristEncoder());
    }

    public void resetWristEncoder() {
        wristMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-ElevatorConstants.kWristAbsoluteEncoderOffset;
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return -measurement*2*Math.PI; // Inverted
    }

    public void setWristSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinWristAngle, ElevatorConstants.kMaxWristAngle);
        wristProfiledPIDController.setGoal(setpoint);
    }

    public double getWristEncoder() {
        return wristMotor.getPositionAsRad();
    }

    public void setRollSetpoint(double setpoint) {
        rollProfiledPIDController.setGoal(setpoint);
    }

    public double getRollEncoder() {
        return rollMotor.getPositionAsRad();
    }
}

