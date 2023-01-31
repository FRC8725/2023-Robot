package frc.robot.subsystems.Elevator;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.robot.RobotMap.ElevatorPort;
import frc.robot.Constants.ElevatorConstants;

public class Gripper extends SubsystemBase {

    private final static Gripper INSTANCE = new Gripper();

    @SuppressWarnings("WeakerAccess")
    public static Gripper getInstance() {
        return INSTANCE;
    }

    LazySparkMax wristMotor, rollMotor;

    ProfiledPIDController wristProfiledPIDController, rollProfiledPIDController;

    DutyCycleEncoder absoluteEncoder;

    private Gripper() {
        rollMotor = new LazySparkMax(ElevatorPort.kRollMotor, ElevatorConstants.kRollMotorGearRatio);
        rollMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        wristMotor = new LazySparkMax(ElevatorPort.kWristMotor, ElevatorConstants.kWristGearRatio);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kWristAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWristAbsoluteEncoderOffset);
        wristMotor.setRadPosition(getAbsoluteEncoderRad());

        wristProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWrist, ElevatorConstants.kIWrist, ElevatorConstants.kDWrist, ElevatorConstants.kWristControllerConstraints);
        wristProfiledPIDController.setTolerance(ElevatorConstants.kPIDGripperAngularToleranceRads);
        wristProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        rollProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPRoll, ElevatorConstants.kIRoll, ElevatorConstants.kDRoll, ElevatorConstants.kRollControllerConstraints);
        rollProfiledPIDController.setTolerance(ElevatorConstants.kPIDRollAngularToleranceRads);
        rollProfiledPIDController.disableContinuousInput();
        rollMotor.setRadPosition(Math.PI/2);
        resetWristEncoder();
    }

    @Override
    public void periodic() {
        wristMotor.set(wristProfiledPIDController.calculate(wristMotor.getPositionAsRad()));
        rollMotor.set(rollProfiledPIDController.calculate(rollMotor.getPositionAsRad()));
        SmartDashboard.putNumber("Wrist Angle", wristMotor.getPositionAsRad());
    }

    public void resetWristEncoder() {
        wristMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition();
        return measurement*2*Math.PI;
    }

    public void setWristSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinWristAngle, ElevatorConstants.kMaxWristAngle);
        wristProfiledPIDController.setGoal(setpoint);
    }

    public double getWristEncoder() {
        return wristMotor.getPositionAsRad();
    }

    public void setRollSetpoint(double setpoint) {
        wristProfiledPIDController.setGoal(setpoint);
    }

    public double getRollEncoder() {
        return wristMotor.getPositionAsRad();
    }

    public void stop() {
        rollMotor.set(0);
        rollProfiledPIDController.setGoal(rollMotor.get());
        wristMotor.set(0);
        wristProfiledPIDController.setGoal(wristMotor.get());
    }
}

