package frc.robot.subsystems.Elevator;


import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

    LazyTalonFX leftMotor, rightMotor;
    LazySparkMax wristMotor;
    MotorControllerGroup intakeMotorGroup;

    ProfiledPIDController wristProfiledPIDController;

    CANCoder absoluteEncoder;

    private Gripper() {
        leftMotor = new LazyTalonFX(ElevatorPort.kLeftIntakeMotor, ElevatorConstants.kIntakeGearRatio);
        rightMotor = new LazyTalonFX(ElevatorPort.kRightIntakeMotor, ElevatorConstants.kIntakeGearRatio);
        rightMotor.setInverted(true);
        intakeMotorGroup = new MotorControllerGroup(leftMotor, rightMotor);

        wristMotor = new LazySparkMax(ElevatorPort.kWristMotor, ElevatorConstants.kWristGearRatio);

        absoluteEncoder = new CANCoder(ElevatorPort.kWristAbsoluteEncoder);
        CANCoderConfiguration absoluteEncoderConfiguration = new CANCoderConfiguration();
        absoluteEncoderConfiguration.sensorTimeBase = SensorTimeBase.Per100Ms_Legacy;
        absoluteEncoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        absoluteEncoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(absoluteEncoderConfiguration);
        absoluteEncoder.setPositionToAbsolute();

        wristProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWrist, ElevatorConstants.kIWrist, ElevatorConstants.kIWrist, ElevatorConstants.kWristControllerConstraints);
        wristProfiledPIDController.setTolerance(ElevatorConstants.kPIDGripperAngularToleranceRads);
        wristProfiledPIDController.reset(getAbsoluteEncoderRad());
    }

    @Override
    public void periodic() {
        wristMotor.set(wristProfiledPIDController.calculate(getAbsoluteEncoderRad()));
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-ElevatorConstants.kWristAbsoluteEncoderOffsetAngle;
        return measurement/180*Math.PI;
    }

    public void runIntake(double speed) {
        intakeMotorGroup.set(MathUtil.clamp(speed, -1, 1));
    }

    public void setWristSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, Math.PI);
        wristProfiledPIDController.setGoal(setpoint);
    }

    public double getWristSetpoint() {
        return wristProfiledPIDController.getSetpoint().position;
    }

    public void stop() {
        intakeMotorGroup.set(0);
        wristMotor.set(0);
        wristProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

