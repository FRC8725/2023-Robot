package frc.robot.subsystems.Elevator;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.lib.LazyTalonFX;
import frc.robot.RobotMap.ElevatorPort;
import frc.robot.Constants.ElevatorConstants;

import java.util.ArrayList;

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

    DutyCycleEncoder absoluteEncoder;
    ArrayList<DigitalInput> limitSwitch;

    private Gripper() {
        leftMotor = new LazyTalonFX(ElevatorPort.LEFT_INTAKE_MOTOR, ElevatorConstants.kIntakeGearRatio);
        rightMotor = new LazyTalonFX(ElevatorPort.RIGHT_INTAKE_MOTOR, ElevatorConstants.kIntakeGearRatio);
        rightMotor.setInverted(true);
        intakeMotorGroup = new MotorControllerGroup(leftMotor, rightMotor);
        intakeMotorGroup.setInverted(true);

        wristMotor = new LazySparkMax(ElevatorPort.WRIST_MOTOR, ElevatorConstants.kWristGearRatio);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.WRIST_ABS_ENCODER);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWristAbsoluteEncoderOffset);
        wristMotor.setRadPosition(getAbsoluteEncoderRad());

        wristProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWrist, ElevatorConstants.kIWrist, ElevatorConstants.kDWrist, ElevatorConstants.kWristControllerConstraints);
        wristProfiledPIDController.setTolerance(ElevatorConstants.PID_GRIPPER_ANGULAR_TOLERANCE_RADS);
        wristProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
        wristProfiledPIDController.reset(getAbsoluteEncoderRad());

        limitSwitch = new ArrayList<DigitalInput>();
        limitSwitch.add(new DigitalInput(ElevatorPort.GRIPPER_LIMIT_SWITCH[0]));
        limitSwitch.add(new DigitalInput(ElevatorPort.GRIPPER_LIMIT_SWITCH[1]));
    }

    @Override
    public void periodic() {
        wristMotor.set(wristProfiledPIDController.calculate(wristMotor.getPositionAsRad()));
        SmartDashboard.putNumber("Wrist Angle", wristMotor.getPositionAsRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition();
        return measurement*2*Math.PI;
    }

    public void runIntake(double speed) {
        intakeMotorGroup.set(MathUtil.clamp(speed, -1, 1));
    }

    public boolean getIntakeLimitSwitch() {
        return limitSwitch.get(0).get() && limitSwitch.get(1).get();
    }

    public void setWristSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinWristAngle, ElevatorConstants.kMaxWristAngle);
        wristProfiledPIDController.setGoal(setpoint);
    }

    public double getWristEncoder() {
        return wristMotor.getPositionAsRad();
    }

    public void stop() {
        intakeMotorGroup.set(0);
        wristMotor.set(0);
        wristProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

