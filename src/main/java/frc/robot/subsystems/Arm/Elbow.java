package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMap.ElevatorPort;
import frc.lib.LazySparkMax;

public class Elbow extends SubsystemBase {

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
        elbowMotor = new LazySparkMax(ElevatorPort.kElbowMotor, ElevatorConstants.kElbowGearRatio);
        elbowMotor.setCurrent(true);
        elbowMotor.setInverted(ElevatorConstants.kElbowMotorInverted);
        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        elbowProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPElbow, ElevatorConstants.kIElbow, ElevatorConstants.kDElbow, ElevatorConstants.kElbowControllerConstraints);
        elbowProfiledPIDController.setTolerance(ElevatorConstants.kPIDElbowAngularToleranceRads);
        elbowProfiledPIDController.disableContinuousInput();

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kElbowAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kElbowAbsoluteEncoderOffset);
        resetEncoder();
    }

    @Override
    public void periodic() {
        elbowMotor.set(MathUtil.clamp(elbowProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ElevatorConstants.kMaxElbowSpeed, ElevatorConstants.kMaxElbowSpeed));
        SmartDashboard.putNumber("Elbow Absolute", absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Elbow Encoder", elbowMotor.getPositionAsRad());
//        SmartDashboard.putNumber("Elbow Encoder", getAbsoluteEncoderRad());
    }

    public void resetEncoder() {
        elbowMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        if (!absoluteEncoder.isConnected()) return elbowMotor.getPositionAsRad();
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        measurement *= ElevatorConstants.kElbowAbosoluteEncoderInverted? -1: 1;
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinElbowAngle, ElevatorConstants.kMaxElbowAngle);
        SmartDashboard.putNumber("Elbow Setpoint", setpoint);
        elbowProfiledPIDController.setGoal(setpoint);
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getAbsoluteEncoderRad()) < ElevatorConstants.kPIDWinchAngularToleranceRads;
    }

    public void stop() {
        elbowMotor.set(0);
        elbowProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}

