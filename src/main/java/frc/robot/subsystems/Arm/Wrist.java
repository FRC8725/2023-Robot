package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
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
    double setpoint = 0;

    LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);

    private Wrist() {

        wristMotor = new LazySparkMax(ArmPort.WRIST_MOTOR, ArmConstants.WRIST_GEAR_RATIO);
        wristMotor.setInverted(ArmConstants.WRIST_MOTOR_INVERTED);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        absoluteEncoder = new DutyCycleEncoder(ArmPort.WRIST_ABSOLUTE_ENCODER);
        absoluteEncoder.setPositionOffset(ArmConstants.WRIST_ABSOLUTE_ENCODER_OFFSET);

        wristProfiledPIDController = new ProfiledPIDController(ArmConstants.P_WRIST, ArmConstants.I_WRIST, ArmConstants.D_WRIST, ArmConstants.WRIST_CONTROLLER_CONSTRAINTS);
        wristProfiledPIDController.setTolerance(ArmConstants.PID_GRIPPER_ANGULAR_TOLERANCE_RADS);
        wristProfiledPIDController.disableContinuousInput();
        resetEncoder();
    } 

    public void calculate() {
//        wristMotor.set(MathUtil.clamp(wristProfiledPIDController.calculate(getAbsoluteEncoderRad() + Units.degreesToRadians(gyro.getPitch())), -ElevatorConstants.MAX_WRIST_SPEED, ElevatorConstants.MAX_WRIST_SPEED));
//        if (atSetpoint()) wristProfiledPIDController.setP(ArmConstants.P_WRIST_BRAKE);
        double speed;
        speed = MathUtil.clamp(wristProfiledPIDController.calculate(getAbsoluteEncoderRad()), -ArmConstants.MAX_WRIST_SPEED, ArmConstants.MAX_WRIST_SPEED);
        wristMotor.set(speed);
        SmartDashboard.putNumber("Wrist Absolute", absoluteEncoder.getAbsolutePosition());
//        SmartDashboard.putNumber("Wrist Encoder", getAbsoluteEncoderRad());
//        SmartDashboard.putNumber("Wrist Setpoint Degrees", Units.radiansToDegrees(wristProfiledPIDController.getGoal().position));
    }

    public void resetEncoder() {
        wristMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        measurement *= (ArmConstants.WRIST_ABSOLUTE_ENCODER_INVERTED ? -1: 1);
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        measurement = filter.calculate(measurement);
        return measurement*2*Math.PI;
    }

    public void setWristSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ArmConstants.MIN_WRIST_ANGLE, ArmConstants.MAX_WRIST_ANGLE);
        wristProfiledPIDController.setGoal(setpoint);
//        wristProfiledPIDController.setP(ArmConstants.P_WRIST);
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(getAbsoluteEncoderRad() - setpoint) < ArmConstants.PID_GRIPPER_ANGULAR_TOLERANCE_RADS;
    }

}

