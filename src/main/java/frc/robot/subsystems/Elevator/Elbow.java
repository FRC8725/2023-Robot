package frc.robot.subsystems.Elevator;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    // Elevator factors
    boolean freeControl = false;

    private Elbow() {
        elbowMotor = new LazySparkMax(ElevatorPort.kElbowMotor, ElevatorConstants.kElbowGearRatio);
        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        elbowProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPElbow, ElevatorConstants.kIElbow, ElevatorConstants.kDElbow, ElevatorConstants.kElbowControllerConstraints);
        elbowProfiledPIDController.setTolerance(ElevatorConstants.kPIDElbowAngularToleranceRads);
        elbowProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kElbowAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kElbowAbsoluteEncoderOffset);
        resetEncoder();
    }

    @Override
    public void periodic() {
        if (freeControl) elbowMotor.set(0);
        else elbowMotor.set(elbowProfiledPIDController.calculate(elbowMotor.getPositionAsRad()));
    }

    public void resetEncoder() {
        elbowMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition();
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinElbowAngle, ElevatorConstants.kMaxElbowAngle);
        elbowProfiledPIDController.setGoal(setpoint);
    }

    public double getEncoder() {
        return elbowMotor.getPositionAsRad();
    }

    public void setFreeControl(boolean isFreeControl) {
        elbowMotor.setIdleMode(isFreeControl? CANSparkMax.IdleMode.kCoast: CANSparkMax.IdleMode.kBrake);
        elbowProfiledPIDController.setGoal(elbowMotor.getPositionAsRad());
        freeControl = isFreeControl;
    }

    public void stop() {
        elbowMotor.set(0);
        elbowProfiledPIDController.setGoal(elbowMotor.getPositionAsRad());
    }
}

