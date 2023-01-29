package frc.robot.subsystems.Elevator;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMap.ElevatorPort;
import frc.lib.LazySparkMax;

public class Arm extends SubsystemBase {

    private final static Arm INSTANCE = new Arm();

    @SuppressWarnings("WeakerAccess")
    public static Arm getInstance() {
        return INSTANCE;
    }

    LazySparkMax armMotor;


    ProfiledPIDController armProfiledPIDController;
    DutyCycleEncoder absoluteEncoder;

    // Elevator factors
    boolean freeControl = false;

    private Arm() {
        armMotor = new LazySparkMax(ElevatorPort.kArmMotor, ElevatorConstants.kArmGearRatio);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        armProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPArm, ElevatorConstants.kIArm, ElevatorConstants.kDArm, ElevatorConstants.kArmControllerConstraints);
        armProfiledPIDController.setTolerance(ElevatorConstants.kPIDArmAngularToleranceRads);
        armProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kArmAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kArmAbsoluteEncoderOffset);
        armMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    @Override
    public void periodic() {
        if (freeControl) armMotor.set(0);
        else armMotor.set(armProfiledPIDController.calculate(armMotor.getPositionAsRad()));
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition();
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinArmAngle, ElevatorConstants.kMaxArmAngle);
        armProfiledPIDController.setGoal(setpoint);
    }

    public double getEncoder() {
        return armMotor.getPositionAsRad();
    }

    public void setFreeControl(boolean isFreeControl) {
        armMotor.setIdleMode(isFreeControl? CANSparkMax.IdleMode.kCoast: CANSparkMax.IdleMode.kBrake);
        freeControl = isFreeControl;
    }

    public void stop() {
        armMotor.set(0);
        armProfiledPIDController.setGoal(armMotor.getPositionAsRad());
    }
}

