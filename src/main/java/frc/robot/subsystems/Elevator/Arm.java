package frc.robot.subsystems.Elevator;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    DigitalInput limitSwitch;

    PIDController armPIDController;

    // Elevator factors
    boolean isPIDControlled = false;
    double speed = 0;

    private Arm() {
        armMotor = new LazySparkMax(ElevatorPort.kArmMotor, ElevatorConstants.kArmGearRatio);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        limitSwitch = new DigitalInput(ElevatorPort.kArmLimitSwitch);
        armPIDController = new PIDController(ElevatorConstants.kPArm, ElevatorConstants.kIArm, ElevatorConstants.kDArm);
        armPIDController.setTolerance(ElevatorConstants.kPIDArmPositionTolerance);
    }

    @Override
    public void periodic() {
        if (isPIDControlled) {
            speed = MathUtil.clamp(armPIDController.calculate(armMotor.getPositionAsMeters(ElevatorConstants.kArmReelCircumferenceMeters)), -ElevatorConstants.kArmSpeed, ElevatorConstants.kArmSpeed);
            if (atSetpoint()) speed = 0;
        }
        armMotor.set(speed);
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMaxArmHeight, ElevatorConstants.kMaxArmHeight);
        isPIDControlled = true;
        armPIDController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return armPIDController.atSetpoint();
    }

    public double getSetpoint() {
        return armPIDController.getSetpoint();
    }

    public void set(double speed) {
        isPIDControlled = false;
        this.speed = speed;
    }

    public void zeroEncoder() {
        armMotor.setRadPosition(0);
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }
}

