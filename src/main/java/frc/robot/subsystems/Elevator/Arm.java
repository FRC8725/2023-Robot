package frc.robot.subsystems.Elevator;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    DigitalInput limitSwitch;

    PIDController armPIDController;

    // Elevator factors
    boolean isPIDControlled = false;
    double speed = 0;

    private Arm() {
        armMotor = new LazySparkMax(ElevatorPort.ARM_MOTOR, ElevatorConstants.kArmGearRatio);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        limitSwitch = new DigitalInput(ElevatorPort.ARM_LIMIT_SWITCH);
        armPIDController = new PIDController(ElevatorConstants.kPArm, ElevatorConstants.kIArm, ElevatorConstants.kDArm);
        armPIDController.setTolerance(ElevatorConstants.PID_ARM_POSITION_TOLERANCE);
    }

    @Override
    public void periodic() {
        double output = armPIDController.calculate(armMotor.getPositionAsMeters(ElevatorConstants.ARM_REEL_CIRCUMFERENCE_METERS));
        if (isPIDControlled) {
            if (armPIDController.atSetpoint()) speed = 0;
            else speed = MathUtil.clamp(output, -ElevatorConstants.kArmSpeed, ElevatorConstants.kArmSpeed);
        }
        armMotor.set(speed);
        SmartDashboard.putNumber("arm position", armMotor.getPositionAsMeters(ElevatorConstants.ARM_REEL_CIRCUMFERENCE_METERS));
        SmartDashboard.putNumber("arm setpoint", armPIDController.getSetpoint());
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinArmHeight, ElevatorConstants.kMaxArmHeight);
        isPIDControlled = true;
        armPIDController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return armPIDController.atSetpoint();
    }

    public double getEncoder() {
        return armMotor.getPositionAsMeters(ElevatorConstants.ARM_REEL_CIRCUMFERENCE_METERS);
    }

    public void set(double speed) {
        isPIDControlled = false;
        this.speed = speed;
    }

    public void zeroEncoder() {
        armMotor.setRadPosition(0);
    }

    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }
}

