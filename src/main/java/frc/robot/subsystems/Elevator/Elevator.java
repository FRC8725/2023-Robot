package frc.robot.subsystems.Elevator;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMap.ElevatorPort;

public class Elevator extends SubsystemBase {

    private final static Elevator INSTANCE = new Elevator();

    @SuppressWarnings("WeakerAccess")
    public static Elevator getInstance() {
        return INSTANCE;
    }

    LazySparkMax elevatorMotor;

    DigitalInput limitSwitch;

    PIDController elevatorPIDController;

    // Elevator factors
    boolean isPIDControlled = false;
    double speed = 0;

    private Elevator() {
        elevatorMotor = new LazySparkMax(ElevatorPort.kElevatorMotor, ElevatorConstants.kElevatorGearRatio);
        elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        limitSwitch = new DigitalInput(ElevatorPort.kElevatorLimitSwitch);
        elevatorPIDController = new PIDController(ElevatorConstants.kPElevator, ElevatorConstants.kIElevator, ElevatorConstants.kDElevator);
        elevatorPIDController.setTolerance(ElevatorConstants.kPIDElevatorPositionTolerance);
    }

    @Override
    public void periodic() {
        if (isPIDControlled && !atSetpoint()) {
            if (!atSetpoint())speed = MathUtil.clamp(elevatorPIDController.calculate(elevatorMotor.getPositionAsMeters(ElevatorConstants.kElevatorReelCircumferenceMeters)), -1, 1);
            else speed = 0;
        }
        elevatorMotor.set(speed);
    }

    public void setSetpoint(double setpoint) {
        if (setpoint > ElevatorConstants.kMaxElevatorHeight && setpoint < ElevatorConstants.kMinElevatorHeight) return;
        isPIDControlled = true;
        elevatorPIDController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return elevatorPIDController.atSetpoint();
    }
    public double getSetpoint() {
        return elevatorPIDController.getSetpoint();
    }

    public void setSpeed(double speed) {
        isPIDControlled = false;
        this.speed = speed;
    }

    public void zeroEncoder() {
        elevatorMotor.setRadPosition(0);
    }

    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

}

