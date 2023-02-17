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
        elevatorMotor = new LazySparkMax(ElevatorPort.ELEVATOR_MOTOR, ElevatorConstants.kElevatorGearRatio);
        elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        limitSwitch = new DigitalInput(ElevatorPort.ELEVATOR_LIMIT_SWITCH);
        elevatorPIDController = new PIDController(ElevatorConstants.kPElevator, ElevatorConstants.kIElevator, ElevatorConstants.kDElevator);
        elevatorPIDController.setTolerance(ElevatorConstants.PID_ELEVATOR_POSITION_TOLERANCE);
    }

    @Override
    public void periodic() {
        double output = elevatorPIDController.calculate(elevatorMotor.getPositionAsMeters(ElevatorConstants.kElevatorReelCircumferenceMeters));
        if (isPIDControlled) {
            if (elevatorPIDController.atSetpoint()) speed = 0;
            else speed = MathUtil.clamp(output, -ElevatorConstants.kElevatorSpeed, ElevatorConstants.kElevatorSpeed);
        }
        elevatorMotor.set(speed);
//        SmartDashboard.putNumber("elevator speed", elevatorMotor.get());
//        SmartDashboard.putNumber("elevator position", elevatorMotor.getPositionAsMeters(ElevatorConstants.kElevatorReelCircumferenceMeters));
//        SmartDashboard.putNumber("elevator position error", elevatorPIDController.getPositionError());
//        SmartDashboard.putBoolean("elevator atSetpoint", elevatorPIDController.atSetpoint());
//        SmartDashboard.putNumber("elevator setpoint", elevatorPIDController.getSetpoint());
//        SmartDashboard.putNumber("elevator tolerance", elevatorPIDController.getPositionTolerance());
//        SmartDashboard.putBoolean("limitSwitch", getLimitSwitch());
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinElevatorHeight, ElevatorConstants.kMaxElevatorHeight);
        isPIDControlled = true;
        elevatorPIDController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return elevatorPIDController.atSetpoint();
    }
    public double getEncoder() {
        return elevatorMotor.getPositionAsMeters(ElevatorConstants.kElevatorReelCircumferenceMeters);
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

