package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LazyTalonFX;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.RobotMap;

public class SwerveModule {

    private final LazyTalonFX driveMotor, turningMotor;

    private final PIDController turningPIDController;

    private final CANCoder absoluteEncoder;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetAngle;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetAngle = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);
        CANCoderConfiguration absoluteEncoderConfiguration = new CANCoderConfiguration();

        absoluteEncoderConfiguration.sensorTimeBase = SensorTimeBase.Per100Ms_Legacy;
        absoluteEncoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        absoluteEncoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(absoluteEncoderConfiguration);
        absoluteEncoder.setPositionToAbsolute();

        double driveGearRatio = SwerveModuleConstants.DRIVE_MOTOR_GEAR_RATIO;
        driveMotor = new LazyTalonFX(driveMotorId, driveGearRatio);
        configDriveMotor(driveMotorReversed);

        double turningGearRatio = SwerveModuleConstants.TURNING_MOTOR_GEAR_RATIO;
        turningMotor = new LazyTalonFX(turningMotorId, turningGearRatio);
        configTurningMotor(turningMotorReversed);


//         turning Motor configuration

        turningPIDController = new PIDController(SwerveModuleConstants.KP_TURNING, SwerveModuleConstants.KI_TURNING, SwerveModuleConstants.KD_TURNING);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();
        putDashboard();
    }

    private void configDriveMotor(boolean reversed) {
        driveMotor.configFactoryDefault();
        driveMotor.setCurrent(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(reversed);
    }

    private void configTurningMotor(boolean reversed) {
        turningMotor.configFactoryDefault();
        turningMotor.setCurrent(false);
        turningMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setInverted(reversed);
    }

    public double getDrivePosition() {
        return driveMotor.getPositionAsRad();
    }

    public double getTurningPosition() {
        return turningMotor.getPositionAsRad();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocityAsMPS(SwerveModuleConstants.WHEEL_CIRCUMFERENCE);
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocityAsMPS(SwerveModuleConstants.WHEEL_CIRCUMFERENCE);
    }

    public double getDriveMeters() {
        return driveMotor.getPositionAsRad() * SwerveModuleConstants.WHEEL_DIAMETER_METERS / 2;
    }

    public double getAbsoluteEncoderRad() {
        var angle = absoluteEncoder.getAbsolutePosition() / 180. * Math.PI;
        angle -= absoluteEncoderOffsetAngle / 180 * Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), new Rotation2d(getTurningPosition()));
    }

    public void resetEncoders() {
        driveMotor.setRadPosition(0);
        turningMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        putDashboard();
    }


    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);

        switch (turningMotor.getDeviceID()) {
            case (RobotMap.DriverPort.FRONT_LEFT_TURNING_MOTOR_PORT):
            case (RobotMap.DriverPort.BACK_RIGHT_TURNING_MOTOR_PORT):
                turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), -Math.PI / 4));
                break;
            case (RobotMap.DriverPort.FRONT_RIGHT_TURNING_MOTOR_PORT):
            case (RobotMap.DriverPort.BACK_LEFT_TURNING_MOTOR_PORT):
                turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), Math.PI / 4));
                break;
        }

//        turningMotor.set(0);
    }

    public void putDashboard() {
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition());
    }

}