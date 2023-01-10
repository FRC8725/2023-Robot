package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.lib.LazyTalonFX;
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

        double driveGearRatio = SwerveModuleConstants.kDriveMotorGearRatio;
        driveMotor = new LazyTalonFX(driveMotorId, driveGearRatio);
        configDriveMotor(driveMotorReversed);

        double turningGearRatio = SwerveModuleConstants.kTurningMotorGearRatio;
        turningMotor = new LazyTalonFX(turningMotorId, turningGearRatio);
        configTurningMotor(turningMotorReversed);


//         turning Motor configuration

        turningPIDController = new PIDController(SwerveModuleConstants.kPTurning, SwerveModuleConstants.kITurning, SwerveModuleConstants.kDTurning);
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
        return driveMotor.getVelocityAsMPS(SwerveModuleConstants.kWheelCircumference);
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocityAsMPS(SwerveModuleConstants.kWheelCircumference);
    }

    public double getDriveMeters() {
        return driveMotor.getPositionAsRad() * SwerveModuleConstants.kWheelDiameterMeters / 2;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition() / 360.;
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetAngle / 180 * Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), new Rotation2d(getTurningPosition()));
    }

    public void resetEncoders() {
        driveMotor.setRadPosition(0);
        turningMotor.setRadPosition(getAbsoluteEncoderRad());
//        driveEncoder.setPosition(0);
//        turningEncoder.setPosition(getAbsoluteEncoderRad());
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
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        putDashboard();
    }


    public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);

        switch (turningMotor.getDeviceID()) {
            case (RobotMap.DriverPort.kFrontLeftTurningMotorPort):
            case (RobotMap.DriverPort.kBackRightTurningMotorPort):
                turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), -Math.PI/4));
                break;
            case (RobotMap.DriverPort.kFrontRightTurningMotorPort):
            case (RobotMap.DriverPort.kBackLeftTurningMotorPort):
                turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), Math.PI/4));
                break;
        }

//        turningMotor.set(0);
    }

    public void putDashboard () {
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition());
       // SmartDashboard.putNumber("Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getPosition());
//       SmartDashboard.putNumber("Turing position " + turningMotor.getDeviceID(), turningMotor.getPositionAsDegrees());
    }
}