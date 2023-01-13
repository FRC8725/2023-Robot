package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class LazyTalonFX extends WPI_TalonFX {

    private double gearRatio = 1;
    public LazyTalonFX(int deviceNumber, double gearRatio) {
        super(deviceNumber);
    //         if the motor is for chassis, won't set the current limit.
        setCurrent(false);
        this.gearRatio = gearRatio;
    }


    public void setCurrent(boolean isHighCurrent) {
         if (isHighCurrent){
             configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 10, 0.5));
             configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 48, 20, 1));
         } else {
             configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));
             configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 20, 1));
         }
    }

    public void setRadPosition(double rad) {
        setSelectedSensorPosition(rad / (2 * Math.PI) * (2048. / gearRatio));
    }

//    public double getPositionAsDegrees() {
//        return getSelectedSensorPosition() / (2048. / gearRatio) * 360.;
//    }

    public double getVelocityAsMPS(double circumference) {
        double motorRPM = getSelectedSensorVelocity() * (600. / 2048.);
        double mechRPM = motorRPM * gearRatio;
        double mechMPS = (mechRPM * circumference) / 60;
        return mechMPS;
    }
//
//    public void setPositionAsDegrees(double setpoint) {
//        set(TalonFXControlMode.Position, setpoint / 360.* (2048. / gearRatio));
//    }

    public double getPositionAsRad() {
        return getSelectedSensorPosition() / (2048 / gearRatio) * 2 * Math.PI;
    }
}