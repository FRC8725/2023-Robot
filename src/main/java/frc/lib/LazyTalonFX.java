package frc.lib;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class LazyTalonFX extends WPI_TalonFX {

    private final double gearRatio;
    public LazyTalonFX(int deviceNumber, double gearRatio) {
        super(deviceNumber);
    //         if the motor is for chassis, won't set the current limit.
        setCurrent(false);
        this.gearRatio = gearRatio;
    }


    public void setCurrent(boolean isHighCurrent) {
         if (isHighCurrent){
             configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 0.2));
             configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 40, 0.2));
         } else {
             configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 40, 0.2));
             configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 40, 0.2));
         }
    }

    public void setRadPosition(double rad) {
        setSelectedSensorPosition(rad / (2 * Math.PI) * (2048. / gearRatio));
    }

    public double getVelocityAsMPS(double circumference) {
        double motorRPM = getSelectedSensorVelocity() * (600. / 2048.);
        double mechRPM = motorRPM * gearRatio;
        return (mechRPM * circumference) / 60;
    }

    public double getPositionAsRad() {
        return getSelectedSensorPosition() / (2048 / gearRatio) * 2 * Math.PI;
    }
}