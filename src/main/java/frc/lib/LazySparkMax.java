package frc.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class LazySparkMax extends CANSparkMax {

    private final double gearRatio;
    private double gearRatioLeader = 1;

    public LazySparkMax(int deviceNumber, double gearRatio) {
        super(deviceNumber, MotorType.kBrushless);
        setCurrent(false);
        getEncoder().setPositionConversionFactor(gearRatio);
        getEncoder().setVelocityConversionFactor(gearRatio);
        this.gearRatio = gearRatio;
    }

    public void setGearRatioLeader(double gearRatioLeader) {
        this.gearRatioLeader = gearRatioLeader;
    }

    public void setCurrent(boolean isHighCurrent) {
        if (isHighCurrent){
            setSmartCurrentLimit(60, 40);
        } else {
            setSmartCurrentLimit(50, 30);
        }
    }

    public void setRadPosition(double rad) {
        getEncoder().setPosition(rad / (2 * Math.PI));
    }

    /**
     * You need to setGearRatioFollow() with the leaderGearRatio before this function.
     */
    public void setSpeedFollowGearRatio(double speed) {
        set(speed / gearRatio * gearRatioLeader);
    }

    public double getPositionAsRad() {
        return getEncoder().getPosition()  * 2 * Math.PI;
    }

    public double getVelocityAsRad() {
        return getEncoder().getVelocity()  * 2 * Math.PI;
    }

    public double getPositionAsMeters(double circumference) {
        return getEncoder().getPosition() * circumference;
    }
}
