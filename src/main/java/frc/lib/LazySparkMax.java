package frc.lib;

import com.revrobotics.CANSparkMax;

public class LazySparkMax extends CANSparkMax {

    private final double gearRatio;

    public LazySparkMax(int deviceNumber, double gearRatio) {
        super(deviceNumber, MotorType.kBrushless);
        setCurrent(false);
        this.gearRatio = gearRatio;
    }

//    public void setGearRatioLeader(double gearRatioLeader) {
//        this.gearRatioLeader = gearRatioLeader;
//    }

    public void setCurrent(boolean isHighCurrent) {
        if (isHighCurrent){
            setSmartCurrentLimit(40);
        } else {
            setSmartCurrentLimit(30);
        }
    }

    public void setRadPosition(double rad) {
        getEncoder().setPosition(rad / (2 * Math.PI) / gearRatio);
    }

    /**
     * You need to setGearRatioFollow() with the leaderGearRatio before using this function.
     */
//    public void setSpeedFollowGearRatio(double speed) {
//        set(speed / gearRatio * gearRatioLeader);
//    }

    public double getPositionAsRad() {
        return getEncoder().getPosition()  * 2 * Math.PI * gearRatio;
    }

    public double getVelocityAsRad() {
        return getEncoder().getVelocity() * gearRatio  * 2 * Math.PI;
    }

    public double getPositionAsMeters(double circumference) {
        return getEncoder().getPosition() * gearRatio * circumference;
    }
}
