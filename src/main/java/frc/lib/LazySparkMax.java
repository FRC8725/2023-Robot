package frc.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class LazySparkMax extends CANSparkMax {

    private final RelativeEncoder encoder;

    public LazySparkMax(int deviceNumber, double gearRatio) {
        super(deviceNumber, MotorType.kBrushless);
        setCurrent(false);
        encoder = getEncoder();
        encoder.setPositionConversionFactor(gearRatio);
        encoder.setVelocityConversionFactor(gearRatio);
    }

    public void setCurrent(boolean isHighCurrent) {
        if (isHighCurrent){
            setSmartCurrentLimit(60, 40);
        } else {
            setSmartCurrentLimit(50, 30);
        }
    }

    public void setRadPosition(double rad) {
        encoder.setPosition(rad / (2 * Math.PI));
    }

    public double getPositionAsRad() {
        return encoder.getPosition()  * 2 * Math.PI;
    }

    public double getVelocityAsRad() {
        return encoder.getVelocity()  * 2 * Math.PI;
    }

    public double getPositionAsMeters(double circumference) {
        return encoder.getPosition() * circumference;
    }
}
