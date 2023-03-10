package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotMap;

public final class LEDSubsystem extends SubsystemBase {

    private static final LEDSubsystem INSTANCE = new LEDSubsystem();

    public static LEDSubsystem getInstance() {
        return INSTANCE;
    }
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    boolean isIdle = true;

    private LEDSubsystem() {
        led = new AddressableLED(RobotMap.LEDPort.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.BUFFER_SIZE);
        led.setLength(LEDConstants.BUFFER_SIZE);
        led.start();
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
        if (isIdle) idleMode();
        if (DriverStation.isDisabled()) isFirstAuto = true;
    }

    public void setFrontColor(Color color, int part) {
        for (var i = LEDConstants.BUFFER_SIZE / 2 + ledBuffer.getLength() / 4 * part;
             i < ledBuffer.getLength() / 2 + ledBuffer.getLength() / 4
                     + (ledBuffer.getLength() / 4) * part;
             i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setFrontColor(Color color) {
        for (var i = LEDConstants.BUFFER_SIZE / 2;
             i < ledBuffer.getLength();
             i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setBackColor(Color color) {
        for (var i = 0; i < ledBuffer.getLength() / 2; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    private int rainbowFirstPixelHue = 0;

    // From Example LED Project
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < LEDConstants.BUFFER_SIZE; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (rainbowFirstPixelHue + (i * 180 / (LEDConstants.BUFFER_SIZE))) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    private int idleFirstPixelLightness = 0;

    public void idleMode() {
        // For every pixel
        for (var i = 0; i < LEDConstants.BUFFER_SIZE; i++) {
            int lightness = idleFirstPixelLightness + (int)(255 * ((i + 1) / 30.));
            lightness %= 255;
            ledBuffer.setHSV(i, 5, 255, lightness);
        }
        idleFirstPixelLightness += 2;
        idleFirstPixelLightness %= 255;
    }

    boolean isFirstAuto = true;
    double startTime;

    public void autoMode() {
        if (isFirstAuto) {
            startTime = Timer.getFPGATimestamp();
            isFirstAuto = false;
        }
        var timeCounter = LEDConstants.BUFFER_SIZE / 2 * ((Timer.getFPGATimestamp() - startTime) / 15.);
        if (timeCounter >= LEDConstants.BUFFER_SIZE / 2.) return;
        for (var i = 0; i < LEDConstants.BUFFER_SIZE; i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }
        int i = 0;
        for (; i < timeCounter; i++) {
            ledBuffer.setHSV(i, 80, 255, 255);
            ledBuffer.setHSV(LEDConstants.BUFFER_SIZE - 1 - i, 80, 255, 255);
        }
        ledBuffer.setHSV(i, 80, 255, (int) (255 * (timeCounter - (int)timeCounter)));
        ledBuffer.setHSV(LEDConstants.BUFFER_SIZE - 1 - i, 80, 255, (int) (255 * (timeCounter - (int)timeCounter)));
    }

    public void setIdle(boolean isIdle) {
        this.isIdle = isIdle;
    }

}

