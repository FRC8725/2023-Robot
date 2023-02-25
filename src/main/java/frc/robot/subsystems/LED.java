package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotMap;

public final class LED {

    private static final LED INSTANCE = new LED();

    public static LED getInstance() {
        return INSTANCE;
    }
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    private LED() {
        led = new AddressableLED(RobotMap.LEDPort.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(6);
        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();
    }

    public void setColor(Color color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        led.setData(ledBuffer);
    }

    private int rainbowFirstPixelHue = 0;

    // From Example LED Project
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        led.setData(ledBuffer);
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

}

