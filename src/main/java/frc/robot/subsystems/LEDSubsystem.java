package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

    private LEDSubsystem() {
        led = new AddressableLED(RobotMap.LEDPort.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.BUFFER_SIZE);
        led.setLength(LEDConstants.BUFFER_SIZE);
        led.start();
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

    public void setFrontColor(Color color, int part) {
        for (var i = LEDConstants.BUFFER_SIZE / 2 + ledBuffer.getLength() / 4 * part;
             i < ledBuffer.getLength() / 2 + ledBuffer.getLength() / 4
                     + (ledBuffer.getLength() / 4) * part;
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
            var hue = (rainbowFirstPixelHue + (i * 180 / (LEDConstants.BUFFER_SIZE * 2))) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

}

