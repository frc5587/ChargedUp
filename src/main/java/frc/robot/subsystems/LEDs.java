package frc.robot.subsystems;

import org.frc5587.lib.advanced.RainbowLEDPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    public final AddressableLEDBuffer buffer;
    private final RainbowLEDPattern rainbowLEDPattern = new RainbowLEDPattern(6, 30, 60, 255);
    private boolean isRunningRainbow = false;
    private int rainbowIndexer = 0;

    public LEDs() {
        leds = new AddressableLED(LEDConstants.PORT);

        buffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    public void setColor(int r, int g, int b) {
        isRunningRainbow = false;
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    public void setYellow() {
        setColor(255, 255, 0);
    }

    public void setPurple() {
        setColor(178, 37, 188);
    }

    public void off() {
        setColor(0, 0, 0);
    }

    public void setRed() {
        setColor(255, 0, 0);
    }

    public void setBlue() {
        setColor(0, 0, 255);
    }

    public void setWhite() {
        setColor(255, 255, 255);
    }

    public void setRainbow() {
        isRunningRainbow = true;
    }

    @Override
    public void periodic() {
        if(!isRunningRainbow) {
            leds.setData(buffer);
            rainbowIndexer = 0;
        }
        if(isRunningRainbow) {
            leds.setData(rainbowLEDPattern.step(rainbowIndexer, buffer));
            rainbowIndexer++;
        }
    }
}
