package frc.robot.subsystems;

import org.frc5587.lib.advanced.RainbowLEDPattern;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.PurpleYellowChaseAndBlink;
import frc.robot.commands.RedBlueChaseAndBlink;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    public final AddressableLEDBuffer buffer;
    private final RainbowLEDPattern rainbowLEDPattern = new RainbowLEDPattern(30, LEDConstants.STRIP_LENGTH, LEDConstants.STRIP_LENGTH, 20);
    private final PurpleYellowChaseAndBlink purpleyellow = new PurpleYellowChaseAndBlink(3, 60, LEDConstants.STRIP_LENGTH, 0.2);
    private final RedBlueChaseAndBlink redblue = new RedBlueChaseAndBlink(3, 60, LEDConstants.STRIP_LENGTH, 20);
    private boolean isRunningRainbow = false;
    private boolean isRunningPY = true;
    private boolean isRunningRB = false;
    private int patternIndexer = 0;
    private double timeSinceSetSeconds = 0;

    public LEDs() {
        leds = new AddressableLED(LEDConstants.PORT);

        buffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    public void setColor(int r, int g, int b) {
        isRunningRainbow = false;
        isRunningPY = false;
        isRunningRB = false;
        timeSinceSetSeconds = 0;
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
        isRunningPY = false;
        isRunningRB = false;
        patternIndexer = 0;
    }

    public void setPY() {
        isRunningPY = true;
        isRunningRB = false;
        isRunningRainbow = false;
        patternIndexer = 0;
    }

    public void setRB() {
        isRunningRB = true;
        isRunningPY = false;
        isRunningRainbow = false;
        patternIndexer = 0;
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()) {
            if(timeSinceSetSeconds >= 30) {
                timeSinceSetSeconds = 0;
                isRunningPY = true;
            }
            timeSinceSetSeconds += 0.02;
        }

        if(RobotController.getBatteryVoltage() > 10) {
            if(!isRunningRainbow && !isRunningPY && !isRunningRB) {
                leds.setData(buffer);
            }
            else if(isRunningPY) {
                leds.setData(purpleyellow.step(0, buffer));
            }
            else if(isRunningRainbow) {
                leds.setData(rainbowLEDPattern.step(patternIndexer, buffer));
                patternIndexer++;
            }
            else if(isRunningRB) {
                leds.setData(redblue.step(0, buffer));
            }
        }
    }
}
