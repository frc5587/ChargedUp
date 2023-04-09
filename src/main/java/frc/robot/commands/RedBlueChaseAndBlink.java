package frc.robot.commands;

import org.frc5587.lib.advanced.CustomLEDPattern;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RedBlueChaseAndBlink implements CustomLEDPattern {
    private static final double stepTime = 0.02;
    private double idx = 0;
    private int blinkSpeed, chaseSpeed, brightness, length;
    private int[] redLEDs, blueLEDs;

    /**
     * @param blinkSpeed how fast the pattern blinks, in blinks per second
     * @param chaseSpeed how fast the pattern chases, in pixels per second
     * @param length length of LED strip, in pixels
     * @param brightnessPercent from 0 to 255, the brightness of the LED strip
     */
    public RedBlueChaseAndBlink(int blinkSpeed, int chaseSpeed, int length, int brightness) {
        this.blinkSpeed = blinkSpeed;
        this.chaseSpeed = chaseSpeed;
        this.brightness = brightness;
        this.length = length;
    }

    @Override
    public AddressableLEDBuffer step(int stepNumber, AddressableLEDBuffer ledBuffer) {
        double usedSpeed;

        if(idx > length) {
            idx = 0;
        }
        
        if(idx >= length) {
            updateBlinks();
            usedSpeed = blinkSpeed;
        }
        else {
            updateChase();
            usedSpeed = chaseSpeed;
        }

        for (int i = 0; i < redLEDs.length; i++) {
            if(((int) idx % 2 == 0) || ((int) idx < length)) {
                ledBuffer.setRGB(redLEDs[i], brightness, 0, 0);
            }
            else {
                ledBuffer.setRGB(redLEDs[i], 0, 0, brightness);
            }
        }
        for (int i = 0; i < blueLEDs.length; i++) {
            if(((int) idx % 2 == 0) || ((int) idx < length)) {
                ledBuffer.setRGB(blueLEDs[i], 0, 0, brightness);
            }
            else {
                ledBuffer.setRGB(blueLEDs[i], brightness, 0, 0);
            }
        }
        
        idx += stepTime * usedSpeed;

        return ledBuffer;
    }

    public void updateBlinks() {
        redLEDs = new int[length];
        blueLEDs = new int[length];
        int redArrSetterIndex = 0;
        int blueArrSetterIndex = 0;
        for (int i = 0; i < length; i++) {
            if(((i <= (60))) || ((i <= (length*0.75)) && (i > (length / 2)-5)) || i == 0) {
                redLEDs[redArrSetterIndex] = i;
                redArrSetterIndex++;
            }
            else {
                blueLEDs[blueArrSetterIndex] = i;
                blueArrSetterIndex++;
            }
        }
    }

    public void updateChase() {
        redLEDs = new int[length/2+1];
        blueLEDs = new int[length/2+1];
        int groupLength = length / 2 + (int) idx;
        int redArrSetterIndex = 0;
        int blueArrSetterIndex = 0;
        for (int i = 0; i < length; i++) {
            if((i > idx && i < groupLength) || (i < groupLength - length)) {
                redLEDs[redArrSetterIndex] = i;
                redArrSetterIndex++;
            }
            else {
                blueLEDs[blueArrSetterIndex] = i;
                blueArrSetterIndex++;
            }
        }
    }
}
