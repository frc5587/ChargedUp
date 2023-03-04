package frc.robot.commands;

import org.frc5587.lib.advanced.CustomLEDPattern;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class PurpleYellowChaseAndBlink implements CustomLEDPattern {
    private static final double stepTime = 0.02;
    private double idx = 0;
    private int blinkSpeed, chaseSpeed, length;
    private int[] redLEDs, blueLEDs, yellow, purple;

    /**
     * @param blinkSpeed how fast the pattern blinks, in blinks per second
     * @param chaseSpeed how fast the pattern chases, in pixels per second
     * @param length length of LED strip, in pixels
     * @param brightnessPercent from 0 to 1, the brightness of the LED strip
     */
    public PurpleYellowChaseAndBlink(int blinkSpeed, int chaseSpeed, int length, double brightnessPercent) {
        this.blinkSpeed = blinkSpeed;
        this.chaseSpeed = chaseSpeed;
        this.length = length;

        int[] boundYellow = {(int) (255 * brightnessPercent), (int) (255 * brightnessPercent), 0};
        int[] boundPurple = {(int) (178 * brightnessPercent), (int) (37 * brightnessPercent), (int) (188 * brightnessPercent)};
        yellow = boundYellow;
        purple = boundPurple;
    }

    @Override
    public AddressableLEDBuffer step(int stepNumber, AddressableLEDBuffer ledBuffer) {
        double usedSpeed;

        if(idx > length + 7) {
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
                ledBuffer.setRGB(redLEDs[i], purple[0], purple[1], purple[2]);
            }
            else {
                ledBuffer.setRGB(redLEDs[i], yellow[0], yellow[1], yellow[2]);
            }
        }
        for (int i = 0; i < blueLEDs.length; i++) {
            if(((int) idx % 2 == 0) || ((int) idx < length)) {
                ledBuffer.setRGB(blueLEDs[i], yellow[0], yellow[1], yellow[2]);
            }
            else {
                ledBuffer.setRGB(blueLEDs[i], purple[0], purple[1], purple[2]);
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
            if(((i <= (60))) || ((i <= (length*0.75)-14) && (i > (length / 2)-5))) {
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
