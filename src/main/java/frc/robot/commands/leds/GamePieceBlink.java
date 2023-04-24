package frc.robot.commands.leds;

import org.frc5587.lib.advanced.CustomLEDPattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.GamePiece;

public class GamePieceBlink implements CustomLEDPattern {
    private final Color ledColor;
    private double stepper = 0;

    public GamePieceBlink(GamePiece gamePiece) {
        if(gamePiece == GamePiece.CONE) {
            ledColor = Color.kYellow;
        }
        else {
            ledColor = Color.kPurple;
        }
    }

    @Override
    public AddressableLEDBuffer step(int stepNumber, AddressableLEDBuffer ledBuffer) {
        stepper = stepNumber * 0.1;
        if(((int) stepper)%2 == 0) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, ledColor);
            }
        }
        else {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
        return ledBuffer;
    }
}