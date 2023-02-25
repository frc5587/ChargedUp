package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandButtonBoard extends CommandGenericHID {
    public enum Button {
        upButton(9),
        middleButton(5),
        downButton(1),
        purpleButton(10),
        yellowButton(11),
        rightButton(6),
        centerButton(7),
        leftButton(8),
        retractIntakeButton(2),
        extendIntakeButton(3),
        resetButton(12),
        stowButton(4),
        doublePressDownButton(13);
    
        public final int value;
    
        Button(int value) {
          this.value = value;
        }
    }

    // public double timeSinceLastPress = 1;
    // public BooleanSupplier doublePressed = () -> downButton().getAsBoolean() && timeSinceLastPress < 1;

    public CommandButtonBoard(int port) {
        super(port);
    }

    public Trigger upButton() {
        return button(Button.upButton.value);
    }

    public Trigger middleButton() {
        return button(Button.middleButton.value);
    }

    public Trigger downButton() {
        return button(Button.downButton.value);
    }

    // public Trigger downButtonDoublePressed() {
    //     System.out.println("DOUBLE PRESSED!!!!");
    //     return new Trigger(doublePressed);
    // }

    public Trigger purpleButton() {
        return button(Button.purpleButton.value);
    }

    public Trigger yellowButton() {
        return button(Button.yellowButton.value);
    }

    public Trigger rightButton() {
        return button(Button.rightButton.value);
    }

    public Trigger centerButton() {
        return button(Button.centerButton.value);
    }

    public Trigger leftButton() {
        return button(Button.leftButton.value);
    }

    public Trigger retractButton() {
        
        return button(Button.retractIntakeButton.value);
    }
    
    public Trigger extendButton() {
        
        return button(Button.extendIntakeButton.value);
    }

    public Trigger resetButton() {
        return button(Button.resetButton.value);
    }

    public Trigger stowButton() {
        return button(Button.stowButton.value);
    }
}
