package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandButtonBoard extends CommandGenericHID {
    public enum Button {
        upButton(0),
        middleButton(4),
        downButton(8),
        purpleButton(9),
        yellowButton(10),
        rightButton(5),
        centerButton(6),
        leftButton(7),
        retractIntakeButton(1),
        extendIntakeButton(2),
        resetButton(11),
        stowButton(3);
    
        public final int value;
    
        Button(int value) {
          this.value = value;
        }
    }

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
