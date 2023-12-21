package frc.robot.util.controllers;

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
        spitButton(3),
        intakeButton(2),
        balanceButton(12),
        stowButton(4),
        doublePressDownButton(13);
    
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

    public Trigger spitButton() {
        return button(Button.spitButton.value);
    }
    
    public Trigger intakeButton() {
        return button(Button.intakeButton.value);
    }

    public Trigger balanceButton() {
        return button(Button.balanceButton.value);
    }

    public Trigger stowButton() {
        return button(Button.stowButton.value);
    }
}
