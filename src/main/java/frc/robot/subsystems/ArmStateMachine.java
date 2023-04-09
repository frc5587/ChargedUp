package frc.robot.subsystems;

public class ArmStateMachine {
    public double positionRadians;
    public double setpointRadians;
    public double velocityRPS;
    public double velocitySetpointRPS;

    public ArmStateMachine(double positionRadians, double setpointRadians, double velocityRPS, double velocitySetpointRPS) {
        this.positionRadians = positionRadians;
        this.setpointRadians = setpointRadians;
        this.velocityRPS = velocityRPS;
        this.velocitySetpointRPS = velocitySetpointRPS;
    }

    public ArmStateMachine(double positionRadians, double setpointRadians) {
        this(positionRadians, setpointRadians, 0, 0);
    }

    public ArmStateMachine() {
        this(0, 0, 0, 0);
    }

    public void setState(double positionRadians, double setpointRadians, double velocityRPS, double velocitySetpointRPS) {
        this.positionRadians = positionRadians;
        this.setpointRadians = setpointRadians;
        this.velocityRPS = velocityRPS;
        this.velocitySetpointRPS = velocitySetpointRPS;
    }

    public void setState(double positionRadians, double setpointRadians) {
        setState(positionRadians, setpointRadians, 0, 0);
    }

    public void resetState() {
        setState(0, 0, 0, 0);
    }

    public enum Input {
        RAISE, LOWER, DONE, INTERRUPT, STOW;
    }
}
