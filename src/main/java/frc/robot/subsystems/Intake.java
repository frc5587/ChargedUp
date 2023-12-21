package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SimpleMotorBase {
    private static final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);
    private static final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);

    private final ColorSensor colorSensor;

    public Intake(ColorSensor colorSensor) {
        super(new MotorControllerGroup(leftIntake, rightIntake), IntakeConstants.THROTTLE_FORWARD, IntakeConstants.THROTTLE_REVERSE);
        setName("Intake");
        this.colorSensor = colorSensor;

        configureMotors();
    }

    /**
     * Configures the motors, this includes inversions, current limits, and idle modes.
     */
    @Override
    public void configureMotors() {
        leftIntake.restoreFactoryDefaults();
        rightIntake.restoreFactoryDefaults();

        leftIntake.setInverted(IntakeConstants.LEFT_MOTOR_INVERTED);
        rightIntake.setInverted(IntakeConstants.RIGHT_MOTOR_INVERTED);

        leftIntake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
        rightIntake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);

        leftIntake.setIdleMode(IdleMode.kBrake);
        rightIntake.setIdleMode(IdleMode.kBrake);
    }

    public boolean hasElement() {
        return colorSensor.hasElementColor();
    }

    public void autoThrottle() {
        motors.set(IntakeConstants.THROTTLE_AUTO);
    }

    public void spitCube() {
        motors.set(-0.25);
    }

    public void spitCone() {
        motors.set(-0.25);
    }
    
    public void dropCone() {
        motors.set(-0.1);
    }

    public void shootCube() {
        motors.set(-5);
    }

    public void holdElement() {
        motors.set(0.1);
    }
    
    @Override
    public void periodic() {
        if(Robot.m_debugMode) {
            SmartDashboard.putBoolean("Has Game Piece", hasElement());
        }
    }
}