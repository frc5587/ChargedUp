package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SimpleMotorBase {
    private static final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);
    private static final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftIntake.getEncoder();
    private final RelativeEncoder rightEncoder = rightIntake.getEncoder();

    private final ColorSensor colorSensor = new ColorSensor(I2C.Port.kOnboard);

    public Intake() {
        super(new MotorControllerGroup(leftIntake, rightIntake), IntakeConstants.THROTTLE_FORWARD, IntakeConstants.THROTTLE_REVERSE);

        setName("Intake");

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

    private double leftVelocity() {
        return leftEncoder.getVelocity();
    }

    private double rightVelocity() {
        return rightEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        if (colorSensor.hasCone()) {
            SmartDashboard.putBoolean("Game Piece", true); // TODO set true color to yellow
        } else if (colorSensor.hasCube()) {
            SmartDashboard.putBoolean("Game Piece", false); // TODO Set false color to purple
        }
        
        SmartDashboard.putBoolean("Has Game Piece", colorSensor.hasCone() || colorSensor.hasCube());
        
        SmartDashboard.putNumber("Left Velocity", leftVelocity());
        SmartDashboard.putNumber("Right Velocity", rightVelocity());
    }
}