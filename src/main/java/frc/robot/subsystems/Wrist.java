package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.WristConstants;

public class Wrist extends PivotingArmBase {
    private static CANSparkMax leftMotor = new CANSparkMax(40, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(41, MotorType.kBrushless);
    private Arm arm;
    private static PivotingArmConstants constants = new PivotingArmConstants(
        WristConstants.GEARING,
        1,
        0,
        new double[]{},
        0,
        WristConstants.ENCODER_CPR,
        WristConstants.PID_CONTROLLER,
        WristConstants.FF_CONTROLLER);

    public Wrist(Arm arm) {
        super("Intake", constants, new MotorControllerGroup(leftMotor, rightMotor));
        this.arm = arm;
        configureMotors();
        this.enable();
    }

    @Override
    public double getEncoderPosition() {
        return leftMotor.getEncoder().getPosition();
    }
    @Override
    public double getEncoderVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }
    @Override
    public void setEncoderPosition(double position) {
        leftMotor.getEncoder().setPosition(position);        
    }
    @Override
    public void configureMotors() {
        leftMotor.setInverted(WristConstants.LEFT_INVERTED);
        rightMotor.setInverted(WristConstants.RIGHT_INVERTED);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        super.periodic();

        /** visualizer for this math at https://www.desmos.com/calculator/enkypdni1s */
        if(arm.getMeasurement() > Units.degreesToRadians(20)) {
            setGoal(-arm.getMeasurement());
        }
        /** visualizer for this equation at https://www.desmos.com/calculator/bgfw72pyeq */
        else {
            setGoal(-(arm.getMeasurement()*4) + Units.degreesToRadians(60));
        }
    }
}
