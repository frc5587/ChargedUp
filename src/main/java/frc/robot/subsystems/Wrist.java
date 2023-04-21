package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.titanlib.PivotingArmBase;

public class Wrist extends PivotingArmBase {
    private static CANSparkMax leftMotor = new CANSparkMax(40, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(41, MotorType.kBrushless);
    private final SparkMaxAbsoluteEncoder throughBore = rightMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private Arm arm;
    private boolean followingArm = true;
    private boolean raisedToSubstation = false;
    private boolean manualOverride = false;

    private static PivotingArmConstants constants = new PivotingArmConstants(
        WristConstants.GEARING,
        1,
        0,
        new double[]{},
        15*100, //TODO CHANGE ZERO OFFSET!!!!!!! this val is in encoder ticks
        WristConstants.ENCODER_CPR,
        WristConstants.PID_CONTROLLER,
        WristConstants.FF_CONTROLLER);

    public Wrist(Arm arm) {
        super("Wrist", constants, new MotorControllerGroup(leftMotor, rightMotor));
        this.arm = arm;
        configureMotors();
        resetEncoders();
        this.enable();

        rightMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(0.5-0.127+0.086-0.347+0.4-0.01);

        SmartDashboard.putBoolean("Wrist Brake Mode", true);
        getController().setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public double getEncoderPosition() {
        // return throughBore.getPosition()-0.5;
        return rightMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition()-0.5;
    }
    @Override
    public double getEncoderVelocity() {
        return rightMotor.getEncoder().getVelocity();
    }
    @Override
    public void setEncoderPosition(double position) {
        rightMotor.getEncoder().setPosition(position/360);        
    }
    
    @Override
    public void configureMotors() {
        leftMotor.setInverted(WristConstants.LEFT_INVERTED);
        rightMotor.setInverted(WristConstants.RIGHT_INVERTED);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setSmartCurrentLimit(20, 20);
        rightMotor.setSmartCurrentLimit(20, 20);
    }

    public void setFollowArm(boolean following) {
        this.followingArm = following;
        this.raisedToSubstation = !following;
    }

    public boolean isFollowingArm() {
        return followingArm;
    }

    public void setRaised(boolean raised) {
        this.raisedToSubstation = raised;
        this.followingArm = !raised;
        if(raised) setGoal(Units.degreesToRadians(-50));
    }

    public boolean isRaised() {
        return raisedToSubstation;
    }

    public void setManualOverride(boolean overridden) {
        this.manualOverride = overridden;
    }
    
    @Override
    public void periodic() {
        System.out.println(throughBore.getZeroOffset());
        if(arm.getController().getGoal().position == ArmConstants.SUB_SETPOINT && !isRaised()) {
            setRaised(true);
        }
        else if(arm.getController().getGoal().position != ArmConstants.SUB_SETPOINT) {
            setFollowArm(true);
        }

        if(isFollowingArm() && !manualOverride) {
            /** wrist visualizer https://www.desmos.com/calculator/9ievw4kltq */
            if(arm.getMeasurement() > Units.degreesToRadians(15) && arm.getMeasurement() < Units.degreesToRadians(75)) {
                setGoal((-arm.getMeasurement()));
            }
            else if(arm.getMeasurement() > Units.degreesToRadians(75)) {
                setGoal((-arm.getMeasurement()) + Units.degreesToRadians(15));
            }
            else {
                setGoal(Units.degreesToRadians(20));
            }
        }

        super.periodic();
    }
}
