package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;

public class Wrist extends PivotingArmBase {
    private static CANSparkMax leftMotor = new CANSparkMax(40, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(41, MotorType.kBrushless);
    private static DigitalInput limitSwitch = new DigitalInput(WristConstants.SWITCH_PORT);
    private Arm arm;
    private boolean followingArm = true;
    private boolean raisedToSubstation = false;

    private static PivotingArmConstants constants = new PivotingArmConstants(
        WristConstants.GEARING,
        1,
        0,
        new double[]{},
        0, //TODO CHANGE ZERO OFFSET!!!!!!! this val is in encoder ticks
        WristConstants.ENCODER_CPR,
        WristConstants.PID_CONTROLLER,
        WristConstants.FF_CONTROLLER);

    public Wrist(Arm arm) {
        super("Intake", constants, new MotorControllerGroup(leftMotor, rightMotor));
        this.arm = arm;
        configureMotors();
        setEncoderPosition(0);
        this.enable();
    }

    @Override
    public double getEncoderPosition() {
        return leftMotor.getEncoder().getPosition() * 42;
    }
    @Override
    public double getEncoderVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }
    @Override
    public void setEncoderPosition(double position) {
        leftMotor.getEncoder().setPosition(position * 42);        
    }
    @Override
    public void configureMotors() {
        leftMotor.setInverted(WristConstants.LEFT_INVERTED);
        rightMotor.setInverted(WristConstants.RIGHT_INVERTED);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }
    /**
     * @param switchPort the port of the limit switch we want the value of
     * @return the limit switch's state, inverted if necessary.
     */
    public boolean getLimitSwitchValue() {
        return ArmConstants.SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get();
    }

    /**
     * @param switchPort the port of the limit switch you want to get
     * @return the DigitalInput of the switch
     */
    public DigitalInput getLimitSwitchObject() {
        return limitSwitch;
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
    }

    public boolean isRaised() {
        return raisedToSubstation;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wrist Limit Switch", getLimitSwitchValue());

        if(getLimitSwitchValue()) {
            this.resetEncoders();
            this.setGoal(Units.degreesToRadians(0)); //TODO
        }

        if(arm.getController().getGoal().position == ArmConstants.SUB_SETPOINT) {
            setRaised(true);
        }
        
        else {
            setFollowArm(true);
        }

        if(raisedToSubstation) {
            setGoal(Units.degreesToRadians(-39));
        }

        else {
            /** visualizer for this math at https://www.desmos.com/calculator/enkypdni1s */
            if(arm.getMeasurement() > Units.degreesToRadians(20)) {
                setGoal(-arm.getMeasurement());
            }
            /** visualizer for this equation at https://www.desmos.com/calculator/ugfgabk2cz */
            else {
                setGoal(-(arm.getMeasurement()*2) + Units.degreesToRadians(20));
            }
        }

        super.periodic();
    }
}