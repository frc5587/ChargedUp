package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.titanlib.PivotingArmBase;
import frc.robot.Robot;

public class Wrist extends PivotingArmBase {
    private static CANSparkMax leftMotor = new CANSparkMax(40, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(41, MotorType.kBrushless);
    // private static DigitalInput limitSwitch = new DigitalInput(WristConstants.SWITCH_PORT);
    // private final SparkMaxAbsoluteEncoder throughBore; //= new DutyCycleEncoder(2); 
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

        // throughBore = rightMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.k, 0);

        // throughBore.setDutyCycleRange(1./1024., 1023./1024.);

        SmartDashboard.putBoolean("Wrist Brake Mode", true);
        getController().setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public double getEncoderPosition() {
        // return leftMotor.getEncoder().getPosition() * 42;
        // return throughBore.getPosition()-0.772;
        rightMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(0.5-.225);
        return rightMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition()-0.5;
    }
    @Override
    public double getEncoderVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }
    @Override
    public void setEncoderPosition(double position) {
        leftMotor.getEncoder().setPosition(position/360);        
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
    // /**
    //  * @param switchPort the port of the limit switch we want the value of
    //  * @return the limit switch's state, inverted if necessary.
    //  */
    // public boolean getLimitSwitchValue() {
    //     CANSparkMax limitSwitch;
    //     return ArmConstants.SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get();
    // }

    // /**
    //  * @param switchPort the port of the limit switch you want to get
    //  * @return the DigitalInput of the switch
    //  */
    // public DigitalInput getLimitSwitchObject() {
    //     return limitSwitch;
    // }

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

    public void setManualOverride(boolean overridden) {
        this.manualOverride = overridden;
        // this.followingArm = false;
        // this.raisedToSubstation = false;
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // double ff = ffController.calculate(setpoint.position+constants.offsetFromHorizontalRadians, setpoint.velocity);
        double ff = 0;
        if(arm.getMeasurement()>Units.degreesToRadians(75)) {
            ff = WristConstants.HIGH_FF.calculate(setpoint.position+constants.offsetFromHorizontalRadians, setpoint.velocity);
        }
        else {
            ff = ffController.calculate(setpoint.position+constants.offsetFromHorizontalRadians, setpoint.velocity);
        }
        //TODO: remove debug prints once we know this code works
        if(Robot.m_debugMode) {
            SmartDashboard.putNumber(subsystemName + " FF", ff);
            SmartDashboard.putNumber(subsystemName + " PID", output);
            SmartDashboard.putNumber(subsystemName + " Percent", motor.get());
            SmartDashboard.putNumber(subsystemName + " Setpoint", Units.radiansToDegrees(setpoint.position));
            SmartDashboard.putNumber(subsystemName + " Position", getAngleDegrees());
            SmartDashboard.putBoolean(subsystemName + " At Setpoint", pidController.atGoal());
        }
        
        /** if the driver has set output on, useOutput. */
        if(SmartDashboard.getBoolean(subsystemName + " Output On?", true)) {
            setVoltage(output + ff);
        }
        /** otherwise, set output to 0 */
        else {
            setVoltage(0);
        }
    }
    
    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Wrist Limit Switch", getLimitSwitchValue());
        SmartDashboard.putNumber("Wrist Position", getEncoderPosition());

        // if(SmartDashboard.getBoolean("Wrist Brake Mode", true)) {
        //     leftMotor.setIdleMode(IdleMode.kBrake);
        //     rightMotor.setIdleMode(IdleMode.kBrake);
        // } else {
        //     leftMotor.setIdleMode(IdleMode.kCoast);
        //     rightMotor.setIdleMode(IdleMode.kCoast);
        // }

        // if(getLimitSwitchValue()) {
        //     this.resetEncoders();
        //     this.setGoal(Units.degreesToRadians(0)); //TODO
        // }

        if(arm.getController().getGoal().position == ArmConstants.SUB_SETPOINT && !manualOverride) {
            setRaised(true);
        }
        
        else {
            setFollowArm(true);
        }

        if(isRaised() && !manualOverride) {
            setGoal(Units.degreesToRadians(-50));
        }

        if(isFollowingArm() && !manualOverride) {

        /** wrist visualizer https://www.desmos.com/calculator/9ievw4kltq */
            // else {
                if(arm.getMeasurement() > Units.degreesToRadians(15) && arm.getMeasurement() < Units.degreesToRadians(75)) {
                    setGoal((-arm.getMeasurement()));
                }
                else if(arm.getMeasurement() > Units.degreesToRadians(75)) {
                    setGoal((-arm.getMeasurement())+Units.degreesToRadians(15));
                }
                else {
                    // setGoal(-(arm.getMeasurement()*2) + Units.degreesToRadians(20));
                    setGoal(Units.degreesToRadians(25));
                }
            // }
        }

        // SmartDashboard.putBoolean("WRISTTHROUGHBORE", throughBore.isConnected());

        // if(!throughBore.isConnected()) {
        //     this.disable();
        //     this.stop();
        // }
        // else {
        //     this.enable();
        // }

        super.periodic();
    }
}
