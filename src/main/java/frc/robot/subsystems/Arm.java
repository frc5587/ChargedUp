package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;

import java.util.function.Supplier;

// import org.frc5587.lib.subsystems.PivotingArmBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends PivotingArmBase {
    private static WPI_TalonFX leader = new WPI_TalonFX(ArmConstants.LEADER_PORT);
    private static WPI_TalonFX follower = new WPI_TalonFX(ArmConstants.FOLLOWER_PORT);
    private static MotorControllerGroup group = new MotorControllerGroup(leader, follower);
    private static DigitalInput limitSwitch = new DigitalInput(ArmConstants.SWITCH_PORT);
    private final ColorSensor colorSensor;
    private final Supplier<Pose2d> poseSupplier;
    private boolean shouldLower, shouldLowerOverride = false;

    public Arm(ColorSensor colorSensor, Supplier<Pose2d> poseSupplier) {
        super(ArmConstants.ARM_CONSTANTS, group);
        this.colorSensor = colorSensor;
        this.poseSupplier = poseSupplier;
        configureMotors();
        this.enable();
    }

    @Override
    public double getEncoderPosition() {
        return leader.getSelectedSensorPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return leader.getSelectedSensorVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        leader.setSelectedSensorPosition(position);
    }

    @Override
    public void configureMotors() {
        leader.configFactoryDefault();
        follower.configFactoryDefault();
        leader.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);
        leader.setInverted(true);
        follower.setInverted(false);
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
    
    public void highSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.HIGH_SETPOINT);
    }

    public void middleSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.MEDIUM_SETPOINT);
    }

    public void lowSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.INTAKE_SETPOINT);
    }

    public void stow() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.STOW_SETPOINT);
    }

    public void liftAwayFromGrid() {
        this.setGoal(pidController.getGoal().position+Units.degreesToRadians(5));
    }
    
    public void lowerFromGrid() {
        this.setGoal(pidController.getGoal().position-Units.degreesToRadians(5));
    }

    public void setShouldLower(boolean shouldLower) {
        if(shouldLowerOverride) {
            shouldLower = false;
        }
        this.shouldLower = shouldLower;
    }

    public boolean isWithinLoweringArea(Pose2d pose) {
        boolean withinX = pose.getX() > AutoConstants.POSE_WINDOW[0].getX() && pose.getX() < AutoConstants.POSE_WINDOW[1].getX();
        boolean withinY = pose.getY() > AutoConstants.POSE_WINDOW[0].getY() && pose.getY() < AutoConstants.POSE_WINDOW[1].getY();
        return withinX && withinY;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("Limit Switch", getLimitSwitchValue());
        if(getLimitSwitchValue()) {
            this.resetEncoders();
        }

        if(isWithinLoweringArea(poseSupplier.get()) 
                && (pidController.getSetpoint().position > ArmConstants.DRIVE_SETPOINT 
                || (colorSensor.hasElement() && pidController.getSetpoint().position < ArmConstants.DRIVE_SETPOINT))) {
            setShouldLower(true);
        }
        else {
            setShouldLower(false);
            shouldLowerOverride = false;
        }
        if(shouldLower) {
            setGoal(ArmConstants.DRIVE_SETPOINT);
        }
    }
}
