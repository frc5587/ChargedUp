package frc.robot;

import org.frc5587.lib.pid.FPID;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.swervelib.util.COTSFalconSwerveConstants;
import frc.robot.util.swervelib.util.SwerveModuleConstants;
import org.frc5587.lib.subsystems.PivotingArmBase.PivotingArmConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveConstants {
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        public static final double WHEEL_BASE = Units.inchesToMeters(22);
        public static final double WHEEL_CIRCUMFERENCE_METERS = CHOSEN_MODULE.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERTED = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERTED = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERTED = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CONT_LIMIT = 25;
        public static final int ANGLE_PEAK_LIMIT = 40;
        public static final double ANGLE_PEAK_DURATION = 0.1;
        public static final boolean ANGLE_LIMIT_ENABLED = true;

        public static final int DRIVE_CONT_LIMIT = 35;
        public static final int DRIVE_PEAK_LIMIT = 60;
        public static final double DRIVE_PEAK_DURATION = 0.1;
        public static final boolean DRIVE_LIMIT_ENABLED = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final FPID ANGLE_FPID = new FPID(
                CHOSEN_MODULE.angleKF, CHOSEN_MODULE.angleKP, CHOSEN_MODULE.angleKI, CHOSEN_MODULE.angleKD);

        /* Drive Motor PID Values */
        public static final FPID DRIVE_FPID = new FPID(
                0.05, 0., 0., 0.); // TODO Characterize

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double DRIVE_KS = (0.32 / 12); // TODO Characterize
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; // TODO Confirm
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO Confirm

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_ID = 10;
            public static final int ANGLE_ID = 15;
            public static final int CANCODER_ID = 50;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(212.607);// TODO: tune all angle offsets
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int DRIVE_ID = 11;
            public static final int ANGLE_ID = 16;
            public static final int CANCODER_ID = 51;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(196.435);// TODO: tune all angle offsets
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_ID = 12;
            public static final int ANGLE_ID = 17;
            public static final int CANCODER_ID = 52;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(261.563);// TODO: tune all angle offsets
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int DRIVE_ID = 13;
            public static final int ANGLE_ID = 18;
            public static final int CANCODER_ID = 53;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(168.662);// TODO: tune all angle offsets
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants { //TODO Confirm
        public static final double MAX_SPEED_MPS = 3; //m/s
        public static final double MAX_ACCEL_MPS_2 = 3; // m/s^2
        public static final double MAX_ANGULAR_SPEED_R_S = Math.PI; // radians/s
        public static final double MAX_ANGULAR_ACCEL_R_S_2 = Math.PI; //radians/s^2
    
        public static final double KP_X_CONTROLLER = 1;
        public static final double KP_Y_CONTROLLER = 1;
        public static final double KP_THETA_CONTROLLER = 1;
    
        public static final TrapezoidProfile.Constraints K_PXY_CONSTRAINTS =
            new TrapezoidProfile.Constraints(MAX_SPEED_MPS, MAX_ACCEL_MPS_2);

        public static final TrapezoidProfile.Constraints K_THETA_CONSTRAINTS =
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_R_S, MAX_ANGULAR_ACCEL_R_S_2);

        public static TrajectoryConfig DEFAULT_TRAJECTORY_CONFIG = new TrajectoryConfig(MAX_SPEED_MPS, MAX_ACCEL_MPS_2);

        
        public static final ProfiledPIDController BOT_DRIVE_CONTROLLER =
        new ProfiledPIDController(
            2.5, 0.0, 0.0, K_PXY_CONSTRAINTS);
        public static final ProfiledPIDController BOT_ANGLE_CONTROLLER =
        new ProfiledPIDController(
            7.0, 0.0, 0.0, K_THETA_CONSTRAINTS);

        public static final class GridLocationGroup {
            public final Pose2d greaterPose, poseLeft, poseRight;
            public final Pose2d[] poseArray;

            public GridLocationGroup(Pose2d greaterPose, Alliance alliance) {
                this.greaterPose = greaterPose;
                if(alliance.equals(Alliance.Blue)) {
                    this.poseLeft = new Pose2d(greaterPose.getX(), greaterPose.getY()+.56, greaterPose.getRotation());
                    this.poseRight = new Pose2d(greaterPose.getX(), greaterPose.getY()-.56, greaterPose.getRotation());
                }
                else {
                    this.poseLeft = new Pose2d(greaterPose.getX(), greaterPose.getY()-.56, greaterPose.getRotation());
                    this.poseRight = new Pose2d(greaterPose.getX(), greaterPose.getY()+.56, greaterPose.getRotation());
                }
                this.poseArray = new Pose2d[]{this.poseLeft, this.greaterPose, this.poseRight};
            }

            public GridLocationGroup(Pose2d greaterPose, Pose2d poseLeft, Pose2d poseRight) {
                this.greaterPose = greaterPose;
                this.poseLeft = poseLeft;
                this.poseRight = poseRight;
                this.poseArray = new Pose2d[]{this.poseLeft, this.greaterPose, this.poseRight};
            }
        }

        public static final GridLocationGroup BLUE_LEFT = new GridLocationGroup(new Pose2d(2, 4.42, Rotation2d.fromDegrees(-90)), Alliance.Blue);
        public static final GridLocationGroup BLUE_CENTER = new GridLocationGroup(new Pose2d(2, 2.75, Rotation2d.fromDegrees(-90)), Alliance.Blue);
        public static final GridLocationGroup BLUE_RIGHT = new GridLocationGroup(new Pose2d(2, 1.06, Rotation2d.fromDegrees(-90)), Alliance.Blue);
        public static final GridLocationGroup RED_RIGHT = new GridLocationGroup(new Pose2d(14.525, 4.42, Rotation2d.fromDegrees(90)), Alliance.Red);
        public static final GridLocationGroup RED_CENTER = new GridLocationGroup(new Pose2d(14.525, 2.75, Rotation2d.fromDegrees(90)), Alliance.Red);
        public static final GridLocationGroup RED_LEFT = new GridLocationGroup(new Pose2d(14.525, 1.06, Rotation2d.fromDegrees(90)), Alliance.Red);

        public static final GridLocationGroup[] GRID_LOCATIONS = {
            BLUE_LEFT, BLUE_CENTER, BLUE_RIGHT, RED_RIGHT, RED_CENTER, RED_LEFT
        };
    }

    public static class LimelightConstants {
        public static final double MOUNT_ANGLE = 0;
        public static final double LENS_HEIGHT = Units.inchesToMeters(38.275);
        public static final double GOAL_HEIGHT = Units.inchesToMeters(14.25);
        public static final double DISTANCE_OFFSET = 0;
    }

    public static final class ArmConstants {
        public static final int LEADER_PORT = 20;
        public static final int FOLLOWER_PORT = 21;
        public static final double GEARING = 80; // TODO: Calculate
        public static final double VELOCITY_DENOMINATOR = 0.1;
        public static final double[] SOFT_LIMITS = {0, Units.degreesToRadians(100)};
        public static final int ZERO_OFFSET = 0; // TODO: Find
        public static final int ENCODER_CPR = 2048; // Change to 2048 if using Falcon
        public static final int[] SWITCH_PORTS = {0};
        public static final boolean[] SWITCHES_INVERTED = {false};
        public static final TrapezoidProfile.Constraints PID_CONSTRAINTS = new TrapezoidProfile.Constraints(0.8, 0.4); // TODO: Verify
        public static final double KP = 5.1051;//4.6126;
        public static final double KI = 0;
        public static final double KD = 0.73244;//0.5518;
        public static final double KS = 0.10322;
        public static final double KG = 0.4412;
        public static final double KV = 1.4181;
        public static final double KA = 0.025661;
        public static final ProfiledPIDController ARM_PID = new ProfiledPIDController(KP, KI, KD, PID_CONSTRAINTS);
        public static final ArmFeedforward ARM_FF = new ArmFeedforward(KS, KG, KV, KA);
        public static final double HIGH_SETPOINT = Units.degreesToRadians(80);
        public static final double MEDIUM_SETPOINT = Units.degreesToRadians(73);
        public static final double INTAKE_SETPOINT = Units.degreesToRadians(15);
        public static final double STOW_SETPOINT = Units.degreesToRadians(0);
        public static final double FF_ANGLE_OFFSET = Units.degreesToRadians(85);

        public static final PivotingArmConstants ARM_CONSTANTS = new PivotingArmConstants(
                GEARING, VELOCITY_DENOMINATOR, FF_ANGLE_OFFSET, SOFT_LIMITS, ZERO_OFFSET, ENCODER_CPR, SWITCH_PORTS, SWITCHES_INVERTED, ARM_PID, ARM_FF);
    }

    public static class IntakeConstants {
        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 1;
    }
    
    public static final class LEDConstants {
        public static final int PORT = 0; // TODO Check
        public static final int STRIP_LENGTH = 0; // TODO
    }
}
