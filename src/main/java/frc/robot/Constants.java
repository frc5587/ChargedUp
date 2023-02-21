package frc.robot;

import org.frc5587.lib.pid.FPID;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(18); // TODO Measure
        public static final double WHEEL_BASE = Units.inchesToMeters(18); // TODO Measure
        public static final double WHEEL_CIRCUMFERENCE_METERS = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERTED = chosenModule.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERTED = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERTED = chosenModule.canCoderInvert;

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
                chosenModule.angleKF, chosenModule.angleKP, chosenModule.angleKI, chosenModule.angleKD);

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
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.);// TODO: tune all angle offsets
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int DRIVE_ID = 11;
            public static final int ANGLE_ID = 16;
            public static final int CANCODER_ID = 51;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.);// TODO: tune all angle offsets
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_ID = 12;
            public static final int ANGLE_ID = 17;
            public static final int CANCODER_ID = 52;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.);// TODO: tune all angle offsets
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int DRIVE_ID = 13;
            public static final int ANGLE_ID = 18;
            public static final int CANCODER_ID = 53;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.);// TODO: tune all angle offsets
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
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints K_THETA_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_R_S, MAX_ANGULAR_ACCEL_R_S_2);
    }

    public static class LimelightConstants {
        public static final double MOUNT_ANGLE = 0;
        public static final double LENS_HEIGHT = Units.inchesToMeters(20.375);
        public static final double GOAL_HEIGHT = Units.inchesToMeters(14.25);
        public static final double DISTANCE_OFFSET = 0;
    }
    
    public static final class ArmConstants {
        public static final int motorPort = 15;
        public static final double gearing = 3; // TODO: Calculate
        public static final double[] softLimits = {0, Units.degreesToRadians(100)};
        public static final int zeroOffset = 0; // TODO: Find
        public static final int encoderCPR = 42; // Change to 2048 if using Falcon
        public static final int[] limitSwitchPorts = {0};
        public static final boolean[] switchInverted = {false};
        public static final TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(1, 0.2); // TODO: Verify
        public static final ProfiledPIDController armPIDController = new ProfiledPIDController(1, 0, 0, pidConstraints); // TODO: Characterize
        public static final ArmFeedforward armFF = new ArmFeedforward(0, 0, 0); // TODO: Characterize

        public static final PivotingArmConstants armConstants = new PivotingArmConstants(
                gearing, softLimits, zeroOffset, encoderCPR, limitSwitchPorts, switchInverted, armPIDController, armFF);
    }

    public static class IntakeConstants {
        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 1;
    }
}
