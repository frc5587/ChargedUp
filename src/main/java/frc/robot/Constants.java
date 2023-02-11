package frc.robot;

import org.frc5587.lib.subsystems.PivotingArmBase.PivotingArmConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
}
