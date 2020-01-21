/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

// TODO Need to tweak almost all of these constants.  Also going to use preferences to control some of the more variable things.
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;
        public static final int kPigeonPort = 0;
    
        public static final int[] kLeftEncoderPorts = new int[]{1, 2};
        public static final int[] kRightEncoderPorts = new int[]{3, 4};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        public static final double kGyroPID = .01;
        /**
         * This is a property of the Pigeon IMU, and should not be changed.
         */
        public final static int kPigeonUnitsPerRotation = 8192;

        /**
         * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
         * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
         */
        public final static double kTurnTravelUnitsPerRotation = 3600;

        /**
         * Set to zero to skip waiting for confirmation.
         * Set to nonzero to wait and report to DS if action fails.
         */
        public final static int kTimeoutMs = 30;

        /**
         * Motor neutral dead-band, set to the minimum 0.1%.
         */
        public final static double kNeutralDeadband = 0.001;
        
        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
         * Not all set of Gains are used in this project and may be removed as desired.
         * 
         * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
        public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
        public final static Gains kGains_Turning = new Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );
        public final static Gains kGains_Velocit = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
        public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );
        
        /** ---- Flat constants, you should not need to change these ---- */
        /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
        public final static int REMOTE_0 = 0;
        public final static int REMOTE_1 = 1;
        /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
        public final static int PID_PRIMARY = 0;
        public final static int PID_TURN = 1;
        /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
        public final static int SLOT_0 = 0;
        public final static int SLOT_1 = 1;
        public final static int SLOT_2 = 2;
        public final static int SLOT_3 = 3;
        /* ---- Named slots, used to clarify code ---- */
        public final static int kSlot_Distanc = SLOT_0;
        public final static int kSlot_Turning = SLOT_1;
        public final static int kSlot_Velocit = SLOT_2;
        public final static int kSlot_MotProf = SLOT_3;
        
        // Motion Profiling Constants
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    }
    
    public static final class ShooterConstants {
        public static final int[] kEncoderPorts = new int[]{5, 6};
        public static final boolean kEncoderReversed = false;
        public static final int kEncoderCPR = 1024;
        public static final double kEncoderDistancePerPulse =
            // Distance units will be rotations
            1.0 / (double) kEncoderCPR;
    
        public static final int kShooterMotorPort = 5;
        public static final int kFeederMotorPort = 6;
    
        public static final double kShooterFreeRPS = 5300;
        public static final double kShooterTargetRPS = 4000;
        public static final double kShooterToleranceRPS = 50;
    
        // These are not real PID gains, and will have to be tuned for your specific robot.
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
    
        // On a real robot the feedforward constants should be empirically determined; these are
        // reasonable guesses.
        public static final double kSVolts = 0.05;
        public static final double kVVoltSecondsPerRotation =
            // Should have value 12V at free speed...
            12.0 / kShooterFreeRPS;
    
        public static final double kFeederSpeed = 0.5;
    }
    
    public static final class ControlPanelConstants {
        public static final int kSolenoidPort = 0;
        public static final int kSpinWheelPort = 0;
        public static final double kWheelSpeedFast = 0.6;
        public static final double kWheelSpeedSlow = 0.1;
        public static final double colorwheel_past = 2;
        public static final double colorwheel_slow = 0.2;
        public static final double colorwheel_fast = 0.3;
        public static final double colorwheel_ticks = 110;
    }

    public static final class AutoConstants {
        public static final double kAutoTimeoutSeconds = 12;
        public static final double kAutoShootTimeSeconds = 7;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class LEDConstants {
        public static final int kLEDPWMPort = 0;
        public static final int kBufferSize = 60;
    }
    
    public static final class AutoAimConstants {
        public static final double kP = 0.03;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;
        public static final int kOperatorControllerPort = 2;
    }

    public static final class IntakeConstants {
        public static final int kIntakeControllerPort = 1;
        public static final int kConveyor1ControllerPort = 2;
        public static final int kConveyor2ControllerPort = 3;
        public static final int kSolenoid1ControllerPort = 1;
        public static final int kSolenoid2ControllerPort = 2;
        public static final int kSolenoid3ControllerPort = 3;
        public static final int kSolenoid4ControllerPort = 4;
    }
}
