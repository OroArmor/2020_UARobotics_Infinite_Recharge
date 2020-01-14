package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // TODO Need to fill in which Motor Controllers are actually being used on the drive
  // The motors on the left and right side of the drivetrain
  private final TalonSRX m_talonsrxleft = new TalonSRX(DriveConstants.kLeftMotor1Port);
  private final TalonSRX m_talonsrxleft2 = new TalonSRX(DriveConstants.kLeftMotor2Port);
  private final TalonSRX m_talonsrxright = new TalonSRX(DriveConstants.kRightMotor1Port);
  private final VictorSPX m_victorspxright = new VictorSPX(DriveConstants.kRightMotor2Port);

  // Pigeon is plugged into the second talon on the left side
  private final PigeonIMU m_pigeon = new PigeonIMU(m_talonsrxleft2);
  
  /** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;
	int _printCount = 0;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    /* Disable all motor controllers */
		m_talonsrxright.set(ControlMode.PercentOutput, 0);
		m_talonsrxleft.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		m_talonsrxright.configFactoryDefault();
		m_talonsrxleft.configFactoryDefault();
		m_pigeon.configFactoryDefault();
		
		/* Set Neutral Mode */
		m_talonsrxleft.setNeutralMode(NeutralMode.Brake);
		m_talonsrxright.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the Pigeon IMU as a Remote Sensor for the right Talon */
		m_talonsrxright.configRemoteFeedbackFilter(m_pigeon.getDeviceID(),			// Device ID of Source
												RemoteSensorSource.Pigeon_Yaw,	// Remote Feedback Source
												DriveConstants.REMOTE_1,				// Remote number [0, 1]
												DriveConstants.kTimeoutMs);			// Configuration Timeout
		
		/* Configure the Remote Sensor to be the Selected Sensor of the right Talon */
		m_talonsrxright.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1, 	// Set remote sensor to be used directly
													DriveConstants.PID_TURN, 			// PID Slot for Source [0, 1]
													DriveConstants.kTimeoutMs);			// Configuration Timeout
		
		/* Scale the Selected Sensor using a coefficient (Values explained in Constants.java */
		m_talonsrxright.configSelectedFeedbackCoefficient(	Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation,	// Coefficient
                            DriveConstants.PID_TURN, 														// PID Slot of Source
														DriveConstants.kTimeoutMs);														// Configuration Timeout
		/* Configure output and sensor direction */
		m_talonsrxleft.setInverted(false);
		m_talonsrxleft.setSensorPhase(true);
		m_talonsrxright.setInverted(true);
		m_talonsrxright.setSensorPhase(true);
		
		/* Set status frame periods */
		m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, DriveConstants.kTimeoutMs);
		m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, DriveConstants.kTimeoutMs);
		m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, DriveConstants.kTimeoutMs);
		m_pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, DriveConstants.kTimeoutMs);
		
		/* Configure neutral deadband */
		m_talonsrxright.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);
		m_talonsrxleft.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);		

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		m_talonsrxleft.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
		m_talonsrxleft.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);
		m_talonsrxright.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
		m_talonsrxright.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);

		/* FPID Gains for turn servo */
		m_talonsrxright.config_kP(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kP, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kI(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kI, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kD(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kD, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kF(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kF, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_IntegralZone(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kIzone, DriveConstants.kTimeoutMs);
		m_talonsrxright.configClosedLoopPeakOutput(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kPeakOutput, DriveConstants.kTimeoutMs);
		m_talonsrxright.configAllowableClosedloopError(DriveConstants.kSlot_Turning, 0, DriveConstants.kTimeoutMs);	
		
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        int closedLoopTimeMs = 1;
        m_talonsrxright.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
        m_talonsrxright.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		m_talonsrxright.configAuxPIDPolarity(false, Constants.kTimeoutMs);

		/* Initialize */
		_firstCall = true;
		_state = false;
		_printCount = 0;
		zeroHeading();
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}

  /** Zero all sensors used. */
	void zeroHeading() {
		_pidgey.setYaw(0, Constants.kTimeoutMs);
		_pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Pigeon] All sensors are zeroed.\n");
	}
}