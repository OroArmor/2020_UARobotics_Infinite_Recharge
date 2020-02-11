package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Paths;
import java.io.IOException;

import frc.robot.Constants.DriveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase implements Loggable{
  // TODO Need to fill in which Motor Controllers are actually being used on the drive
	// The motors on the left and right side of the drivetrain
	@Log
	private final WPI_TalonSRX m_talonsrxleft = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
	private final WPI_VictorSPX m_talonsrxleft2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);
	@Log
	private final WPI_TalonSRX m_talonsrxright = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
  private final WPI_VictorSPX m_victorspxright = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);
  @Log
	private final WPI_TalonSRX m_talonsrxright2 = new WPI_TalonSRX(DriveConstants.kPigeonPort);

  // Pigeon is plugged into the second talon on the left side
  private final PigeonIMU m_pigeon = new PigeonIMU(m_talonsrxright2);
	
	// Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // PID Controller for Driving straight with Gyro
  private final PIDController m_gyropid = new PIDController(DriveConstants.kGyroPID, 0, 0);

  // Using onboard feedforward since it is more accurate than Talon Feedforward
  private final SimpleMotorFeedforward m_driveFeedforward =
      new SimpleMotorFeedforward(DriveConstants.kS,
                                 DriveConstants.kV,
                                 DriveConstants.kA);

  /** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;
	int _printCount = 0;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

			// Set followers
		m_talonsrxleft2.set(ControlMode.Follower, DriveConstants.kLeftMotor1Port);
		m_victorspxright.set(ControlMode.Follower, DriveConstants.kRightMotor1Port);

    /* Disable all motor controllers */
		m_talonsrxright.set(ControlMode.PercentOutput, 0);
		m_talonsrxleft.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		m_talonsrxright.configFactoryDefault();
		m_talonsrxleft.configFactoryDefault();
		m_pigeon.configFactoryDefault();
    
    // Set Ramping

    m_talonsrxleft.configClosedloopRamp(DriveConstants.kClosedRamp);
    m_talonsrxleft.configClosedloopRamp(DriveConstants.kOpenRamp);
    m_talonsrxright.configClosedloopRamp(DriveConstants.kClosedRamp);
    m_talonsrxright.configClosedloopRamp(DriveConstants.kOpenRamp);

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
		m_talonsrxright.configSelectedFeedbackCoefficient(	DriveConstants.kTurnTravelUnitsPerRotation / DriveConstants.kPigeonUnitsPerRotation,	// Coefficient
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
        final int closedLoopTimeMs = 1;
    m_talonsrxright.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
    m_talonsrxright.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.kTimeoutMs);

    /*
     * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
     * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
     * local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    m_talonsrxright.configAuxPIDPolarity(false, DriveConstants.kTimeoutMs);

    /* Initialize */
    _firstCall = true;
    _state = false;
    _printCount = 0;
    zeroHeading();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      m_talonsrxleft.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse,
      m_talonsrxright.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_talonsrxleft.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
        m_talonsrxright.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(final Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    fwd = Deadband(fwd);
    rot = Deadband(rot);
    m_talonsrxleft.set(ControlMode.PercentOutput, fwd, DemandType.ArbitraryFeedForward, +rot);
    m_talonsrxright.set(ControlMode.PercentOutput, fwd, DemandType.ArbitraryFeedForward, -rot);
  }

   /**
   * Drives the robot using tank controls.
   *
   * @param left the commanded left side drivetrain power
   * @param right the commanded right side drivetrain power
   */
  public void tankDrive(double left, double right) {
    left = Deadband(left);
    right = Deadband(right);
    m_talonsrxleft.set(ControlMode.PercentOutput, left);
    m_talonsrxright.set(ControlMode.PercentOutput, right);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_talonsrxleft.setSelectedSensorPosition(0);
    m_talonsrxright.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  @Log
  public double getAverageEncoderDistance() {
    return (m_talonsrxleft.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse
        + m_talonsrxright.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse) / 2.0;
  }

  /** Deadband 5 percent, used on the gamepad */
  double Deadband(final double value) {
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
    m_pigeon.setYaw(0, DriveConstants.kTimeoutMs);
    m_pigeon.setAccumZAngle(0, DriveConstants.kTimeoutMs);
    System.out.println("[Pigeon] All sensors are zeroed.\n");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  @Log
  public double getHeading() {
    final double[] ypr = new double[3];
		m_pigeon.getYawPitchRoll(ypr);
    return Math.IEEEremainder(ypr[0], 360);
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - stepsPerDecisecToMetersPerSec(m_talonsrxleft.getSelectedSensorVelocity())) / 20;
    var rightAccel = (rightVelocity - stepsPerDecisecToMetersPerSec(m_talonsrxright.getSelectedSensorVelocity())) / 20;
    
    var leftFeedForwardVolts = m_driveFeedforward.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = m_driveFeedforward.calculate(rightVelocity, rightAccel);

    m_talonsrxleft.set(
        ControlMode.Velocity, 
        metersPerSecToStepsPerDecisec(leftVelocity), 
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
    m_talonsrxright.set(
        ControlMode.Velocity,
        metersPerSecToStepsPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);
  }

    /**
   * Converts from encoder steps to meters.
   * 
   * @param steps encoder steps to convert
   * @return meters
   */
  public static double stepsToMeters(int steps) {
    return (DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.SENSOR_UNITS_PER_ROTATION) * steps;
  }

  /**
   * Converts from encoder units per 100 milliseconds to meters per second.
   * @param stepsPerDecisec steps per decisecond
   * @return meters per second
   */
  public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

  /**
   * Converts from meters to encoder units.
   * @param meters meters
   * @return encoder units
   */
  public static double metersToSteps(double meters) {
    return (meters / DriveConstants.WHEEL_CIRCUMFERENCE_METERS) * DriveConstants.SENSOR_UNITS_PER_ROTATION;
  }

  /**
   * Convers from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
  }

  // Drives straight specified distance 
  public void drivestraight() {
    
  }
}