package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ClimbSubsystem extends SubsystemBase implements Loggable {
	// private final WPI_TalonSRX m_ClimbMotor = new
	// WPI_TalonSRX(ClimbConstants.kClimbControllerPort);
	@Config(name = "ClimbMotorLeft")
	private final WPI_TalonSRX m_LeftClimbMotor = new WPI_TalonSRX(ClimbConstants.kClimbLeftControllerPort);

	// private final WPI_TalonSRX m_ClimbMotor2 = new
	// WPI_TalonSRX(ClimbConstants.kClimbController2Port);
	@Config(name = "ClimbMotorRight")
	private final WPI_TalonSRX m_RightClimbMotor = new WPI_TalonSRX(ClimbConstants.kClimbRightControllerPort);

	@Log
	private int climbInvert = 1;

	@Log
	private int climbStage = 0;

	@Log
	private int setPoint = 4200;

	//limelight entry
	private NetworkTableEntry limelightEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream");

	public ClimbSubsystem() {
		// m_RightClimbMotor.setInverted(true);
		setOutput(0, 0);
		configure();
	}

	private void configure(){
		m_LeftClimbMotor.setInverted(true);
		m_LeftClimbMotor.setSensorPhase(true);
		m_RightClimbMotor.setInverted(false);
		m_RightClimbMotor.setSensorPhase(true);
		m_LeftClimbMotor.config_kP(0, ClimbConstants.kClimbP);
		m_RightClimbMotor.config_kP(0, ClimbConstants.kClimbP);
		m_LeftClimbMotor.configPeakOutputReverse(0);
		m_RightClimbMotor.configPeakOutputReverse(0);
	}

	@Config
	public void numOfRotations(double rotations) {
		double targetposition = rotations * ClimbConstants.kEncoderCPR;
		m_LeftClimbMotor.set(ControlMode.Position, targetposition);
		m_RightClimbMotor.set(ControlMode.Position, targetposition);
	}

	/*
	 * @Config public void setP(double p){ m_LeftClimbMotor.config_kP(0, p); }
	 */
	public void setOutput(double leftMotorPercent, double rightMotorPercent) {
		m_LeftClimbMotor.set(leftMotorPercent * climbInvert);
		m_RightClimbMotor.set(rightMotorPercent * climbInvert);

		// As soon as we start raising the hooks switch the camera so the hook view is
		// larger
		if (leftMotorPercent > .5 || rightMotorPercent > .5) {
			limelightEntry.setNumber(2);
		}
	}

	@Config
	public void setPosition(double position) {
		if (limelightEntry.getDouble(0) < 2) {
			limelightEntry.setNumber(2);
		}
		m_RightClimbMotor.set(ControlMode.Position, position);
		m_LeftClimbMotor.set(ControlMode.Position, position);
	}

	@Log
	public double getRightPosition() {
		return m_RightClimbMotor.getSelectedSensorPosition();
	}

	@Log
	public double getLeftPosition() {
		return m_LeftClimbMotor.getSelectedSensorPosition();
	}

	@Config.ToggleButton
	public void resetEnc(boolean enabled) {
		m_LeftClimbMotor.setSelectedSensorPosition(0);
		m_RightClimbMotor.setSelectedSensorPosition(0);
	}

	@Config.ToggleButton
	public void invertClimber(boolean enabled) {
		climbInvert = enabled ? -1 : 1;
		m_LeftClimbMotor.configPeakOutputReverse(enabled ? -1 : 0);
		m_RightClimbMotor.configPeakOutputReverse(enabled ? -1 : 0);
	}

	@Config.ToggleButton
	public void nextClimbStage(boolean enabled) {
		switch (climbStage++) {
		case 1:
			setPoint = ClimbConstants.kFullUpEncoderCount;
			break;
		case 2:
			setPoint = ClimbConstants.kOnBarEncoderCount;
			break;
		case 3:
			setPoint = ClimbConstants.kHangingEncoderCount;
			break;
		default:
			return;
		}
		setPosition(setPoint);
	}

	// Determines if the talon is at the desired position
	@Log
	public boolean atPosition() {
		return inRange(m_LeftClimbMotor.getSelectedSensorPosition(), setPoint)
				&& inRange(m_RightClimbMotor.getSelectedSensorPosition(), setPoint)
				&& m_RightClimbMotor.getSelectedSensorPosition() > 100;
	}

	public boolean inRange(double position, double setpoint) {
		return Math.abs(position-setpoint) > ClimbConstants.kErrorTolerance;
	}

	public int getClimbInvert() {
		return climbInvert;
	}

	public void setClimbInvert(int climbInvert) {
		this.climbInvert = climbInvert;
	}

	public int getClimbStage() {
		return climbStage;
	}

	public void setClimbStage(int climbStage) {
		this.climbStage = climbStage;
	}

	public int getSetpoint() {
		return setPoint;
	}

	public void setSetpoint(int setpoint) {
		this.setPoint = setpoint;
	}
}