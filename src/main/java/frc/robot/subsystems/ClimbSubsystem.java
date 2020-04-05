package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
	private int setpoint = 4200;

	public ClimbSubsystem() {
		// m_RightClimbMotor.setInverted(true);
		setOutput(0, 0);
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
		this.m_LeftClimbMotor.set(leftMotorPercent * climbInvert);
		this.m_RightClimbMotor.set(rightMotorPercent * climbInvert);

		// As soon as we start raising the hooks switch the camera so the hook view is
		// larger
		if (leftMotorPercent > .5 || rightMotorPercent > .5) {
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
		}
	}

	@Config
	public void setPosition(double position) {
		if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").getDouble(0) < 2) {
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
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
		if (enabled) {
			climbInvert = -1;
			m_LeftClimbMotor.configPeakOutputReverse(-1);
			m_RightClimbMotor.configPeakOutputReverse(-1);
		} else {
			climbInvert = 1;
			m_LeftClimbMotor.configPeakOutputReverse(0);
			m_RightClimbMotor.configPeakOutputReverse(0);
		}
	}

	@Config.ToggleButton
	public void nextClimbStage(boolean enabled) {
		climbStage = climbStage + 1;
		switch (climbStage) {
		case 1:
			setpoint = ClimbConstants.kFullUpEncoderCount;
			setPosition(ClimbConstants.kFullUpEncoderCount);
			break;
		case 2:
			setpoint = ClimbConstants.kOnBarEncoderCount;
			setPosition(ClimbConstants.kOnBarEncoderCount);
			break;
		case 3:
			setpoint = ClimbConstants.kHangingEncoderCount;
			setPosition(ClimbConstants.kHangingEncoderCount);
			break;
		default:
		}
	}

	// Determines if the talon is at the desired position
	@Log
	public boolean atposition() {
		return inRange(m_LeftClimbMotor.getSelectedSensorPosition(), setpoint)
				&& inRange(m_RightClimbMotor.getSelectedSensorPosition(), setpoint)
				&& m_RightClimbMotor.getSelectedSensorPosition() > 100;
	}

	public boolean inRange(double position, double setpoint) {
		if (position > setpoint + ClimbConstants.kErrorTolerance) {
			return false;
		} else if (position < setpoint - ClimbConstants.kErrorTolerance) {
			return false;
		} else {
			return true;
		}
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
		return setpoint;
	}

	public void setSetpoint(int setpoint) {
		this.setpoint = setpoint;
	}
}