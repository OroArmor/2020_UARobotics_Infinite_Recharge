package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
	@Config
	private final WPI_VictorSPX m_IntakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeControllerPort);
	@Config
	private final DoubleSolenoid m_intakeSolenoid1 = new DoubleSolenoid(IntakeConstants.kSolenoid1ControllerPort,
			IntakeConstants.kSolenoid2ControllerPort);
	@Config
	private final DoubleSolenoid m_intakeSolenoid2 = new DoubleSolenoid(IntakeConstants.kSolenoid3ControllerPort,
			IntakeConstants.kSolenoid4ControllerPort);

	public IntakeSubsystem() {
		setOutput(0);
	}

	@Config
	public void setOutput(double speed) {
		m_IntakeMotor.set(speed);
	}

	@Config
	public void toggleIntakePosition(boolean enabled) {
		DoubleSolenoid.Value val = m_intakeSolenoid1.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;
		m_intakeSolenoid1.set(val);
		m_intakeSolenoid2.set(val);

	}

	@Config
	public void toggleIntakeWheels(boolean enabled) {
		if (m_IntakeMotor.get() > 0 || m_intakeSolenoid1.get() == DoubleSolenoid.Value.kForward) 
			m_IntakeMotor.set(0);
		else 
			m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
	}
}
