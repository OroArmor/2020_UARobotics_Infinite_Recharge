package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import io.github.oblarg.oblog.annotations.Log;

public class TimedAutoAim extends CommandBase {
	private final DriveSubsystem m_robotDrive;

	private final Timer m_timer = new Timer();
	private final double timeout;

	public TimedAutoAim(double timeoutIn, DriveSubsystem drivetrain) {
		m_robotDrive = drivetrain;
		timeout = timeoutIn;

		m_timer.reset();
		m_timer.start();

		addRequirements(m_robotDrive);
	}

	@Log
	@Override
	public boolean isFinished() {
		// End when the controller is at the reference.
		return m_timer.get() > timeout;
	}

	@Override
	public void execute() {
		new AutoAim(m_robotDrive);
	}
}