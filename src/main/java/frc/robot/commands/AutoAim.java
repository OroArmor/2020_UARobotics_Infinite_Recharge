package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AutoAim extends CommandBase {
	private final DriveSubsystem m_robotDrive;
	private NetworkTable m_table;

	/**
	 * Creates a new AutoAimCommand.
	 * 
	 * @param m_robotDrive The subsystem used by this command.
	 */
	public AutoAim(DriveSubsystem robotDrive) {
		m_robotDrive = robotDrive;
		m_table = NetworkTableInstance.getDefault().getTable("limelight");
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_robotDrive);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_table.getEntry("camMode").setNumber(0);
		m_table.getEntry("ledMode").setNumber(3);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Read Limelight Data
		double tv = m_table.getEntry("tv").getDouble(0);
		double tx = m_table.getEntry("tx").getDouble(0);

		if (tv == 1) {
			new TurnToRelativeAngle(tx, m_robotDrive);
		} else {
			m_robotDrive.arcadeDrive(0, 0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_robotDrive.arcadeDrive(0, 0);
		m_table.getEntry("ledMode").setNumber(1);
		m_table.getEntry("camMode").setNumber(1);
	}
}