package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.Constants.AutoAimConstants;

public class AutoAim extends CommandBase implements Loggable{
  private final DriveSubsystem m_robotDrive;
  private final ShooterSubsystem m_shooter;
  private final PIDController pid = new PIDController(AutoAimConstants.kP, 0, 0);
  /**
   * Creates a new AutoAimCommand.
   * @param m_shooter
   * @param m_robotDrive The subsystem used by this command.
   */
  public AutoAim(DriveSubsystem robotDrive, ShooterSubsystem shooter) {
    m_robotDrive = robotDrive;
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setSetpoint(4500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read Limelight Data
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if (tv == 1)
    {
      m_robotDrive.arcadeDrive(0,pid.calculate(tx,0));
    }
    else
    {
      m_robotDrive.arcadeDrive(0,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.arcadeDrive(0,0);
  }
}