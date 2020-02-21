package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.Constants.AutoAimConstants;

public class TrenchAuto extends CommandBase implements Loggable{
  private final DriveSubsystem m_robotDrive;
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  /**
   * Creates a new TrenchAuto.
   * @param shooter
   * @param robotDrive The subsystem used by this command.
   * @param intake
   */
  public AutoAim(ShooterSubsystem shooter, DriveSubsystem robotDrive, IntakeSubsystem intake) {
    m_shooter = shooter;
    m_robotDrive = robotDrive;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_robotDrive, m_intake);
  }
}