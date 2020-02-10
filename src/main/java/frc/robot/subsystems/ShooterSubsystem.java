package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterSubsystem extends PIDSubsystem implements Loggable{
  @Log
  private final WPI_VictorSPX m_shooterMotor = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort);
  private final WPI_VictorSPX m_shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort2);

  @Log
  private final WPI_VictorSPX m_feederMotor = new WPI_VictorSPX(ShooterConstants.kFeederMotorPort);

  @Log
  private final Encoder m_shooterEncoder =
      new Encoder(ShooterConstants.kEncoderPorts[0], ShooterConstants.kEncoderPorts[1],
                  ShooterConstants.kEncoderReversed);

  @Log
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.kSVolts,
                                 ShooterConstants.kVVoltSecondsPerRotation);

  // The shooter subsystem for the robot.
  public ShooterSubsystem() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPM);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    m_shooterMotor2.follow(m_shooterMotor);
    m_shooterMotor2.setInverted(InvertType.OpposeMaster);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_shooterEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void runFeeder() {
    m_feederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
  }
}