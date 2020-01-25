package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends PIDSubsystem implements Loggable{
  // TODO need to change the lines below to our actual motor controllers 
  @Log
  private final WPI_VictorSPX m_shooterMotor = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort);
  @Log
  private final WPI_VictorSPX m_feederMotor = new WPI_VictorSPX(ShooterConstants.kFeederMotorPort);
  
  /* private final Encoder m_shooterEncoder =
      new Encoder(ShooterConstants.kEncoderPorts[0], ShooterConstants.kEncoderPorts[1],
                  ShooterConstants.kEncoderReversed); */
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.kSVolts,
                                 ShooterConstants.kVVoltSecondsPerRotation);

  /**
   * The shooter subsystem for the robot.
   */
  public ShooterSubsystem() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    //m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return 0; //m_shooterEncoder.getRate();
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
  public void setRPM(double RPM) {
    this.setSetpoint(RPM);
  }
}