package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterSubsystem extends PIDSubsystem implements Loggable{
  @Log
  private final WPI_VictorSPX m_shooterMotor = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort);

  private final WPI_VictorSPX m_shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort2);

  private final Encoder m_shooterEncoder =
      new Encoder(ShooterConstants.kEncoderPorts[0], ShooterConstants.kEncoderPorts[1],
                  ShooterConstants.kEncoderReversed, CounterBase.EncodingType.k1X);

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.kSVolts,
                                 ShooterConstants.kVVoltSecondsPerRotation,
                                 ShooterConstants.kA);

  @Config
  private final PIDController shooterPID;

  // The shooter subsystem for the robot.
  public ShooterSubsystem() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    shooterPID = getController();
    shooterPID.setTolerance(ShooterConstants.kShooterToleranceRPM);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    m_shooterEncoder.setSamplesToAverage(50);
    m_shooterMotor2.follow(m_shooterMotor);
    m_shooterMotor.setInverted(true);
    m_shooterMotor2.setInverted(InvertType.OpposeMaster);
  }

  @Override
  @Log
  public void useOutput(double output, double setpoint) {
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("setpoint", m_shooterFeedforward.calculate(setpoint));
    m_shooterMotor.setVoltage(MathUtil.clamp((output / 60) + (m_shooterFeedforward.calculate(setpoint) / 60), 0, 14));
  }

  @Log
  @Log(tabName = "Dashboard")
  @Override
  public double getMeasurement() {
    return m_shooterEncoder.getRate();
  }

  @Log
  @Log(tabName = "Dashboard")
  public boolean atSetpoint() {
    return shooterPID.atSetpoint() && shooterPID.getSetpoint() > 0;
  }

  @Log
  public double getSetpoint() {
    return shooterPID.getSetpoint();
  }
}