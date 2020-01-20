package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class IntakeSubsystem extends SubsystemBase implements Loggable{
    @Log
    private final WPI_VictorSPX m_IntakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeControllerPort);
    @Log
    private final WPI_VictorSPX m_ConveyorMotor1 = new WPI_VictorSPX(IntakeConstants.kConveyor1ControllerPort);
    @Log
    private final WPI_VictorSPX m_ConveyorMotor2 = new WPI_VictorSPX(IntakeConstants.kConveyor2ControllerPort);
    @Log
    private final DoubleSolenoid m_intakeSolenoid1 = new DoubleSolenoid(IntakeConstants.kSolenoid1ControllerPort, IntakeConstants.kSolenoid2ControllerPort);
    @Log
    private final DoubleSolenoid m_intakeSolenoid2 = new DoubleSolenoid(IntakeConstants.kSolenoid3ControllerPort, IntakeConstants.kSolenoid4ControllerPort);

    public IntakeSubsystem() {
        setSetpoint(0);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    }

    public void setOutput(double speed) {
        @Log
        m_IntakeMotor.set(ControlMode.PercentOutput, speed);
        @Log
        m_ConveyorMotor1.set(ControlMode.PercentOutput, speed);
        @Log
        m_ConveyorMotor2.set(ControlMode.PercentOutput, speed);
    }

    public void setIntakePosition(int extend) {
        @Log
        m_intakeSolenoid1.set(DoubleSolenoid.Value.kForward);
        @Log
        m_intakeSolenoid2.set(extend);
    }
}

