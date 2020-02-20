package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.IntakeConstants;

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
        m_ConveyorMotor2.setInverted(true);
        m_ConveyorMotor2.set(ControlMode.Follower, IntakeConstants.kConveyor1ControllerPort);
        setOutput(0);
    }

    @Log
    public void setOutput(double speed) {
        this.m_IntakeMotor.set(speed);
        this.m_ConveyorMotor1.set(speed);
        this.m_ConveyorMotor2.set(speed);
    }

    @Log
    public void toggleIntakePosition() {
        if(this.m_intakeSolenoid1.get() == DoubleSolenoid.Value.kForward) {
            this.m_intakeSolenoid1.set(DoubleSolenoid.Value.kReverse);
            this.m_intakeSolenoid2.set(DoubleSolenoid.Value.kReverse);
        }
        else{
            this.m_intakeSolenoid1.set(DoubleSolenoid.Value.kForward);
            this.m_intakeSolenoid2.set(DoubleSolenoid.Value.kForward);
        }
        
    }

    @Log
    public void toggleIntakeWheels() {
        if(this.m_IntakeMotor.get() > 0) {
            this.m_IntakeMotor.set(0);
        }
        else{
            this.m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
        }
    }

    @Log
    public void toggleConveyor() {
        if(this.m_ConveyorMotor1.get() > 0) {
            this.m_ConveyorMotor1.set(0);
        }
        else{
            this.m_ConveyorMotor1.set(IntakeConstants.kConveyorMotorSpeed);
        }
        
    }
}

