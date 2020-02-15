package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase implements Loggable{
    @Log
    //private final WPI_TalonSRX m_ClimbMotor = new WPI_TalonSRX(ClimbConstants.kClimbControllerPort);
    private final WPI_VictorSPX m_ClimbMotor = new WPI_VictorSPX(ClimbConstants.kClimbControllerPort);
    
    @Log
    //private final WPI_TalonSRX m_ClimbMotor2 = new WPI_TalonSRX(ClimbConstants.kClimbController2Port);
    private final WPI_VictorSPX m_ClimbMotor2 = new WPI_VictorSPX(ClimbConstants.kClimbController2Port);

    public ClimbSubsystem() {
        m_ClimbMotor2.setInverted(true);
        setOutput(0,0);
    }

    @Log
    public void setOutput(double motorPercent, double motorPercent2) {
        this.m_ClimbMotor.set(motorPercent);
        this.m_ClimbMotor2.set(motorPercent2);
    }
}