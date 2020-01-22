package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ClimbSubsystem extends SubsystemBase implements Loggable{
    @Log
    private final WPI_TalonSRX m_ClimbMotor = new WPI_TalonSRX(ClimbConstants.kClimbControllerPort);
    
    public ClimbSubsystem() {
        setOutput(0);
    }

    @Log
    public void setOutput(double motorPercent) {
        this.m_ClimbMotor.set(motorPercent)
    }
}