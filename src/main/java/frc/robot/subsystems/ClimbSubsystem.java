package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;

public class ClimbSubsystem extends SubsystemBase implements Loggable{
    @Log
    //private final WPI_TalonSRX m_ClimbMotor = new WPI_TalonSRX(ClimbConstants.kClimbControllerPort);
    private final WPI_VictorSPX m_ClimbMotor = new WPI_VictorSPX(ClimbConstants.kClimbControllerPort);
    
    @Log
    //private final WPI_TalonSRX m_ClimbMotor2 = new WPI_TalonSRX(ClimbConstants.kClimbController2Port);
    private final WPI_VictorSPX m_ClimbMotor2 = new WPI_VictorSPX(ClimbConstants.kClimbController2Port);

    private int climbinvert = 1;

    public ClimbSubsystem() {
        m_ClimbMotor2.setInverted(true);
        setOutput(0,0);
    }

    @Log
    public void setOutput(double motorPercent, double motorPercent2) {
        this.m_ClimbMotor.set(motorPercent * climbinvert);
        this.m_ClimbMotor2.set(motorPercent2 * climbinvert);

        // As soon as we start raising the hooks switch the camera so the hook view is larger
        if(motorPercent > .5 || motorPercent2 > .5) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
        }
    }

    @Config.ToggleButton
    public void invertclimber(boolean enabled) {
        if (enabled) {
            climbinvert = -1;
        }
        else {
            climbinvert = 1;
        }
    }

}