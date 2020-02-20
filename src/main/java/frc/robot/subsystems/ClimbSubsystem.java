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
    @Config(name="ClimbMotorLeft")
    //private final WPI_TalonSRX m_ClimbMotor = new WPI_TalonSRX(ClimbConstants.kClimbControllerPort);
    private final WPI_VictorSPX m_LeftClimbMotor = new WPI_VictorSPX(ClimbConstants.kClimbLeftControllerPort);
    
    @Config(name="ClimbMotorRight")
    //private final WPI_TalonSRX m_ClimbMotor2 = new WPI_TalonSRX(ClimbConstants.kClimbController2Port);
    private final WPI_VictorSPX m_RightClimbMotor = new WPI_VictorSPX(ClimbConstants.kClimbRightControllerPort);

    private int climbinvert = 1;

    public ClimbSubsystem() {
        m_RightClimbMotor.setInverted(true);
        setOutput(0,0);
    }

    public void setOutput(double leftMotorPercent, double rightMotorPercent) {
        this.m_LeftClimbMotor.set(leftMotorPercent * climbinvert);
        this.m_RightClimbMotor.set(rightMotorPercent * climbinvert);

        // As soon as we start raising the hooks switch the camera so the hook view is larger
        if(leftMotorPercent > .5 || rightMotorPercent > .5) {
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