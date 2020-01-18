/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// WPI Imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

// Command Imports
import frc.robot.commands.ExampleCommand;

// Subsystem Imports
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

// Constant Imports
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  @Log
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  @Log
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  @Log
  private final ControlPanelSubsystem m_controlpanel = new ControlPanelSubsystem();
  @Log
  private final LEDSubsystem m_LED = new LEDSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // The driver's controller
  @Config
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // The operator's controller
  @Config
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive
            .arcadeDrive(m_driverController.getY(GenericHID.Hand.kLeft),
                         m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));
                         
    // Sets the LEDs to start up with a rainbow config
    m_LED.setDefaultCommand(
      new RunCommand(() -> m_LED.rainbow()));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // TODO need to decide on which buttons to use and if they should be driver / operator
    // Spin up the shooter when the 'A' button is pressed
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new InstantCommand(m_shooter::enable, m_shooter));

    // Turn off the shooter when the 'B' button is pressed
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(new InstantCommand(m_shooter::disable, m_shooter));

    // Run the feeder when the 'X' button is held, but only if the shooter is at speed
    new JoystickButton(m_driverController, Button.kX.value).whenPressed(new ConditionalCommand(
        // Run the feeder
        new InstantCommand(m_shooter::runFeeder, m_shooter),
        // Do nothing
        new InstantCommand(),
        // Determine which of the above to do based on whether the shooter has reached the
        // desired speed
        m_shooter::atSetpoint)).whenReleased(new InstantCommand(m_shooter::stopFeeder, m_shooter));

    // Rainbow Pattern on LEDs when 'Y' button is pressed
    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(new InstantCommand(m_LED::rainbow, m_LED));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
