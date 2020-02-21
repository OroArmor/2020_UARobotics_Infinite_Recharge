/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

// WPI Imports
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

// Command Imports
import frc.robot.commands.AutoAim;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfiled;
// Subsystem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

// Constant Imports
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  @Log
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  @Log
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
 /*  @Log
  private final ControlPanelSubsystem m_controlpanel = new ControlPanelSubsystem(); */
  @Log
  private final LEDSubsystem m_LED = new LEDSubsystem();
  @Log
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  @Log
  private final ClimbSubsystem m_climb = new ClimbSubsystem();

  @Log.PDP
  PowerDistributionPanel m_PDP = new PowerDistributionPanel(0);
  
  // Creating this so we get logging in the Command
  Command m_TurnToAngle = new TurnToAngle(0, m_robotDrive);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // The operator's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Limelight Setup
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive
            .arcadeDrive(-m_driverController.getY(GenericHID.Hand.kLeft),
                         m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));

    m_intake.setDefaultCommand(
        // Use right trigger to control the speed of the intake
        new RunCommand(() -> m_intake
            .setOutput(m_driverController.getRawAxis(3)), m_intake));

/*     m_controlpanel.setDefaultCommand(
      // Use left x axis to control the speed of the control panel
      new RunCommand(
        () -> m_controlpanel
          .setOutput(m_operatorController.getRawAxis(0)), m_controlpanel)); */

    m_climb.setDefaultCommand(
      // Use right y axis to control the speed of the climber
      new RunCommand(
        () -> m_climb
          .setOutput(m_operatorController.getRawAxis(2),
            m_operatorController.getRawAxis(3)), m_climb));

/*     m_controlpanel.setDefaultCommand(
      new RunCommand(
        () -> m_controlpanel
          .StartColorFind(m_operatorController.getRawAxis(2)), m_controlpanel)); */
                         
    // Sets the LEDs to start up with a rainbow config
    //m_LED.rainbow();

    // Makes sure climber is not inverted
    m_climb.invertclimber(false);

    autoChooser.addOption("Auto Aim", new AutoAim(m_robotDrive));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Spin up the shooter to Auto Line Speed when the 'Start' button is pressed
    new JoystickButton(m_driverController, Button.kStart.value)
      .or(new JoystickButton(m_operatorController, Button.kStart.value))
      .whenActive(new InstantCommand(() -> {
        m_shooter.setSetpoint(ShooterConstants.kShooterAutoLineRPM);
        m_shooter.enable();
      }, m_shooter));
    
    // Spin up the shooter to the near trench speed when the 'Back' button is pressed
    new JoystickButton(m_driverController, Button.kBack.value)
      .or(new JoystickButton(m_operatorController, Button.kBack.value))
      .whenActive(new InstantCommand(() -> {
        m_shooter.setSetpoint(ShooterConstants.kShooterNearTrenchRPM);
        m_shooter.enable();
      }, m_shooter));

    // Spin up the shooter to far trench speed when the 'X' button is pressed.
    new JoystickButton(m_driverController, Button.kX.value)
      .or(new JoystickButton(m_operatorController, Button.kX.value))
      .whenActive(new InstantCommand(() -> {
        m_shooter.setSetpoint(ShooterConstants.kShooterFarTrenchRPM);
        m_shooter.enable();
      }, m_shooter));

    // Stop the Shooter when the B button is pressed
    new JoystickButton(m_driverController, Button.kB.value)
      .or(new JoystickButton(m_operatorController, Button.kB.value))
      .whenActive(new InstantCommand(() -> {
        m_shooter.setSetpoint(0);
        m_shooter.disable();
      }, m_shooter));

    // Run the feeder when the 'A' button is held, but only if the shooter is at speed
    new JoystickButton(m_driverController, Button.kA.value)
      .or(new JoystickButton(m_operatorController, Button.kA.value))  
      .whileActiveOnce(new ConditionalCommand(
          // Run the feeder
          new InstantCommand(m_shooter::runFeeder, m_shooter),
          // Do nothing
          new InstantCommand(),
          // Determine which of the above to do based on whether the shooter has reached the
          // desired speed
          m_shooter::atSetpoint)).whenInactive(new InstantCommand(m_shooter::stopFeeder, m_shooter));

    // When right bumper is pressed raise/lower the intake and stop/start the conveyor and intake on both controllers
    new JoystickButton(m_operatorController, Button.kBumperRight.value).or(new JoystickButton(m_driverController, Button.kBumperRight.value))
      .whenActive(new InstantCommand(() -> m_intake.toggleIntakePosition(true), m_intake)
      .andThen(new InstantCommand(() -> m_intake.toggleIntakeWheels(true), m_intake))
      .andThen(new InstantCommand(() -> m_intake.toggleConveyor(true), m_intake)));

    /*     // When left bumper is pressed spin control panel
    new JoystickButton(m_operatorController, Button.kBumperLeft.value).or(new JoystickButton(m_driverController, Button.kBumperLeft.value))
      .whenActive(new InstantCommand(m_controlpanel::rotateWheel, m_controlpanel)); */

    new JoystickButton(m_operatorController, Button.kBumperLeft.value).or(new JoystickButton(m_driverController, Button.kBumperLeft.value))
      .whileActiveContinuous(new InstantCommand(() -> {
        m_robotDrive.drivestraight(120); }, m_robotDrive)
      .beforeStarting(m_robotDrive::distancesetup));

    // Auto Aim when Y button is pressed
    /* new JoystickButton(m_driverController, Button.kY.value)
      .whileHeld(new AutoAim(m_robotDrive, m_shooter));
     */

    new JoystickButton(m_driverController, Button.kY.value).whenPressed(() ->
      m_robotDrive.createCommandForTrajectory("LS to CP"));
     
    // Create "button" from POV Hat in up direction.  Use both of the angles to the left and right also.
    new POVButton(m_driverController, 315).or(new POVButton(m_driverController, 0)).or(new POVButton(m_driverController, 45))
      .whenActive(new TurnToAngle(90, m_robotDrive).withTimeout(5));
    
    // Create "button" from POV Hat in down direction.  Use both of the angles to the left and right also.
    new POVButton(m_driverController, 225).or(new POVButton(m_driverController, 180)).or(new POVButton(m_driverController, 135))
      .whenActive(new TurnToAngle(-90, m_robotDrive).withTimeout(5));
  }

  @Config(name="shooterPID")
  public void setPID(double kP, double kI, double kD, double kF) {
    m_shooter.getController().setPID(kP, kI, kD);
  }

  @Log
  public double getTurn(){
    return m_driverController.getX(GenericHID.Hand.kRight);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
