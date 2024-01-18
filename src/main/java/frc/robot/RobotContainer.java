package frc.robot;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();

    public double intakeVec = 0;

    public Command autoCode = Commands.sequence(new PrintCommand("no auto selected"));

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final JoystickButton driveStraight = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton e_presButton_0 = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton e_presButton_1 = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton e_presButton_2 = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton e_presButton_3 = new JoystickButton(operator, XboxController.Button.kB.value);


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton xModeButton = new JoystickButton(driver, XboxController.Button.kX.value);

    /* LED Initialization Code */


    /* Variables */
    boolean driveStatus = false;

    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      s_Swerve.setDefaultCommand(
        new TeleopSwerve(
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> -driver.getRawAxis(rotationAxis), 
          () -> robotCentric.getAsBoolean()
        )
      );

        
      // Configure the button bindings
      configureButtonBindings();
      
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link 
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        /* Driver Buttons (and op buttons) */

        robotCentric.onTrue(new InstantCommand(() -> s_Swerve.changeFieldRelative()));
        
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        driveStraight.whileTrue(new RunCommand(() -> s_Swerve.driveStraight()));

        // xModeButton.whileTrue(new InstantCommand(()-> s_Swerve.setXMode()));
        
    }
    
    public void printValues(){
        SmartDashboard.putNumber("balanceP", 0.03);
        // SmartDashboard.getNumber("balanceI", elevatorAxis);
        // SmartDashboard.getNumber("balanceD", elevatorAxis);


        SmartDashboard.putBoolean("Pov pressed", e_presButton_0.getAsBoolean());
        SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll().getValueAsDouble());


        // SmartDashboard.putNumber("RX", s_Limelight.getRX());
        // SmartDashboard.putNumber("RY", s_Limelight.getRY());
        // SmartDashboard.putNumber("RZ", s_Limelight.getRZ());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {

      Constants.gyroOffset = s_Swerve.gyro.getPitch().getValueAsDouble();
      //s_Swerve.zeroGyro();
      s_Swerve.gyro.setYaw(180);
      return null;
    }
}
