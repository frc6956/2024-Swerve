// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.Vision;
import frc.robot.commands.TeleopLimelightCenter;
import frc.robot.commands.TeleopLimelightTurret;
import frc.robot.commands.drivetrain.*;
import frc.robot.auton.common.*;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

	public static final int LX = 0;
	public static final int LY = 1;
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4;
	public static final int RY = 5;

	// choosers (for auton)
	
	public static final String AUTON_DO_NOTHING = "Do Nothing";
	public static final String AUTON_CUSTOM = "My Auto";
	public static final String AUTON_SAMPLE_SWERVE = "Sample Swerve";
	public static final String AUTON_SAMPLE_MOVE_FORWARD = "Sample Move Forward";
	public static final String AUTON_SAMPLE_MOVE_IN_REVERSE = "Sample Move In Reverse";
	public static final String AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE = "Sample Move In Gamma Shape";
	public static final String AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE = "Sample Move In L Shape In Reverse";
	public static final String AUTON_TEST_HARDCODED_MOVE_1 = "Test Hardcoded Move 1";
	public static final String AUTON_TEST_HARDCODED_MOVE_2 = "Test Hardcoded Move 2";
	private String autonSelected;
	private SendableChooser<String> autonChooser = new SendableChooser<>();


	// motorized devices
	private final Pigeon2 gyro = new Pigeon2(0);
	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro);
	private final Vision vision = new Vision();
	
	// pneumatic devices

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field


	// The driver's and copilot's joystick(s) and controller(s)

	/*CommandJoystick joyLeft = new CommandJoystick(Ports.USB.LEFT_JOYSTICK);
	CommandJoystick joyRight = new CommandJoystick(Ports.USB.RIGHT_JOYSTICK);*/
	//CommandJoystick joyMain = new CommandJoystick(Ports.USB.MAIN_JOYSTICK);
	CommandXboxController driveController = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
	//CommandXboxController copilotGamepad = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);
	

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// choosers (for auton)
		
		autonChooser.setDefaultOption("Do Nothing", AUTON_DO_NOTHING);
		autonChooser.addOption("My Auto", AUTON_CUSTOM);
		autonChooser.addOption("Sample Swerve", AUTON_SAMPLE_SWERVE);
		autonChooser.addOption("Sample Move Forward", AUTON_SAMPLE_MOVE_FORWARD);
		autonChooser.addOption("Sample Move In Reverse", AUTON_SAMPLE_MOVE_IN_REVERSE);
		autonChooser.addOption("Sample Move In Gamma Shape", AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE);
		autonChooser.addOption("Sample Move In L Shape In Reverse", AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE);
		autonChooser.addOption("Test Hardcoded Move 1", AUTON_TEST_HARDCODED_MOVE_1);
		autonChooser.addOption("Test Hardcoded Move 2", AUTON_TEST_HARDCODED_MOVE_2);
		SmartDashboard.putData("Auto choices", autonChooser);
		

		// Configure the button bindings

		configureButtonBindings();


		// Configure default commands

		drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			new RunCommand(
				() -> drivetrain.drive(
					-MathUtil.applyDeadband(driveController.getLeftY(), JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driveController.getLeftX(), JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driveController.getRightX(), JOYSTICK_AXIS_THRESHOLD),
					true, true),
				drivetrain));

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

		// driver (joystick)

		driveController.povUp()
			.onTrue(new DrivetrainZeroHeading(drivetrain));	

			//driveController.povDown()
			//.onTrue(new DrivetrainOppositeHeading(drivetrain));	

			driveController.povDown()
			.whileTrue(new DrivetrainSetXFormation(drivetrain));	

			driveController.button(1)
			.whileTrue(
				new TeleopLimelightCenter(
					vision, 
					drivetrain, 
					() -> -driveController.getLeftY(),
					() -> -driveController.getLeftX(),
					() -> driveController.getRightX(),
					() -> true)
			);
			
			//driveController.button(3)
			//.onTrue(new MoveInLShapeInReverse(drivetrain, this, 3));
			
			//driveController.button(4)
			//.onTrue(new MoveInGammaShape(drivetrain, this, 3));

			//driveController.button(5)
			//.onTrue(new MoveForward(drivetrain, this, 3));
			//.onTrue(new DrivetrainTurnAngleUsingPidController(drivetrain, -90));
			//.onTrue(new MoveInUShapeInReverse(drivetrain, this, 1));

			driveController.button(2)
			//.onTrue(new MoveInReverse(drivetrain, this, 3));
			//.onTrue(new DrivetrainTurnAngleUsingPidController(drivetrain, 90));
			.whileTrue(
				new TeleopLimelightTurret(
					vision, 
					drivetrain, 
					() -> -driveController.getLeftY(),
					() -> -driveController.getLeftX(),
					() -> driveController.getRightX(),
					() -> true)
			);

/*
		joyMain.button(7)
			.whileTrue(new RollerJoystickControl(roller, drivetrain, getMainJoystick()));
		
		joyMain.button(8)
			.whileTrue(new NeckJoystickControl(neck, drivetrain, getMainJoystick()));
		
		joyMain.button(9)
			.whileTrue(new DrawerJoystickControl(drawer, drivetrain, getMainJoystick()));
		
		joyMain.button(10)
			.whileTrue(new ElevatorJoystickControl(elevator, drivetrain, getMainJoystick()));

		//joyMain.button(11)
			//.onTrue(new DrivetrainZeroHeading(drivetrain));
		
		//joyMain.button(12)
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));
			
				
		// copilot (gamepad)
		
		copilotGamepad.a()
			.whileTrue(new RollerRelease(roller));
		
		copilotGamepad.b()
			.whileTrue(new RollerRoll(roller));

		copilotGamepad.x()
			.onTrue(new MouthSafeClose(mouth, neck, getCopilotGamepad()));

		copilotGamepad.y()
			.onTrue(new MouthOpen(mouth));

		copilotGamepad.back()
			.onTrue(new DrivetrainAndGyroReset(drivetrain));

		copilotGamepad.start()
			.onTrue(new AlmostEverythingStop(elevator, drawer, neck, roller));


		copilotGamepad.leftTrigger()
			.onTrue(new DrawerRetractWithStallDetection(drawer));

		copilotGamepad.rightTrigger()
			.onTrue(new DrawerExtendWithStallDetection(drawer));


		copilotGamepad.povDown()
			.onTrue(new ElevatorMoveDownWithStallDetection(elevator));

		copilotGamepad.povLeft()
			.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));

		copilotGamepad.povRight()
			.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));

		copilotGamepad.povUp()
			.onTrue(new ElevatorMoveUpWithStallDetection(elevator));


		copilotGamepad.leftBumper()
			.onTrue(new NeckSafeMoveUpWithStallDetection(neck, mouth, getCopilotGamepad()));

		copilotGamepad.rightBumper()
			.onTrue(new NeckMoveDownWithStallDetection(neck));


		copilotGamepad.leftStick()
			.onTrue(new RollerTimedRoll(roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));			

		copilotGamepad.rightStick()
			.onTrue(new RollerTimedRelease(roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));


		copilotGamepad.axisGreaterThan(LY,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		copilotGamepad.axisLessThan(LY,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));
*/
		/*copilotGamepad.axisGreaterThan(LX,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();

		copilotGamepad.axisLessThan(LX,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();*/
/*
		copilotGamepad.axisGreaterThan(RY,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RY,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisGreaterThan(RX,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RX,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));	
*/			
	}


	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		autonSelected = autonChooser.getSelected();
		System.out.println("Auton selected: " + autonSelected);
		

		switch (autonSelected) {
			case AUTON_SAMPLE_SWERVE:
				//return createSwerveControllerCommand(createExampleTrajectory());
				//return new DrivetrainSwerveRelative(drivetrain, this, createExampleTrajectory());
				return new MoveInSShape(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_FORWARD:
				return new MoveForward(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_REVERSE:
				return new MoveInReverse(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE:
				return new MoveInGammaShape(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE:
				return new MoveInLShapeInReverse(drivetrain, this, 3);
				//break;

			case AUTON_TEST_HARDCODED_MOVE_1:
				return new CompletelyLeaveCommunity(drivetrain, this);
				//break;

			case AUTON_TEST_HARDCODED_MOVE_2:
				return new MoveInNonBumpKTurn(drivetrain, this);
				//break;


			case AUTON_DO_NOTHING:
				return null;
				//break;
				
			default:
				// nothing
				return null;
				//break;
		} // end switch
	}

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createReverseTrajectoryConfig() {

		TrajectoryConfig config = createTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	/*public Trajectory createExampleTrajectory() {
		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
			createTrajectoryConfig());

		return exampleTrajectory;
	}*/
	
	/*public Command createSwerveControllerCommand(Trajectory trajectory) {

		ProfiledPIDController thetaController = new ProfiledPIDController(
			AutoConstants.THETA_CONTROLLER_P, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
			
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			trajectory, // trajectory to follow
			drivetrain::getPose, // Functional interface to feed supplier
			DrivetrainConstants.DRIVE_KINEMATICS, // kinematics of the drivetrain
			new PIDController(AutoConstants.X_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for x position
			new PIDController(AutoConstants.Y_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for y position
			thetaController, // trajectory tracker PID controller for rotation
			drivetrain::setModuleStates, // raw output module states from the position controllers
			drivetrain); // subsystems to require

		// Reset odometry to the starting pose of the trajectory.
		drivetrain.resetOdometry(trajectory.getInitialPose()); // WARNING: https://github.com/REVrobotics/MAXSwerve-Java-Template/issues/13

		field.getObject("trajectory").setTrajectory(trajectory);

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false, false));
	}*/

	public Field2d getField()
	{
		return field;
	}

	public SwerveDrivetrain getDrivetrain()
	{
		return drivetrain;
	}

	public Vision getVision(){
		return vision;
	}

	//public Joystick getMainJoystick()
	//{
//		return joyMain.getHID();
//	}

	public XboxController getCopilotGamepad()
	{
		return driveController.getHID();
	}

	public SendableChooser<String> getAutonChooser()
	{
		return autonChooser;
	}
}