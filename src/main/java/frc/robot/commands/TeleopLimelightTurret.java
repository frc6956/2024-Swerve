
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.apriltagvision.Vision;

public class TeleopLimelightTurret extends Command {
    private final Vision vision;
    private final SwerveDrivetrain swervedrivetrain;
    private final DoubleSupplier translation;
    private final DoubleSupplier strafe;
    private final BooleanSupplier robotCentric;
    private final DoubleSupplier rotationJoy;

    public TeleopLimelightTurret(Vision vision, SwerveDrivetrain swervedrivetrain, DoubleSupplier translation, DoubleSupplier strafeSup, DoubleSupplier rotationJoy, BooleanSupplier robotCentric) {
        this.vision = vision;
        this.swervedrivetrain = swervedrivetrain;
        this.translation = translation;
        this.strafe = strafeSup;
        this.rotationJoy = rotationJoy;
        this.robotCentric = robotCentric;
        addRequirements(this.vision, swervedrivetrain);
        vision.setPipeline(1);
    }

    /* how to go to apriltag:
     * find tag (duh)
     * find position of tag relative to robot
     * rotate from current position to tag using rotation PID controller
     * variate PID magnitude by distance factor
     * allow turret mode ("locked" rotation to tag)
     */
    @Override
    public void execute() {
        vision.updateTables();
        vision.setPipeline(1);

       /* Apply Deadband*/
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), RobotContainer.JOYSTICK_AXIS_THRESHOLD);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), RobotContainer.JOYSTICK_AXIS_THRESHOLD);
        double rotationJoy = MathUtil.applyDeadband(strafe.getAsDouble(), RobotContainer.JOYSTICK_AXIS_THRESHOLD);

        /* Calculate Rotation Magnitude */
        PIDController rotController = new PIDController(
                Constants.Vision.VISION_P,
                Constants.Vision.VISION_I,
                Constants.Vision.VISION_D
        );

        rotController.enableContinuousInput(Constants.MINIMUM_ANGLE, Constants.MAXIMUM_ANGLE);

        // Calculate more accurate target using RX and RZ angle values, then get rid of varied P in PID

        
        double rotate = rotController.calculate(
            0,
            (vision.getTX() < 0 ? -1 : 1) * Math.atan(
                Math.abs(vision.getTX()) / Math.abs(vision.getTZ())
            )
        );

        SmartDashboard.putNumber("Vision Rotate", rotate);
        /* Drive */

        if (vision.hasValidTarget() && vision.hasSpeakerTarget()){
            swervedrivetrain.drive(
                translationVal,
                strafeVal,
                -rotate,
            true,
            true
            ); 
        }else {
            swervedrivetrain.drive(
                translationVal,
                strafeVal,
                -rotationJoy,
            true,
            true
            );
        }
    }

	@Override
	public void end(boolean interrupted) {
        vision.setPipeline(0);
	}
        
}
 
