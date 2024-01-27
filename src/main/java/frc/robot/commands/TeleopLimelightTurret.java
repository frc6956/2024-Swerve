
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.apriltagvision.Vision;

public class TeleopLimelightTurret extends Command {
    private final Vision vision;
    private final SwerveDrivetrain swervedrivetrain;
    private final DoubleSupplier translation;
    private final DoubleSupplier strafe;
    private final BooleanSupplier robotCentric;

    public TeleopLimelightTurret(Vision vision, SwerveDrivetrain swervedrivetrain, DoubleSupplier translation, DoubleSupplier strafeSup, BooleanSupplier robotCentric) {
        this.vision = vision;
        this.swervedrivetrain = swervedrivetrain;
        this.translation = translation;
        this.strafe = strafeSup;
        this.robotCentric = robotCentric;
        addRequirements(this.vision, swervedrivetrain);
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

       /* Apply Deadband*/
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.STICK_DEADBAND);

        /* Calculate Rotation Magnitude */
        PIDController rotController = new PIDController(
                Constants.Vision.VISION_P,
                Constants.Vision.VISION_I,
                Constants.Vision.VISION_D
        );
        rotController.enableContinuousInput(Constants.MINIMUM_ANGLE, Constants.MAXIMUM_ANGLE);

        // TODO: Calculate more accurate target using RX and RZ angle values, then get rid of varied P in PID

        
        double rotate = rotController.calculate(swervedrivetrain.getYaw(), swervedrivetrain.getYaw() + 15 * limelight.getRX());

        /* Drive */
        swervedrivetrain.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.SwerveDrivetrain.MAX_SPEED),
            -rotate,
            !robotCentric.getAsBoolean(),
            true
        );
    }
 
}