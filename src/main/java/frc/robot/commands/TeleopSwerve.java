
package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;


public class TeleopSwerve extends Command {    
    private final SwerveDrivetrain swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;

    public TeleopSwerve(
            SwerveDrivetrain swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup
    ) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), RobotContainer.JOYSTICK_AXIS_THRESHOLD);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), RobotContainer.JOYSTICK_AXIS_THRESHOLD);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), RobotContainer.JOYSTICK_AXIS_THRESHOLD);

        /* Drive */
        swerve.drive(
            translationVal,
            strafeVal,
            +rotationVal,
            true,
            true
        );
    }
}