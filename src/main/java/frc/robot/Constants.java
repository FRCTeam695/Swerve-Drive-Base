package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double MAX_SPEED_METERS_PER_SECONDS = 4.572;
    public static final double MAX_ANGULAR_SPEED_METERS_PER_SECOND = 2 * Math.PI; // Physical max speed is 4 * PI
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(21.25);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                //Positive x values represent moving towards the front of the robot, positive y values represent moving to the left
                new Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2), //Front right wheel
                new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2), //Front left wheel
                new Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2), //Back left wheel
                new Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2)); //Back right wheel


                //In hindsight when im looking at this I realize that it should be what is written below, will change on Tuesday
                //new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //Front right wheel
                //new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //Front left wheel
                //new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), //Back left wheel
                //new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //Back right wheel
}
