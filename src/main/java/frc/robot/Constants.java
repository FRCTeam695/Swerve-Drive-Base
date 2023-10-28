package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class Swerve{
        public static final double MAX_SPEED_METERS_PER_SECONDS = Units.feetToMeters(13.5);
        public static final double MAX_ANGULAR_SPEED_METERS_PER_SECOND = 2 * Math.PI;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND = 3;
        public static final double TURNING_GEAR_RATIO = 150.0/7;
        public static final double DRIVING_GEAR_RATIO = 8.14;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(4 * Math.PI);
        public static final double THETA_KP_VALUE = 0.015;
        public static final double PROFILED_KP_VALUE = 1; 
        public static final double MAX_VELOCITY_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final int FRONT_RIGHT_DRIVE_ID = 13;
        public static final int FRONT_RIGHT_TURN_ID = 12;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;
        public static final int FRONT_RIGHT_ABS_ENCODER_OFFSET = 171;
        public static final boolean FRONT_RIGHT_DRIVE_MOTOR_REVERSED = true;

        public static final int FRONT_LEFT_DRIVE_ID = 23;
        public static final int FRONT_LEFT_TURN_ID = 22;
        public static final int FRONT_LEFT_CANCODER_ID = 21;
        public static final int FRONT_LEFT_ABS_ENCODER_OFFSET = 49;
        public static final boolean FRONT_LEFT_DRIVE_MOTOR_REVERSED = true;

        public static final int BACK_LEFT_DRIVE_ID = 33;
        public static final int BACK_LEFT_TURN_ID = 32;
        public static final int BACK_LEFT_CANCODER_ID = 31;
        public static final int BACK_LEFT_ABS_ENCODER_OFFSET = 349;
        public static final boolean BACK_LEFT_DRIVE_MOTOR_REVERSED = true;

        public static final int BACK_RIGHT_DRIVE_ID = 43;
        public static final int BACK_RIGHT_TURN_ID = 42;
        public static final int BACK_RIGHT_CANCODER_ID = 41;
        public static final int BACK_RIGHT_ABS_ENCODER_OFFSET = 207;
        public static final boolean BACK_RIGHT_DRIVE_MOTOR_REVERSED = false;


        public static final TrapezoidProfile.Constraints TRAPEZOID_THETA_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double TRACK_WIDTH = Units.inchesToMeters(19);
        public static final double WHEEL_BASE = Units.inchesToMeters(19);
    
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                    //Positive x values represent moving towards the front of the robot, positive y values represent moving to the left
                    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //Front right wheel
                    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //Front left wheel
                    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), //Back left wheel
                    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //Back right wheel
    }
}
