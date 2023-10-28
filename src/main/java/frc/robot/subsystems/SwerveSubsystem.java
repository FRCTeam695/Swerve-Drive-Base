package frc.robot.subsystems;

//import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontRight;
    private final SwerveModule frontLeft;
    private final SwerveModule bottomLeft;
    private final SwerveModule bottomRight;
    private final Field2d m_field = new Field2d();

    // creates the odometry class
    public SwerveDriveOdometry odometry;

    // initialize the gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private double gyroAngle;

    public double tickStart;
    public ChassisSpeeds latestChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    public SwerveSubsystem() {
        // Summer Swerve Chassis
        frontRight = new SwerveModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET, Constants.Swerve.FRONT_RIGHT_CANCODER_ID);
        frontLeft = new SwerveModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET, Constants.Swerve.FRONT_LEFT_CANCODER_ID);
        bottomLeft = new SwerveModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET, Constants.Swerve.BACK_LEFT_CANCODER_ID);
        bottomRight = new SwerveModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET, Constants.Swerve.BACK_RIGHT_CANCODER_ID);

        SwerveDriveKinematics driveKinematics = Constants.Swerve.kDriveKinematics;
        odometry = new SwerveDriveOdometry(driveKinematics, new Rotation2d(),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(),
                        bottomLeft.getPosition(), bottomRight.getPosition() });

        tickStart = 0;

        // resets the gyro, it is calibrating when this code is reached so we reset it
        // on a different thread with a delay
        new Thread(() -> {
            try {
                Thread.sleep(500);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    //Sets the gyro heading to 0
    public void zeroHeading() {
        gyro.reset();
    }

    //getPitch is used for balancing the robot on the charge station
    public double getPitch() {
        return gyro.getPitch();
    }

    //getHeading returns the direction the robot is facing
    //assumes we have already reset the gyro
    public double getHeading() {
        gyroAngle = -1 * Math.IEEEremainder(gyro.getAngle(), 360);
        SmartDashboard.putNumber("Gyro Reading", gyroAngle);
        return gyroAngle; // Multiply by negative one because on wpilib as you go counterclockwise angles
                          // should get bigger
    }

    //Outdated method for using pure odometry to make autons
    public void startTickCount() {
        tickStart = frontLeft.getDriveTicks();
    }

    //Outdated method for using pure odometry to make autons
    public double getTicks() {
        return Math.abs(frontLeft.getDriveTicks() - tickStart);
    }

    //Gets the pose of the robot from the odometry, used in any self driving periods
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    //Sets all of the swerve modules
    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS);
        frontRight.setDesiredState(desiredStates[0], 1);
        frontLeft.setDesiredState(desiredStates[1], 2);
        bottomLeft.setDesiredState(desiredStates[2], 3);
        bottomRight.setDesiredState(desiredStates[3], 4);
    }

    //returns the position of each swerve module (check SwerveModule.java for further details)
    public SwerveModulePosition getModulePosition(int motor) {
        switch (motor) {
            case 1:
                return frontRight.getPosition();
            case 2:
                return frontLeft.getPosition();
            case 3:
                return bottomLeft.getPosition();
            case 4:
                return bottomRight.getPosition();
            default:
                return bottomLeft.getPosition();
        }
    }

    //Gets the position of the wheel as returned by the absolute encoder (check SwerveModule.java for further details)
    public double getAbsoluteEncoderValue(int motor) {
        switch (motor) {
            case 1:
                return frontRight.getAbsoluteEncoderRadians();
            case 2:
                return frontLeft.getAbsoluteEncoderRadians();
            case 3:
                return bottomLeft.getAbsoluteEncoderRadians();
            case 4:
                return bottomRight.getAbsoluteEncoderRadians();
            default:
                return -1;
        }
    }

    //Gets the swerve module state of the motor (check SwerveModule.java for further details)
    public SwerveModuleState getState(int motor) {
        switch (motor) {
            case 1:
                return frontRight.getState();
            case 2:
                return frontLeft.getState();
            case 3:
                return bottomLeft.getState();
            case 4:
                return bottomRight.getState();
            default:
                return bottomRight.getState(); // just bcs I need a default
        }
    }

    //Gets the relative encoder position of each swerve module (check SwerveModule.java for further details)
    public double getRelativeTurnEncoderValue(int motor) {
        switch (motor) {
            case 1:
                return frontRight.getTurnPosition(false);
            case 2:
                return frontLeft.getTurnPosition(false);
            case 3:
                return bottomLeft.getTurnPosition(false);
            case 4:
                return bottomRight.getTurnPosition(false);
            default:
                return -1;
        }
    }

    // resets the drive motors (check SwerveModule.java for further details)
    public void resetDriveMotorEncoders(){
        frontRight.zeroDrivePosition();
        frontLeft.zeroDrivePosition();
        bottomLeft.zeroDrivePosition();
        bottomRight.zeroDrivePosition();
    }

    //Sets the relative encoder value using the absolute encoders (check SwerveModule.java for further details)
    public void setRelativeTurnEncoderValue() {
        frontRight.setTurnEncoder(frontRight.getAbsoluteEncoderRadians());
        frontLeft.setTurnEncoder(frontLeft.getAbsoluteEncoderRadians());
        bottomLeft.setTurnEncoder(bottomLeft.getAbsoluteEncoderRadians());
        bottomRight.setTurnEncoder(bottomRight.getAbsoluteEncoderRadians());
    }

    //periodic updates the odometry object
    @Override
    public void periodic() {
        odometry.update(new Rotation2d(getHeading() * Math.PI / 180),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(),
                        bottomLeft.getPosition(), bottomRight.getPosition() });
        m_field.setRobotPose(getPose());
        SmartDashboard.putData("field", m_field);

        
    }

    //Allows us to manually reset the odometer
    public void setOdometer(SwerveModulePosition[] modulePositions, Pose2d pose) {
        odometry.resetPosition(new Rotation2d(getHeading() * Math.PI / 180), modulePositions, pose);
    }

    //Allows us to manually reset the odometer, used with vision pose extrapolation
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        frontRight.getPosition(),
                        frontLeft.getPosition(),
                        bottomLeft.getPosition(),
                        bottomRight.getPosition()
                },
                pose);
    }

    //Stops all of the swerve modules
    public void stopModules() {
        frontRight.stop();
        frontLeft.stop();
        bottomLeft.stop();
        bottomRight.stop();
    }

    public ChassisSpeeds getChassisSpeeds(){
        return latestChassisSpeeds;
    }

    //Method that actually drives swerve
    public void driveSwerve(double Xj, double Zj, double Yj, boolean feildOriented) {
        // Deadband
        Xj = Math.abs(Xj) > 0.01 ? Xj : 0;
        Yj = Math.abs(Yj) > 0.01 ? Yj : 0;
        Zj = Math.abs(Zj) > 0.01 ? Zj : 0;

        // Scale up the speeds, WPILib likes them in meters per second
        Xj = Xj * Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;
        Yj = Yj * Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;
        Zj = Zj * Constants.Swerve.MAX_ANGULAR_SPEED_METERS_PER_SECOND;

        // construct chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (feildOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(Yj, Xj, Zj, Rotation2d.fromDegrees(getHeading()));
        } else {
            chassisSpeeds = new ChassisSpeeds(Yj, Xj, Zj);
        }

        latestChassisSpeeds = chassisSpeeds;

        // convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // set the modules to their desired speeds
        setModules(moduleStates);
    }
}
