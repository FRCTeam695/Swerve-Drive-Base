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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontRight;
    private final SwerveModule frontLeft;
    private final SwerveModule bottomLeft;
    private final SwerveModule bottomRight;

    // creates the odometry class
    public SwerveDriveOdometry odometry;
    private final double maxSpeedMPS;

    // initialize the gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private double gyroAngle;

    public double tickStart;

    public SwerveSubsystem() {
        // Summer Swerve Chassis
        frontRight = new SwerveModule(13, 12, true, 171, 11);
        frontLeft = new SwerveModule(23, 22, true, 49, 21);
        bottomLeft = new SwerveModule(33, 32, true, 349, 31);
        bottomRight = new SwerveModule(43, 42, false, 207, 41);

        // Flint Swerve Chassis
        // frontRight = new SwerveModule(13, 12, true, 158, 11);
        // frontLeft = new SwerveModule(23, 22, true, 222, 21);
        // bottomLeft = new SwerveModule(33, 32, true, 247, 31);
        // bottomRight = new SwerveModule(43, 42, false, 29, 41);
        SwerveDriveKinematics driveKinematics = Constants.Swerve.kDriveKinematics;
        odometry = new SwerveDriveOdometry(driveKinematics, new Rotation2d(),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(),
                        bottomLeft.getPosition(), bottomRight.getPosition() });
        maxSpeedMPS = Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;

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

    public void zeroHeading() {
        gyro.reset();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getHeading() {
        gyroAngle = -1 * Math.IEEEremainder(gyro.getAngle(), 360);
        SmartDashboard.putNumber("Gyro Reading", gyroAngle);
        return gyroAngle; // Multiply by negative one because on wpilib as you go counterclockwise angles
                          // should get bigger
    }

    public void startTickCount() {
        tickStart = frontLeft.getDriveTicks();
    }

    public double getTicks() {
        return Math.abs(frontLeft.getDriveTicks() - tickStart);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeedMPS);
        SmartDashboard.putNumber("Degrees Module 1", desiredStates[0].angle.getDegrees());
        frontRight.setDesiredState(desiredStates[0], 1);
        frontLeft.setDesiredState(desiredStates[1], 2);
        bottomLeft.setDesiredState(desiredStates[2], 3);
        bottomRight.setDesiredState(desiredStates[3], 4);
    }

    public SwerveModulePosition getModulePosition(int motor) {
        switch (motor) {
            case 1:
                SmartDashboard.putNumber("Module Position 1: ", getRelativeTurnEncoderValue(1));
                return frontRight.getPosition();
            case 2:
                SmartDashboard.putNumber("Module Position 2: ", getRelativeTurnEncoderValue(2));
                return frontLeft.getPosition();
            case 3:
                SmartDashboard.putNumber("Module Position 3: ", getRelativeTurnEncoderValue(3));
                return bottomLeft.getPosition();
            case 4:
                SmartDashboard.putNumber("Module Position 4: ", getRelativeTurnEncoderValue(4));
                return bottomRight.getPosition();
            default:
                return bottomLeft.getPosition();
        }
    }

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

    public void resetDriveMotorEncoders(){
        frontRight.zeroDrivePosition();
        frontLeft.zeroDrivePosition();
        bottomLeft.zeroDrivePosition();
        bottomRight.zeroDrivePosition();
    }

    public void setRelativeTurnEncoderValue() {
        // Uses the absolute encoder value to set relative encoders
        frontRight.setTurnEncoder(frontRight.getAbsoluteEncoderRadians());
        frontLeft.setTurnEncoder(frontLeft.getAbsoluteEncoderRadians());
        bottomLeft.setTurnEncoder(bottomLeft.getAbsoluteEncoderRadians());
        bottomRight.setTurnEncoder(bottomRight.getAbsoluteEncoderRadians());
    }

    @Override
    public void periodic() {
        odometry.update(new Rotation2d(getHeading() * Math.PI / 180),
                new SwerveModulePosition[] { frontRight.getPosition(), frontLeft.getPosition(),
                        bottomLeft.getPosition(), bottomRight.getPosition() });
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
    }

    public void setOdometer(SwerveModulePosition[] modulePositions, Pose2d pose) {
        odometry.resetPosition(new Rotation2d(getHeading() * Math.PI / 180), modulePositions, pose);
    }

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

    public void stopModules() {
        frontRight.stop();
        frontLeft.stop();
        bottomLeft.stop();
        bottomRight.stop();
    }

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

        // convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // set the modules to their desired speeds
        setModules(moduleStates);
    }
}
