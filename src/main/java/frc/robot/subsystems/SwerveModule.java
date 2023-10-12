package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import frc.robot.Constants;

public class SwerveModule{
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private double setPoint;

    private final WPI_CANCoder absoluteEncoder;

    private final PIDController turningPidController;

    private final double absoluteEncoderOffset;
    private final double turningGearRatio;
    private final double drivingGearRatio;
    private final double maxSpeedMPS;
    private final double wheelCircumference;


    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, double absoluteEncoderOffset, int TurnCANCoderId){
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        
        SupplyCurrentLimitConfiguration falconlimit = new SupplyCurrentLimitConfiguration();
        falconlimit.enable = true;
        falconlimit.currentLimit = 20;
        falconlimit.triggerThresholdCurrent = 20;
        falconlimit.triggerThresholdTime = 0;

        driveMotor = new TalonFX(driveMotorId, "drivetrain");
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configSupplyCurrentLimit(falconlimit);

        turnMotor = new TalonFX(turnMotorId, "drivetrain");
        turnMotor.configFactoryDefault();
        turnMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.configSupplyCurrentLimit(falconlimit);


        absoluteEncoder = new WPI_CANCoder(TurnCANCoderId, "drivetrain");

        driveMotor.setInverted(driveMotorReversed);

        turningPidController = new PIDController(0.015, 0.0, 0.0);
        turningPidController.enableContinuousInput(-180, 180); //Tells the PID controller that 180 and -180 are at the same place

        setPoint = 7.0; //This is impossible bcs its in radians so it can't reach that high yk

        //Constants
        turningGearRatio = Constants.Swerve.TURNING_GEAR_RATIO;
        drivingGearRatio = Constants.Swerve.DRIVING_GEAR_RATIO;
        maxSpeedMPS = Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS;
        wheelCircumference = Constants.Swerve.WHEEL_CIRCUMFERENCE_METERS;

    }

    public double getTurnPosition(boolean degrees) {

        if (setPoint == 7){  //Set to 7 in constructor, means relative encoder hasn't been set yet, this is a safety
            return -1;
        }

        double turnDegreeValue = turnMotor.getSelectedSensorPosition() % (2048 * turningGearRatio) / (2048 * turningGearRatio) * 360;

        turnDegreeValue = (setPoint * 180 / Math.PI) + ((setPoint * 180 / Math.PI) - turnDegreeValue); //Adjusts the value cause the falcon encoder is upside down compared to the wheel

        //Binds the value between -180 and 180 degrees
        while (turnDegreeValue > 180) {
            turnDegreeValue -= 360;
        }

        while (turnDegreeValue < -180) {
            turnDegreeValue += 360;
        }

        if (degrees){
            return turnDegreeValue;
        }

        return turnDegreeValue * Math.PI / 180;
    }

    public double falconToRPS(double velocityCounts, double gearRatio) {
        SmartDashboard.putNumber("Velocity Counts", velocityCounts);
        double motorRPS = velocityCounts * (10.0 / 2048.0);        
        double mechRPS = motorRPS / gearRatio * -1;  //multiply by negative 1 bcs the falcon is upside down
        return mechRPS;
    }

    public double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    public double getDriveVelocity() {
        double wheelRPS = falconToRPS(driveMotor.getSelectedSensorVelocity(), drivingGearRatio);
        double wheelMPS = (wheelRPS * wheelCircumference);
        return wheelMPS;
    }

    public double getAbsoluteEncoderRadians() {
        double raw_val = absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffset;

        //Binds the value between -180 and 180 degrees
        while (raw_val > 180) {
            raw_val -= 360;
        }
        while (raw_val < -180) {
            raw_val += 360;
        }

        return raw_val * Math.PI / 180; //Converts to radians
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition(false)));
    }

    public double falconToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    public SwerveModulePosition getPosition() {
        SmartDashboard.putNumber("Drive Velocity", getDriveVelocity());
        return new SwerveModulePosition(
            falconToMeters(driveMotor.getSelectedSensorPosition(), wheelCircumference, drivingGearRatio), 
            getState().angle
        );
        //return new SwerveModulePosition(getDriveVelocity(), getState().angle);
    }

    public void zeroDrivePosition(){
        driveMotor.setSelectedSensorPosition(0);
    }

    public void setDesiredState(SwerveModuleState state, int motor) {
        if (Math.abs(state.speedMetersPerSecond) < 0.1){
           stop();
           return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        if (motor == 3){
            SmartDashboard.putNumber("state", state.angle.getDegrees());
        }
        
        double setpoint = state.angle.getDegrees();
        double turnMotorOutput = -1 * MathUtil.clamp(turningPidController.calculate(getState().angle.getDegrees(), setpoint), -1, 1);
        //Multiply by -1 above because the falcon is upside down compared to the wheel

        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / maxSpeedMPS);
        turnMotor.set(ControlMode.PercentOutput, turnMotorOutput);
    }

    public void stop(){
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setTurnEncoder(double radians) {
        double desired_ticks = radians / Math.PI * (1024 * turningGearRatio);
        turningPidController.reset();
        setPoint = radians;  //Before this line is executed setPoint is equal to 7
        turnMotor.setSelectedSensorPosition(desired_ticks);
    }
    
    public double getDriveTicks(){
        return driveMotor.getSelectedSensorPosition();
    }

}