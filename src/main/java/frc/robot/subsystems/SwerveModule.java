package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

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

    private final double WHEEL_CIRCUMFERENCE_METERS;
    private final double TURNING_GEAR_RATIO;
    private final double DRIVING_GEAR_RATIO;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, double absoluteEncoderOffset, int TurnCANCoderId){
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        driveMotor = new TalonFX(driveMotorId, "drivetrain");
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);

        turnMotor = new TalonFX(turnMotorId, "drivetrain");
        turnMotor.configFactoryDefault();
        turnMotor.setNeutralMode(NeutralMode.Brake);


        absoluteEncoder = new WPI_CANCoder(TurnCANCoderId, "drivetrain");

        driveMotor.setInverted(driveMotorReversed);

        turningPidController = new PIDController(0.015, 0.0, 0.0);
        turningPidController.enableContinuousInput(-180, 180); //Tells the PID controller that 180 and -180 are at the same place

        WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(4 * Math.PI);
        TURNING_GEAR_RATIO = 150.0/7;
        DRIVING_GEAR_RATIO = 6.12;
        setPoint = 7; //This is impossible bcs its in radians so it can't reach that high yk

    }

    public double getTurnPosition(boolean degrees) {

        if (setPoint == 7){  //Set to 7 in constructor, means relative encoder hasn't been set yet, this is a safety
            return -1;
        }

        double turnDegreeValue = turnMotor.getSelectedSensorPosition() % (2048 * TURNING_GEAR_RATIO) / (2048 * TURNING_GEAR_RATIO) * 360;

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

    public double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    public double getDriveVelocity() {
        double wheelRPM = falconToRPM(driveMotor.getSelectedSensorVelocity(), DRIVING_GEAR_RATIO);
        double wheelMPS = (wheelRPM * WHEEL_CIRCUMFERENCE_METERS) / 60;
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

    public void setDesiredState(SwerveModuleState state, int motor) {
        if (Math.abs(state.speedMetersPerSecond) < 0.1){
           stop();
           return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        
        double setpoint = state.angle.getDegrees();
        double turnMotorOutput = -1 * MathUtil.clamp(turningPidController.calculate(getState().angle.getDegrees(), setpoint), -1, 1);
        //Multiply by -1 above because the falcon is upside down compared to the wheel

        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.MAX_SPEED_METERS_PER_SECONDS);
        turnMotor.set(ControlMode.PercentOutput, turnMotorOutput);
    }

    public void stop(){
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setTurnEncoder(int motor, double radians) {
        double desired_ticks = radians / Math.PI * (1024 * TURNING_GEAR_RATIO);
        turningPidController.reset();
        setPoint = radians;  //Before this line is executed setPoint is equal to 7
        turnMotor.setSelectedSensorPosition(desired_ticks);
    }

}