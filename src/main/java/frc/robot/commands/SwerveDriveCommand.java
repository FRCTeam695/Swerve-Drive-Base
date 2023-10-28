// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveSubsystem m_Subsystem;
  private final DoubleSupplier xSpeed, ySpeed, turningSpeed;
  private final boolean fieldOriented;
  private final PIDController thetaController;
  private double previousAngle; //-180 to 180
  private double previousZj; // -1 to 1


  public SwerveDriveCommand(SwerveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier turningSpeed, boolean fieldOriented) {
    this.m_Subsystem = subsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.turningSpeed = turningSpeed;
    this.fieldOriented = fieldOriented;
    previousZj = turningSpeed.getAsDouble();

    m_Subsystem.startTickCount();
    thetaController = new PIDController(0.5, 0, 0);

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousAngle = m_Subsystem.getHeading();
    m_Subsystem.setRelativeTurnEncoderValue();  //Uses the absolute encoder value to set the relative encoders
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(DriverStation.isAutonomous()){
      return;
    }
    
    //Gets values from double suppliers
    Double Xj = -1 * xSpeed.getAsDouble(); //Inverted because WPIlib coordinate system is weird, link to docs below
    Double Yj = -1 * ySpeed.getAsDouble(); //The controller is inverted
    Double Zj = -1 * turningSpeed.getAsDouble(); //Inverted because WPIlib coordinate system is weird, link to docs below

    //Accounts for robot drift
    double deltaZj = previousZj - Zj;

    //if((Math.abs(deltaZj) < 0.1) && (Zj < 0.01)){
    //  Zj = MathUtil.clamp(thetaController.calculate(m_Subsystem.getHeading(), previousAngle), -1, 1);
    //}

    //Update Z and gyro values
    previousAngle = m_Subsystem.getHeading();
    previousZj = Zj;


    m_Subsystem.driveSwerve(Xj, Zj, Yj, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
