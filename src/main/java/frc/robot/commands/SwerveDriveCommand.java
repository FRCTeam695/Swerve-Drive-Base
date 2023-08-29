// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;
import frc.robot.Constants;

public class SwerveDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveSubsystem m_Subsystem;
  private final DoubleSupplier xSpeed, ySpeed, turningSpeed;
  private final boolean feildOriented;


  public SwerveDriveCommand(SwerveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier turningSpeed, boolean feildOriented) {
    this.m_Subsystem = subsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.turningSpeed = turningSpeed;
    this.feildOriented = feildOriented;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Subsystem.setRelativeTurnEncoderValue(1);  //Uses the absolute encoder value to set the relative encoders
    m_Subsystem.setRelativeTurnEncoderValue(2);
    m_Subsystem.setRelativeTurnEncoderValue(3);
    m_Subsystem.setRelativeTurnEncoderValue(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Gets values from double suppliers
    Double Xj = -1 * xSpeed.getAsDouble(); //Inverted because WPIlib coordinate system is weird, link to docs below
    Double Yj = -1 * ySpeed.getAsDouble(); //The controller is inverted
    Double Zj = -1 * turningSpeed.getAsDouble(); //Inverted because WPIlib coordinate system is weird, link to docs below

    //Check "Robot Coordinate System" https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
    //It is the same logic as the SwerveDriveKinematics object created in Constants.java, the (positive, positive) quadrant is in the top left

    //Deadband
    Xj = Math.abs(Xj) > 0.15 ? Xj : 0;
    Yj = Math.abs(Yj) > 0.15 ? Yj : 0;
    Zj = Math.abs(Zj) > 0.15 ? Zj : 0;


    //Scale up the speeds, WPILib likes them in meters per second
    Xj = Xj * Constants.MAX_SPEED_METERS_PER_SECONDS;
    Yj = Yj * Constants.MAX_SPEED_METERS_PER_SECONDS;
    Zj = Zj * Constants.MAX_ANGULAR_SPEED_METERS_PER_SECOND;


    //construct chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (feildOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(Yj, Xj, Zj, Rotation2d.fromDegrees(m_Subsystem.getHeading()));
    }else{
      chassisSpeeds = new ChassisSpeeds(Yj, Xj, Zj);
    }


    //convert chassis speeds to module states
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  

    //set the modules to their desired speeds
    m_Subsystem.setModules(moduleStates);
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
