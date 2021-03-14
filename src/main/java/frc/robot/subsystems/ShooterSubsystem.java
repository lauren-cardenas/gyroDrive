// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final WPI_TalonFX m_leftShooter = new WPI_TalonFX(ShooterConstants.kLeftShooterPort);
  private final WPI_TalonFX m_rightShooter = new WPI_TalonFX(ShooterConstants.kRightShooterPort);

  private final DifferentialDrive m_shooter = new DifferentialDrive(m_leftShooter, m_rightShooter);

  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double leftVelocity(){
    return m_leftShooter.getSelectedSensorVelocity();
  }

  public double rightVelocity(){
    return m_rightShooter.getSelectedSensorVelocity();
  }

  public void shoot(double speed){
    m_shooter.arcadeDrive(speed, 0.0, false);
  }
}
