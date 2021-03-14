// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransitionConstants;

public class TransitionSubsystem extends SubsystemBase {

  private final WPI_VictorSPX m_transitionMotor = new WPI_VictorSPX(TransitionConstants.kTransitionPort);

  private final WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(TransitionConstants.kIntakePort);

  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void transition(double speed){
    m_transitionMotor.set(speed);
  }

  public void intake(){
    m_intakeMotor.set(TransitionConstants.kRollerSpeed);
  }

  public void outtake(){
    m_intakeMotor.set(-TransitionConstants.kRollerSpeed);
  }
}
