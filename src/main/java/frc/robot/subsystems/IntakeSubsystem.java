// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkFlex motor = new SparkFlex(AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
 
 
  public IntakeSubsystem() {
        motor.configure(
        Configs.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }


  public void setPower(double power) {
    motor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
