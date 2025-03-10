// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new Wrist. */
  private SparkFlex motor = new SparkFlex(AlgaeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = motor.getClosedLoopController();
  private RelativeEncoder armEncoder = motor.getEncoder();
  private double setPoint = 0;

  public WristSubsystem() {
    motor.configure(
        Configs.wristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    // Zero arm encoder on initialization
        armEncoder.setPosition(0);
  }

  public void setSetPoint(double setPoint)
  {
    this.setPoint = setPoint;
    armController.setReference(setPoint, ControlType.kPosition);
  }

  public boolean isAtPose()
  {
    return Math.abs(armEncoder.getPosition() - setPoint) < 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmSetPoint", this.setPoint);
    SmartDashboard.putBoolean("ArmAtPose", this.isAtPose());
    SmartDashboard.putNumber("Arm Position: ",armEncoder.getPosition());
  }
}
