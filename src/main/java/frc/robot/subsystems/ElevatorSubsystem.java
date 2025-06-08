// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax elevatorMotor1 =
      new SparkMax(CoralSubsystemConstants.kElevatorMotorCanId1, MotorType.kBrushless);
  private SparkMax elevatorMotor2 =
      new SparkMax(CoralSubsystemConstants.kElevatorMotorCanId2, MotorType.kBrushless);
  
  private SparkClosedLoopController elevatorClosedLoopController1 =
      elevatorMotor1.getClosedLoopController();
  private RelativeEncoder elevatorEncoder1 = elevatorMotor1.getEncoder();

  private double setPoint=0;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    elevatorMotor1.configure(
        Configs.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotor2.configure(
          Configs.elevatorFollowerConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
      elevatorEncoder1.setPosition(0);
  }                                          

  public void setSetPoint(double setPoint)
  {
    this.setPoint = setPoint;
    elevatorClosedLoopController1.setReference(setPoint, ControlType.kPosition);
  }

  public boolean isAtPose()
  {
    return Math.abs(elevatorEncoder1.getPosition() - setPoint) < .1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator/Position", elevatorEncoder1.getPosition());
    SmartDashboard.putNumber("ElevatorSetPoint", this.setPoint);
    SmartDashboard.putBoolean("ElevatorAtPose", this.isAtPose());

    // This method will be called once per scheduler run
  }
}
