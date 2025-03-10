package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
      public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig wristConfig = new SparkFlexConfig();
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();    

    static {
      // Configure basic setting of the arm motor
      wristConfig
      .smartCurrentLimit(40)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.1)
          .outputRange(-0.5, 0.5);

      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

      // Configure basic settings of the elevator motor
      elevatorConfig
      .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .closedLoop
          .outputRange(-0.5, 0.5)
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.1);
          //.pid(0.0012199999764561653, 0.0, 0.0);
        elevatorFollowerConfig
        .follow(Constants.CoralSubsystemConstants.kElevatorMotorCanId1,true);
    }
  }