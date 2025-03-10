// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorStateMachine extends Command {
  
  public enum ElevatorState {
    L0,
    L1,
    L2,
    L3,
    FS,
  }  

  ElevatorState currentState;

  double elevatorSetpoint;
  double wristSetpoint;

  ElevatorSubsystem elevatorSubsystem;
  WristSubsystem wristSubsystem;

  /** Creates a new ElevatorStateMachine. */
  public ElevatorStateMachine(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    currentState = ElevatorState.L1;

    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;

    addRequirements(elevatorSubsystem, wristSubsystem); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SWITCH STATEMENT
    switch (currentState) {
      case L0:
        elevatorSetpoint = Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel0;
        wristSetpoint = Constants.CoralSubsystemConstants.ArmSetpoints.kLevel0;
        break;
      case L1:
        elevatorSetpoint = Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel1;
        wristSetpoint = Constants.CoralSubsystemConstants.ArmSetpoints.kLevel6;
        //wristSetpoint = Constants.CoralSubsystemConstants.ArmSetpoints.kLevel1;
        break;
      case L2:
        elevatorSetpoint = Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel2;
        wristSetpoint = Constants.CoralSubsystemConstants.ArmSetpoints.kLevel5;
        //wristSetpoint = Constants.CoralSubsystemConstants.ArmSetpoints.kLevel2;
        break;
      case L3:
        elevatorSetpoint = Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel3;
        wristSetpoint = Constants.CoralSubsystemConstants.ArmSetpoints.kLevel4;
        //wristSetpoint = Constants.CoralSubsystemConstants.ArmSetpoints.kLevel3;
        break;
      case FS:
        elevatorSetpoint = Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel0;
        wristSetpoint = Constants.CoralSubsystemConstants.ArmSetpoints.kFeederStation;
        break;

      default:

      break;
    }

    //SET ELEVATORS TO SETPOINT
    elevatorSubsystem.setSetPoint(elevatorSetpoint);
    wristSubsystem.setSetPoint(wristSetpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setSetpoint(ElevatorState state) {
    currentState = state;
  }

}
