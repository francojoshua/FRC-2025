package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.Elevator;


public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private SparkClosedLoopController controller;
  private SparkMaxConfig config;

  public ElevatorSubsystem() {
      motor = new SparkMax(Elevator.elevator_motor, MotorType.kBrushless);
      controller = motor.getClosedLoopController();
      config = new SparkMaxConfig();
      config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      config.closedLoop.pid(Elevator.KP,0.0,Elevator.KD);
      config.closedLoop.outputRange(Elevator.Kmin, Elevator.kmax);
      config.encoder.positionConversionFactor(Elevator.positionConversionFactor);

      motor.configure(config, null, PersistMode.kPersistParameters);
  }

    public void set_speed(){
    motor.set(Elevator.speed);
  }

  public void setmax_height(){
    controller.setReference(Elevator.level_4,ControlType.kPosition,null,Elevator.KF);
  }
  public void stop(){
    motor.stopMotor();
  }

  public void hold(){
    motor.set(Elevator.KF);
  }

  




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
