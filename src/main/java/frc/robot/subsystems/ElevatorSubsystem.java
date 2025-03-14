package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.Elevator;


public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private SparkClosedLoopController controller;
  private RelativeEncoder encoder;
  private SparkMaxConfig config;
  private double setpoint;
  private double temp;

  public ElevatorSubsystem() {
      motor = new SparkMax(Elevator.elevator_motor, MotorType.kBrushless);
      encoder = motor.getEncoder();
      controller = motor.getClosedLoopController();
      config = new SparkMaxConfig();
      config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      config.closedLoop.pid(Elevator.KP,0.0,Elevator.KD);
      config.closedLoop.outputRange(Elevator.output_minimum, Elevator.output_maximum);
      config.encoder.positionConversionFactor(Elevator.positionConversionFactor);
    
      motor.configure(config, null, PersistMode.kPersistParameters);

      setpoint = 0.0;
  }


  public void setLevel(){
    controller.setReference(this.setpoint,ControlType.kPosition,null,Elevator.KF);
  }

  public void change_setpoint(double setpoint){
    this.setpoint = setpoint;
  }

  public void increment_setpoint(){
    temp = this.setpoint;
    temp += Elevator.increment_decrement;

    if(temp > Elevator.height_maximum){
        temp = Elevator.height_maximum;
    } 

    this.setpoint = temp;
  }
  
  public void decrement_setpoint(){
    temp = this.setpoint;
    temp -= Elevator.increment_decrement;


    if(temp < Elevator.height_minmum){
        temp = Elevator.height_minmum;
    } 

    this.setpoint = temp;
  }

  public double get_positon(){
    double position = encoder.getPosition();
    return position;
  }





  public void hold(){
    motor.set(Elevator.KF);
  }

  public void set_speed(double speed){
    motor.set(speed);
  }

  public void stop(){
    motor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
