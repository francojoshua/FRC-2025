package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.Intake;;

public class IntakeSubsystem extends SubsystemBase {
  
  private final SparkFlex left;
  private final SparkFlex right;

  


  public IntakeSubsystem() {

    left = new SparkFlex(Intake.intake_left, MotorType.kBrushless);
    right = new SparkFlex(Intake.intake_right, MotorType.kBrushless);
    //Add configations later


  }

  public void set_speed(){
    left.set(Intake.left_speed);
    right.set(Intake.right_speed);
  }

  public void stop(){
    left.stopMotor();
    right.stopMotor();
  }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
