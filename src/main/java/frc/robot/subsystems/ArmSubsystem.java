

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;;

public class ArmSubsystem extends SubsystemBase {
  private final SparkFlex left;
  private final SparkFlex right;


  public ArmSubsystem() {
    left = new SparkFlex(Arm.arm_left, MotorType.kBrushless);
    right = new SparkFlex(Arm.arm_right, MotorType.kBrushless);
    //Add configations later

  }

    public void set_speed(){
    left.set(Arm.left_speed);
    right.set(Arm.right_speed);
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
