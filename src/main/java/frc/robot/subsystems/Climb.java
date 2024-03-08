package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climb extends SubsystemBase{

    CANSparkMax leftClimb;
    CANSparkMax rightClimb;

    private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    private GenericEntry leftClimbRotations =
      tab.add("Left Climb Rotations", 0)
         .getEntry();
    private GenericEntry rightClimbRotations =
      tab.add("Right Climb Rotations", 0)
         .getEntry();


    public Climb(){
        leftClimb = new CANSparkMax(ClimbConstants.kLeftClimbMotorID, MotorType.kBrushless);
        rightClimb = new CANSparkMax(ClimbConstants.kRightClimbMotorID, MotorType.kBrushless);
 
    }

    public void runMotors(double climb1Power, double climb2Power){
        double lep = leftClimb.getEncoder().getPosition();
        double rep = rightClimb.getEncoder().getPosition();

        leftClimbRotations.setDouble(lep);
        rightClimbRotations.setDouble(rep);
        if (lep < ClimbConstants.leftTopLimit && lep > ClimbConstants.leftBottomLimit){
            leftClimb.set(-climb1Power);
        }
        if (rep < ClimbConstants.rightTopLimit && rep > ClimbConstants.rightBottomLimit){
            rightClimb.set(climb2Power);
        }
        rightClimb.set(climb2Power);
        SmartDashboard.putNumber("Climb Power", climb1Power);
    }

}


