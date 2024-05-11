package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climb extends SubsystemBase{

    CANSparkMax leftClimb;
    CANSparkMax rightClimb;

    private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    private ShuffleboardLayout list = tab.getLayout("Climb Info", BuiltInLayouts.kList).withSize(3, 2);
    private GenericEntry leftRotations = list.add("Left Rotations", 0.0).getEntry();
    private GenericEntry rightRotations = list.add("Right Rotations", 0.0).getEntry();


    public Climb(){
        leftClimb = new CANSparkMax(ClimbConstants.kLeftClimbMotorID, MotorType.kBrushless);
        rightClimb = new CANSparkMax(ClimbConstants.kRightClimbMotorID, MotorType.kBrushless);
        
        rightClimb.setIdleMode(IdleMode.kBrake);
        leftClimb.setIdleMode(IdleMode.kBrake);
    }

    public void runClimb(double climb1Power, double climb2Power){  
        leftClimb.set(-climb1Power);
        rightClimb.set(climb2Power);
    }


    @Override
    public void periodic() {
        leftRotations.setDouble(leftClimb.getEncoder().getPosition());
        rightRotations.setDouble(rightClimb.getEncoder().getPosition());
    }

}


