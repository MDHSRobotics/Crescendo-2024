package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class Climb extends SubsystemBase{

    private CANSparkMax leftClimb;
    private CANSparkMax rightClimb;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private DigitalInput m_leftlimitSwitch = new DigitalInput(ClimbConstants.kLeftLimitSwitchID);
    private DigitalInput m_rightlimitSwitch = new DigitalInput(ClimbConstants.kRightLimitSwitchID);

    /* Shuffleboard Logging */
    private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    // Lowest value is -225
    private GenericEntry leftRotations = tab.add("Left Rotations", 0.0).getEntry();
    // Highest value is 225
    private GenericEntry rightRotations = tab.add("Right Rotations", 0.0).getEntry();

    public Climb(){
        leftClimb = new CANSparkMax(ClimbConstants.kLeftClimbMotorID, MotorType.kBrushless);
        rightClimb = new CANSparkMax(ClimbConstants.kRightClimbMotorID, MotorType.kBrushless);

        leftEncoder = leftClimb.getEncoder();
        rightEncoder = rightClimb.getEncoder();
        
        rightClimb.setIdleMode(IdleMode.kBrake);
        leftClimb.setIdleMode(IdleMode.kBrake);

        // Right Climb Spark Max was the only controller with too high of a current limit. You can connect to it and change it, but we just put it here for now.
        rightClimb.setSmartCurrentLimit(40);

        // CAN optimization: https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames
        for (int i = 0; i < 8; ++i) {
            PeriodicFrame frame = PeriodicFrame.fromId(i);
            leftClimb.setPeriodicFramePeriod(frame, 500);
            rightClimb.setPeriodicFramePeriod(frame, 500);
        }
    }

    public void runClimb(double climb1Power, double climb2Power){  
        leftClimb.set(-climb1Power);
        rightClimb.set(climb2Power);
    }

    /**
     * @return True if at least one of the switches is pressed, false if both switches are unpressed
     */
    public boolean getLimitSwitches() {
        return m_leftlimitSwitch.get() || m_rightlimitSwitch.get();
    }

    public void logData() {
        leftRotations.setDouble(leftEncoder.getPosition());
        rightRotations.setDouble(rightEncoder.getPosition());
    }
}


