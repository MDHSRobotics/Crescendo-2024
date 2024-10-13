package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private CANSparkFlex intake;
    private CANSparkMax conveyor;
    private CANSparkMax leftAngle;
    private CANSparkMax rightAngle;

    private SparkPIDController m_pidController;

    private RelativeEncoder m_leftAngleEncoder;
    private RelativeEncoder m_rightAngleEncoder;

    /* Shuffleboard Logging */
    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    private ShuffleboardLayout list = tab.getLayout("Intake Info", BuiltInLayouts.kList).withSize(3, 4);
    private GenericEntry angleRotations = list.add("Angle Rotations", 0.0).getEntry();
    private GenericEntry intakeSpeed = list.add("Intake Speed", 0.0).getEntry();
    private GenericEntry conveyorSpeed = list.add("Conveyer Speed", 0.0).getEntry();

    //private GenericEntry backTx = tab.add("Back Limelight TX", 0.0).getEntry();
    //private GenericEntry backTy = tab.add("Back Limelight TY", 0.0).getEntry();

    private GenericEntry intakeTopRotations = tab.addPersistent("Top Rotations", 0.0)
        .withSize(2, 1)
        .getEntry();
    private GenericEntry intakeBottomRotations = tab.addPersistent("Bottom Rotations", 34.0)
        .withSize(2, 1)
        .getEntry();
    private GenericEntry intakeMidRotations = tab.addPersistent("Mid Rotations", 33.0)
        .withSize(2, 1)
        .getEntry();
    private GenericEntry intakeEjectRotations = tab.addPersistent("Eject Rotations", 30.0)
        .withSize(2, 1)
        .getEntry();
    public GenericEntry intakeAmpRotations = tab.addPersistent("Amp Rotations", 1.0)
        .withSize(2, 1)
        .getEntry();
    public GenericEntry ampFastSpitPower = tab.addPersistent("Amp Fast Spit Power", -0.2)
        .withSize(3, 1)
        .getEntry();    
    

    public Intake(){
        intake = new CANSparkFlex(IntakeConstants.kIntakeID, MotorType.kBrushless);
        conveyor = new CANSparkMax(IntakeConstants.kConveyorID, MotorType.kBrushless);
        leftAngle = new CANSparkMax(IntakeConstants.kLeftAngleID, MotorType.kBrushless);
        rightAngle = new CANSparkMax(IntakeConstants.kRightAngleID, MotorType.kBrushless);

        m_leftAngleEncoder = leftAngle.getEncoder();
        m_rightAngleEncoder = rightAngle.getEncoder();

        rightAngle.setIdleMode(IdleMode.kBrake);
        leftAngle.setIdleMode(IdleMode.kBrake);
        
        m_pidController = rightAngle.getPIDController();
        m_pidController.setP(0.1);

        // CAN optimization: https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        conveyor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        conveyor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        leftAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        rightAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        rightAngle.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        

        leftAngle.setOpenLoopRampRate(0.1);
        rightAngle.setOpenLoopRampRate(0.1);
        intake.setOpenLoopRampRate(0.1);
        conveyor.setOpenLoopRampRate(0.1);

        leftAngle.follow(rightAngle, true);
        
        conveyor.setInverted(true);
    }

    public void runIntake(double intakeSpeed, double conveyerSpeed) {
        intake.set(intakeSpeed);
        conveyor.set(conveyerSpeed);
    }

    public void ampFastSpit() {
        intake.set(ampFastSpitPower.getDouble(0));
        conveyor.set(ampFastSpitPower.getDouble(0));
    }

    public void rotateIntake(double angleSpeed) {
        rightAngle.set(angleSpeed);
    }

    public void topPosition(){
        m_pidController.setReference(intakeTopRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
    }

    public void bottomPosition(){
        m_pidController.setReference(intakeBottomRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
    }

    public void midPosition(){
        m_pidController.setReference(intakeMidRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
    }

    public void ejectPosition() {
        m_pidController.setReference(intakeEjectRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
    }

    public void ampPosition() {
        m_pidController.setReference(intakeAmpRotations.getDouble(0), CANSparkMax.ControlType.kPosition);
    }
    
    
    public void resetEncoders(){
        m_leftAngleEncoder.setPosition(0);
        m_rightAngleEncoder.setPosition(0);
    }

    public boolean noteInSight() {
        boolean noteIsInSight = LimelightHelpers.getTX("limelight-back") != 0;
        return noteIsInSight;
    }

    /** Shuffleboard logging. We avoid overriding periodic() because it runs even when the robot is disabled. */
    public void logData() {
        // Subsystem data
        angleRotations.setDouble(rightAngle.getEncoder().getPosition());
        intakeSpeed.setDouble(intake.get());
        conveyorSpeed.setDouble(conveyor.get());
        //backTx.setDouble(LimelightHelpers.getTX("limelight-back"));
        //backTy.setDouble(LimelightHelpers.getTY("limelight-back"));
    }
}
