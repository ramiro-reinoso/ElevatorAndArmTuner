// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

    /** Creates a new ElevatorSubsystem. */
    private TalonFX rightMasterMotor;
    private TalonFX leftSlaveMotor;
  
    TalonFXConfiguration m_configs;
    CurrentLimitsConfigs m_currentConfigs;
  
    private final MotionMagicVoltage m_MagicMotionrequest = new MotionMagicVoltage(0);
  
    public boolean isRunning = false;
    private double m_currentGoal = Constants.initPos;
  
    double currlimit,kG,kS,kV,kA,kP,kI,kD,vel,acc,jerk;

      private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.kDriverControllerPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Set the initial values for tuning
    setInitValues();

    // Initialize the elevator motors
    // initElevator();
  }


  private void setInitValues()
  {
    currlimit = Constants.maxStatorCurrent;
    kG = Constants.pidkG;
    kS = Constants.pidkS;
    kV = Constants.pidkV;
    kA = Constants.pidkA;
    kP = Constants.pidkP;
    kI = Constants.pidkI;
    kD = Constants.pidkD;
    vel = Constants.MMVelocity;
    acc = Constants.MMAcceleration;
    jerk = Constants.MMJerk;
  }

  private void initElevator() {
    rightMasterMotor = new TalonFX(13,"drivetrain");
    leftSlaveMotor = new TalonFX(14,"drivetrain");

    // Set the configuration of the motor
    m_configs = new TalonFXConfiguration();
    m_currentConfigs = m_configs.CurrentLimits;
    m_currentConfigs.StatorCurrentLimit = currlimit;  // Stator current limit
    m_currentConfigs.StatorCurrentLimitEnable = true; // Start with stator limits on

    // set slot 0 gains
    var slot0Configs = m_configs.Slot0;
    slot0Configs.kG = kG;  // Voltage output to overcome gravity without a coral
    slot0Configs.kS = kS; // Voltage output to overcome static friction
    slot0Configs.kV = kV; // A velocity target of 1 rps requires this voltage output.
    slot0Configs.kA = kA; // An acceleration of 1 rps/s requires this voltage output
    slot0Configs.kP = kP; // A position error of 2.5 rotations requires this voltage output
    slot0Configs.kI = kI; // no output for integrated error
    slot0Configs.kD = kD; // A velocity error of 1 rps requires this voltage output

    // set Motion Magic settings
    var motionMagicConfigs = m_configs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = vel; // Target cruise velocity in revolutions per second (rps)
    motionMagicConfigs.MotionMagicAcceleration = acc; // Target acceleration in rps/sec
    motionMagicConfigs.MotionMagicJerk = jerk; // Target jerk in rps/s/s

    // Set neutral mode to brake
    m_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configuration to the motors
    rightMasterMotor.getConfigurator().apply(m_configs);
    leftSlaveMotor.getConfigurator().apply(m_configs);

/*     
    // No longer necessary
    rightMasterMotor.setNeutralMode(NeutralModeValue.Brake);
    leftSlaveMotor.setNeutralMode(NeutralModeValue.Brake);
 */

    // Make the left motor a slave of the right motor.  The last parameter indicates whether the slave motor
    // direction will be inverted.
    leftSlaveMotor.setControl(new Follower(rightMasterMotor.getDeviceID(), false));

    setZero();  // Zero the elevator at the bottom.
  }

  public void setZero(){

    // Run the motors slowly to lower the elevator.
    rightMasterMotor.setVoltage(-0.2);

    while(rightMasterMotor.getStatorCurrent().getValueAsDouble() < 10.0){
      var angularVelocity = Math.abs(rightMasterMotor.getRotorVelocity().getValueAsDouble());
      if(angularVelocity < 0.1) { // Check if the motor stopped turning (end of travel)
        break;    // If motor stopped running, then exit the loop.
      }
      System.out.println("Setting the elevator to zero. Stator current: " + rightMasterMotor.getStatorCurrent().getValueAsDouble()
                          + " Rotor Angular Velocity: " + rightMasterMotor.getRotorVelocity().getValueAsDouble());
    }

    rightMasterMotor.setVoltage(0.0);
    rightMasterMotor.setPosition(0.0);  // TO-DO.  May want to have this a little lower to avoid hitting the bottom hard when goal is set to zero.
  }

 
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("Current", currlimit);
    SmartDashboard.putNumber("kG", kG);
    SmartDashboard.putNumber("kS", kS);
    SmartDashboard.putNumber("kV", kV);
    SmartDashboard.putNumber("kA", kA);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("MM Velocity", vel);
    SmartDashboard.putNumber("MM Acceleration", acc);
    SmartDashboard.putNumber("MM Jerk", jerk);
    SmartDashboard.putNumber("Position", m_currentGoal);

    counter = 0;
  }

  private boolean setParams, setPos;
  private boolean keyPressed=false;
  private int counter;
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    setParams = false;
    setPos = false;

    var newcurrlimit = SmartDashboard.getNumber("Current", currlimit);
    if(areDifferent(newcurrlimit, currlimit)) { currlimit = newcurrlimit; setParams = true;}

    var newkG = SmartDashboard.getNumber("kG", kG);
    if(areDifferent(newkG, kG)) { kG = newkG; setParams = true;}  

    var newkS = SmartDashboard.getNumber("kS", kS);
    if(areDifferent(newkS, kS)) { currlimit = newcurrlimit; setParams = true;}

    var newkV = SmartDashboard.getNumber("kV", kV);
    if(areDifferent(newkV, kV)) { kV = newkV; setParams = true;}

    var newkA = SmartDashboard.getNumber("kA", kA);
    if(areDifferent(newkA, kA)) { kA = newkA; setParams = true;}

    var newkP = SmartDashboard.getNumber("kP", kP);
    if(areDifferent(newkP, kP)) { kP = newkP; setParams = true;}

    var newkI = SmartDashboard.getNumber("kI", kI);
    if(areDifferent(newkI, kI)) { kI = newkI; setParams = true;}

    var newkD = SmartDashboard.getNumber("kD", kD);
    if(areDifferent(newkD, kD)) { kD = newkD; setParams = true;}

    var newvel = SmartDashboard.getNumber("MM Velocity", vel);
    if(areDifferent(newvel, vel)) { vel = newvel; setParams = true;}

    var newacc = SmartDashboard.getNumber("MM Acceleration", acc);
    if(areDifferent(newacc, acc)) { acc = newacc; setParams = true;}

    var newjerk = SmartDashboard.getNumber("MM Jerk", jerk);
    if(areDifferent(newjerk, jerk)) { jerk = newjerk; setParams = true;}

    var newdesiredPos = SmartDashboard.getNumber("Position", m_currentGoal);
    if(areDifferent(newdesiredPos, m_currentGoal)) { m_currentGoal = newdesiredPos; setPos = true;}

    if(setParams)
    {
      System.out.println("Resetting Parameters");
      //resetParams();
    }

    if(setPos)
    {
      System.out.println("Request to setup New Position");
      if((m_currentGoal > 0) && (m_currentGoal < 40))
      {
        System.out.println("Setting Up New Position");
        // rightMasterMotor.setControl(m_MagicMotionrequest.withPosition(m_currentGoal));
      }
      else {
        System.out.println("New goal is out of range.  Try again.");
      }
    }

/*     if(m_driverController.a().getAsBoolean() && !keyPressed) {
      System.out.println("Button A was pressed");
      keyPressed = true;
    } else if (!m_driverController.a().getAsBoolean())
    { keyPressed = false;} */
  }

  private boolean areDifferent(double one, double two)
  {
    if (Math.abs(one - two) < 0.0001){
      return false;
    }
    else {
      System.out.println("Values are different");
      return true;
    }
  }

  private void resetParams()
  {
    m_currentConfigs.StatorCurrentLimit = currlimit;  // Stator current limit

    // set slot 0 gains
    var slot0Configs = m_configs.Slot0;
    slot0Configs.kG = kG;  // Voltage output to overcome gravity without a coral
    slot0Configs.kS = kS; // Voltage output to overcome static friction
    slot0Configs.kV = kV; // A velocity target of 1 rps requires this voltage output.
    slot0Configs.kA = kA; // An acceleration of 1 rps/s requires this voltage output
    slot0Configs.kP = kP; // A position error of 2.5 rotations requires this voltage output
    slot0Configs.kI = kI; // no output for integrated error
    slot0Configs.kD = kD; // A velocity error of 1 rps requires this voltage output

    // set Motion Magic settings
    var motionMagicConfigs = m_configs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = vel; // Target cruise velocity in revolutions per second (rps)
    motionMagicConfigs.MotionMagicAcceleration = acc; // Target acceleration in rps/sec
    motionMagicConfigs.MotionMagicJerk = jerk; // Target jerk in rps/s/s
   
    // Apply the configuration to the motors
    rightMasterMotor.getConfigurator().apply(m_configs);
    leftSlaveMotor.getConfigurator().apply(m_configs);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
