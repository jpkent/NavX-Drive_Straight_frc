// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.ErrorCode;

public class Robot extends TimedRobot {
  // set up all the constants
  private static final double kP = 0.03;
  private static final double kI = 0.00;
  private static final double kD = 0.00;
  private static final double kToleranceDegrees = 1.0f;

  private static final double inches2Go = 40; // target distance
  private static final double inches2SlowDown = 6; // slow down this far from target
  private static final double wheelDiameter = 6.0;
  private static final double encoderTicksPerRotation = 1440.0; // 360 encode x SRX 4:1
  private static final double drivePower = 0.5;

  // instantiate the motors
  WPI_TalonSRX leftMotor = new WPI_TalonSRX(1);
  WPI_TalonSRX rightMotor = new WPI_TalonSRX(0);
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(3);
  WPI_TalonSRX rightFollower = new WPI_TalonSRX(2);
  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  PIDController turnController;
  AHRS ahrs; // altitude heading reference system aka navigator
  Joystick joy = new Joystick(0);

  private static ErrorCode error; // CTRE specific error codes
  private static double ticks2Go; // how many encode ticks to move
  private static double ticks2SlowDown; // when to slow so you don't overshoot
  private static double drivePower2;

  double inches2Ticks(double inches) {
    return ((inches * encoderTicksPerRotation) / (wheelDiameter * Math.PI));
  }

  @Override
  public void robotInit() {
    if ((error = leftMotor.configFactoryDefault()) != ErrorCode.OK) // factory default the motors
      System.out.println("error setting leftMotor to defaults = " + error);
    if ((error = rightMotor.configFactoryDefault()) != ErrorCode.OK)
      System.out.println("error setting rightMotor to defaults = " + error);
    if ((error = leftFollower.configFactoryDefault()) != ErrorCode.OK)
      System.out.println("error setting leftFollower to defaults = " + error);
    if ((error = rightFollower.configFactoryDefault()) != ErrorCode.OK)
      System.out.println("error setting rightFollower to defaults = " + error);

    leftFollower.follow(leftMotor); // attach followers to leaders
    rightFollower.follow(rightMotor);
    leftMotor.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
    rightMotor.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);
    leftMotor.setSensorPhase(true); // <<<<<< Adjust this
    rightMotor.setSensorPhase(false); // <<<<<< Adjust this

    try { // attempt to instantiate the NavX2. If it throws an exception, catch it and
          // report it.
      ahrs = new AHRS(SPI.Port.kMXP); // SPI is the protocol on the MXP connector that the navigator is plugged into
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX2 MXP:  " + ex.getMessage(), true);
    }
  }

  @Override
  public void robotPeriodic() {
  }

  /**
   * This method is called once at the start of autonomous. It sets up the PID
   * conntroller, the encoder, the encoder end of move count, and the navigator.
   */
  @Override
  public void autonomousInit() {
    turnController = new PIDController(kP, kI, kD); // set up the PID controller
    turnController.enableContinuousInput(-180.0f, 180.0f); // set input range from navigator
    // turnController.setIntegratorRange(-1.0, 1.0); // PID output range to correct
    // steering
    turnController.setTolerance(kToleranceDegrees); // tolerance around set heading to accept
    ahrs.zeroYaw(); // set current heading to zero degrees. Bot might be headed 179 deg and cause
                    // problems
    turnController.setSetpoint(ahrs.getYaw()); // set the desired heading to the current heading (0 degrees)
                                               // This eliminates RIO alignment problems
    ticks2Go = inches2Ticks(inches2Go); // set up encoder stop condition
    ticks2SlowDown = inches2Ticks(inches2SlowDown); // set up encoder slow down condition
    drivePower2 = drivePower; // back to full power for start
    if ((error = leftMotor.setSelectedSensorPosition(0.0)) != ErrorCode.OK) // set encoder to zero
      System.out.println("error setting sensor position to 0 in auto init");
  }

  /**
   * This function is called periodically during autonomous. There are 2 possible
   * conditions. Either you are finished (encoder reading reached ticks2Go), or
   * you are still moving with PID corrections. Maybe add slowing down when close.
   */
  @Override
  public void autonomousPeriodic() {
    drivePower2 = drivePower;
    double position = leftMotor.getSelectedSensorPosition(0);
    if ((ticks2Go + position) < ticks2SlowDown)
      drivePower2 = 0.3; // cut power prepare to stop

    if (position <= -ticks2Go) { // reached desired encoder position - stop
      drive.tankDrive(0.0, 0.0); // stop motors, keep them updated so we don't get a motor safety violation
      System.out.println("finished");
    } else { // move straight
      double rotateToAngleRate = turnController.calculate(ahrs.getYaw(), 0.0); // calc error correction
      drive.tankDrive(-(drivePower2 + rotateToAngleRate), -(drivePower2 - rotateToAngleRate)); // drive with corrected
      // motor powers
      System.out.println("drive power = " + drivePower2 + "  rotToAngleRate = " + rotateToAngleRate
          + "  sensorPosition = " + leftMotor.getSelectedSensorPosition(0) + "  ticksToGo = " + ticks2Go);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double forward = 1.0 * joy.getY(); // Sign this so forward is positive
    double turn = -0.5 * joy.getZ(); // Sign this so right is positive
    drive.arcadeDrive(forward, turn);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
