// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class RomiDrivetrain extends SubsystemBase 
{
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  //Sets up Controller
  Joystick joy1 = new Joystick(0);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() 
  {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void resetEncoders() 
  {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() 
  {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() 
  {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() 
  {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance())/2;
  }

  public void mainDrive()
  {
    double speed = -joy1.getRawAxis(0)*0.6;
    double turn = joy1.getRawAxis(1)*0.3;

    double lfValue = speed + turn;
    double rfValue = speed - turn;

    m_leftMotor.set(lfValue);
    m_rightMotor.set(rfValue);
  }

  public void forward(double distance)
  {
    if(getAverageDistanceInch() < distance)
    {
      m_leftMotor.set(0.6);
      m_rightMotor.set(0.6);
    }
    else
    {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    }
  }

  public void turn(double angle)
  {
    if(getLeftDistanceInch() < (angle/360)*6.5)
    {
      m_leftMotor.set(0.5);
      m_rightMotor.set(-0.5);
    }
    else
    {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    }
  }


  @Override
  public void periodic() 
  {
    double time = Timer.getFPGATimestamp();

    turn(90);
  }

  @Override
  public void simulationPeriodic() 
  {
    //mainDrive();
  }
}
  
