/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.usfirst.frc.team492.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DMC60;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SD540;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * This is a generic autonomous program that drives the robot with the specified power for a set amount of time.
 * It works with most of the 2018 legal motor controllers and 2 or 4-motor drive configured robots.
 * To use this program, you must go through each of the "TODO" sections and select the appropriate configurations
 * and numbers.
 */
public class Robot extends IterativeRobot
{
    //
    // TODO: Need to select drive type.
    //
    private static final boolean USE_DIFFERENTIAL_DRIVE = true;
    private static final boolean USE_MECANUM_DRIVE = false;
    //
    // TODO: Need to select motor controller type.
    //
    private static final boolean USE_DMC60 = false;
    private static final boolean USE_JAGUAR = false;
    private static final boolean USE_SD540 = false;
    private static final boolean USE_SPARK = true;
    private static final boolean USE_TALON = false;
    private static final boolean USE_TALONSRX = false;
    private static final boolean USE_VICTOR = false;
    private static final boolean USE_VICTORSP = false;
    private static final boolean USE_VICTORSPX = false;
    //
    // TODO: Need to update the channel numbers.
    //       Depending on the motor controller type, these could be PWM channels or CAN IDs.
    //
    private static final int LEFT_CHANNEL = 0;
    private static final int RIGHT_CHANNEL = 1;

    private static final int FRONT_LEFT_CHANNEL = 0;
    private static final int REAR_LEFT_CHANNEL = 1;
    private static final int FRONT_RIGHT_CHANNEL = 2;
    private static final int REAR_RIGHT_CHANNEL = 3;
    //
    // TODO: Need to determine if motors need to be inverted.
    //
    private static final boolean LEFT_MOTOR_INVERTED = true;
    private static final boolean RIGHT_MOTOR_INVERTED = false;
    //
    // TODO: Need to tune the following numbers.
    //
    private static final double DRIVE_TIME_IN_SEC = 3.0;
    private static final double LEFT_DRIVE_POWER = 0.5;
    private static final double RIGHT_DRIVE_POWER = 0.5;
    private static final double X_DRIVE_POWER = 0.0;
    private static final double Y_DRIVE_POWER = 0.5;
    private static final double ROTATE_POWER = 0.0;

    private DifferentialDrive myDifferentialRobot = null;
    private MecanumDrive myMecanumRobot = null;
    private double stopTime;

    /**
     * This method is called one time when the program starts to initialize the robot. It creates and initializes
     * the drive motors and the drive base objects.
     */
    @Override
    public void robotInit()
    {
        if (USE_DIFFERENTIAL_DRIVE)
        {
            SpeedController leftMotor = null, rightMotor = null;
            //
            // Create a 2-motor drive base with the selected motor controllers.
            //
            if (USE_DMC60)
            {
                leftMotor = new DMC60(LEFT_CHANNEL);
                rightMotor = new DMC60(RIGHT_CHANNEL);
            }
            else if (USE_JAGUAR)
            {
                leftMotor = new Jaguar(LEFT_CHANNEL);
                rightMotor = new Jaguar(RIGHT_CHANNEL);
            }
            else if (USE_SD540)
            {
                leftMotor = new SD540(LEFT_CHANNEL);
                rightMotor = new SD540(RIGHT_CHANNEL);
            }
            else if (USE_SPARK)
            {
                leftMotor = new Spark(LEFT_CHANNEL);
                rightMotor = new Spark(RIGHT_CHANNEL);
            }
            else if (USE_TALON)
            {
                leftMotor = new Talon(LEFT_CHANNEL);
                rightMotor = new Talon(RIGHT_CHANNEL);
            }
            else if (USE_TALONSRX)
            {
                leftMotor = new WPI_TalonSRX(LEFT_CHANNEL);
                rightMotor = new WPI_TalonSRX(RIGHT_CHANNEL);
            }
            else if (USE_VICTOR)
            {
                leftMotor = new Victor(LEFT_CHANNEL);
                rightMotor = new Victor(RIGHT_CHANNEL);
            }
            else if (USE_VICTORSP)
            {
                leftMotor = new VictorSP(LEFT_CHANNEL);
                rightMotor = new VictorSP(RIGHT_CHANNEL);
            }
            else if (USE_VICTORSPX)
            {
                leftMotor = new WPI_VictorSPX(LEFT_CHANNEL);
                rightMotor = new WPI_VictorSPX(RIGHT_CHANNEL);
            }
            //
            // If no motor controller type is selected, left and right motors will be null.
            //
            try
            {
                //
                // Configure the spin direction of the motors.
                //
                leftMotor.setInverted(LEFT_MOTOR_INVERTED);
                rightMotor.setInverted(RIGHT_MOTOR_INVERTED);
                myDifferentialRobot = new DifferentialDrive(leftMotor, rightMotor);
            }
            catch (NullPointerException e)
            {
                throw new IllegalArgumentException("You must select one of the legal motor controller types.");
            }
        }
        else if (USE_MECANUM_DRIVE)
        {
            SpeedController frontLeftMotor = null, rearLeftMotor = null, frontRightMotor = null, rearRightMotor = null;
            //
            // Create a 4-motor drive base with the selected motor controllers.
            //
            if (USE_DMC60)
            {
                frontLeftMotor = new DMC60(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new DMC60(REAR_LEFT_CHANNEL);
                frontRightMotor = new DMC60(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new DMC60(REAR_RIGHT_CHANNEL);
            }
            else if (USE_JAGUAR)
            {
                frontLeftMotor = new Jaguar(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new Jaguar(REAR_LEFT_CHANNEL);
                frontRightMotor = new Jaguar(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new Jaguar(REAR_RIGHT_CHANNEL);
            }
            else if (USE_SD540)
            {
                frontLeftMotor = new SD540(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new SD540(REAR_LEFT_CHANNEL);
                frontRightMotor = new SD540(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new SD540(REAR_RIGHT_CHANNEL);
            }
            else if (USE_SPARK)
            {
                frontLeftMotor = new Spark(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new Spark(REAR_LEFT_CHANNEL);
                frontRightMotor = new Spark(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new Spark(REAR_RIGHT_CHANNEL);
            }
            else if (USE_TALON)
            {
                frontLeftMotor = new Talon(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new Talon(REAR_LEFT_CHANNEL);
                frontRightMotor = new Talon(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new Talon(REAR_RIGHT_CHANNEL);
            }
            else if (USE_TALONSRX)
            {
                frontLeftMotor = new WPI_TalonSRX(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new WPI_TalonSRX(REAR_LEFT_CHANNEL);
                frontRightMotor = new WPI_TalonSRX(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new WPI_TalonSRX(REAR_RIGHT_CHANNEL);
            }
            else if (USE_VICTOR)
            {
                frontLeftMotor = new Victor(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new Victor(REAR_LEFT_CHANNEL);
                frontRightMotor = new Victor(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new Victor(REAR_RIGHT_CHANNEL);
            }
            else if (USE_VICTORSP)
            {
                frontLeftMotor = new VictorSP(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new VictorSP(REAR_LEFT_CHANNEL);
                frontRightMotor = new VictorSP(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new VictorSP(REAR_RIGHT_CHANNEL);
            }
            else if (USE_VICTORSPX)
            {
                frontLeftMotor = new WPI_VictorSPX(FRONT_LEFT_CHANNEL);
                rearLeftMotor = new WPI_VictorSPX(REAR_LEFT_CHANNEL);
                frontRightMotor = new WPI_VictorSPX(FRONT_RIGHT_CHANNEL);
                rearRightMotor = new WPI_VictorSPX(REAR_RIGHT_CHANNEL);
            }
            //
            // If no motor controller type is selected, frontLeft, rearLeft, frontRight and rearRight motors
            // will be null.
            //
            try
            {
                //
                // Configure the spin direction of the motors.
                //
                frontLeftMotor.setInverted(LEFT_MOTOR_INVERTED);
                rearLeftMotor.setInverted(LEFT_MOTOR_INVERTED);
                frontRightMotor.setInverted(RIGHT_MOTOR_INVERTED);
                rearRightMotor.setInverted(RIGHT_MOTOR_INVERTED);
                myMecanumRobot = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
            }
            catch (NullPointerException e)
            {
                throw new IllegalArgumentException("You must select one of the legal motor controller types.");
            }
        }
        else
        {
            throw new IllegalArgumentException("You must select one a drive type.");
        }
    }   // robotInit

    /**
     * This method is called before autonomous mode starts.
     */
    @Override
    public void autonomousInit()
    {
        //
        // Determine the time to stop the drive.
        //
        stopTime = Timer.getFPGATimestamp() + DRIVE_TIME_IN_SEC;
    }   // autonomousInit

    /**
     * This method is called periodically when autonomous mode is active.
     */
    @Override
    public void autonomousPeriodic()
    {
        if (Timer.getFPGATimestamp() < stopTime)
        {
            //
            // We are still within the drive time frame, keep driving.
            //
            if (USE_DIFFERENTIAL_DRIVE)
            {
                myDifferentialRobot.tankDrive(LEFT_DRIVE_POWER, RIGHT_DRIVE_POWER);
            }
            else if (USE_MECANUM_DRIVE)
            {
                myMecanumRobot.driveCartesian(X_DRIVE_POWER, Y_DRIVE_POWER, ROTATE_POWER);
            }
        }
        else
        {
            //
            // We are done driving, stop the drive.
            //
            if (USE_DIFFERENTIAL_DRIVE)
            {
                myDifferentialRobot.tankDrive(0.0, 0.0);
            }
            else if (USE_MECANUM_DRIVE)
            {
                myMecanumRobot.driveCartesian(0.0, 0.0, 0.0);
            }
        }
    }   // autonomousPeriodic

}   // class Robot
