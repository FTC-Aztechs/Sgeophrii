/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Sgp_robot
{
    /* Public OpMode members. */
    public DcMotor upper_right = null;
    public DcMotor upper_left = null;
    public DcMotor lower_left = null;
    public DcMotor lower_right = null;
    public DcMotor Arm_Motor = null;
    public DcMotor Batman_Belt = null;
    public DcMotor Bravo_6 = null;
    public Servo Wrist_1 = null;
    public Servo Wrist_2 = null;
    public Servo Finger = null;
    public ServoController FingerController = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Sgp_robot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        upper_right  = hwMap.get(DcMotor.class, "upper_right");
        upper_left = hwMap.get(DcMotor.class, "upper_left");
        lower_left = hwMap.get(DcMotor.class, "lower_left");
        lower_right = hwMap.get(DcMotor.class, "lower_right");
        Arm_Motor = hwMap.get(DcMotor.class, "Arm_Motor");
        Batman_Belt = hwMap.get(DcMotor.class, "Batman_Belt");
        Bravo_6 = hwMap.get(DcMotor.class, "Bravo_6");
        Wrist_1 = hwMap.get(Servo.class, "Wrist_1");
        Wrist_2 = hwMap.get(Servo.class, "Wrist_2");
        Finger = hwMap.get(Servo.class, "Finger");

        // Set all motors to zero power
        upper_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Batman_Belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bravo_6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        upper_left.setDirection(DcMotor.Direction.FORWARD);
        upper_right.setDirection(DcMotor.Direction.REVERSE);
        lower_left.setDirection(DcMotor.Direction.FORWARD);
        lower_right.setDirection(DcMotor.Direction.REVERSE);
        Arm_Motor.setDirection(DcMotor.Direction.FORWARD);
        Batman_Belt.setDirection(DcMotor.Direction.REVERSE);
        Bravo_6.setDirection(DcMotor.Direction.FORWARD);
        Wrist_1.setDirection(Servo.Direction.FORWARD );
        Wrist_2.setDirection(Servo.Direction.FORWARD );
        Finger.setDirection(Servo.Direction.FORWARD );

        //zero power behavior
        upper_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Batman_Belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bravo_6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
 }

