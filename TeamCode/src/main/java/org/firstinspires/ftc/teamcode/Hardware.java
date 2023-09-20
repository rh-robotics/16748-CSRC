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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
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
 * Motor channel:  Left  drive motor:        "flDrive"
 * Motor channel:  Right drive motor:        "frDrive"
 * Motor channel:  Lef  drive motor:        "brDrive"
 * Motor channel:  Right drive motor:        "flDrive"
 * Motor channel:  servos and motors to come
 * Servo channel:  ibid
 * Servo channel: ibid
 */
public class Hardware
{
    /* Public OpMode members. */
    public DcMotorEx blDrive = null;
    public DcMotorEx brDrive = null;
    public DcMotorEx flDrive = null;
    public DcMotorEx frDrive = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    //private ElapsedTime period  = new ElapsedTime();
    public static final double MAX_SPEED= 1;
    //public static final double servoClawOut = 0;
    //public static final double servoClawIn = 1;


    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        blDrive  = hwMap.get(DcMotorEx.class, "blDrive");
        brDrive  = hwMap.get(DcMotorEx.class, "brDrive");
        flDrive  = hwMap.get(DcMotorEx.class, "flDrive");
        frDrive  = hwMap.get(DcMotorEx.class, "frDrive");


        blDrive.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        brDrive.setDirection(DcMotorEx.Direction.FORWARD);
        flDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frDrive.setDirection(DcMotorEx.Direction.FORWARD);



        // Set motors (and CRServos eventually) to zero power
        blDrive.setPower(0);
        brDrive.setPower(0);
        flDrive.setPower(0);
        frDrive.setPower(0);


        //Set servos to correct locations when servos applicable (arm and gorail stuff)

        //wristServo.setPosition(Servo.MIN_POSITION);
        //armServo.setPosition();


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        blDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        brDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }
}
//L