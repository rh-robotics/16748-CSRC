//
package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {

    private Hardware robot;

    {
        new Hardware();
    }

    private Telemetry telemetry;
    public boolean drivestate;
    public Drive(Hardware robot) {
        super();
        this.robot = robot;
    }

    public void manualDrive(Gamepad gamepad) {

        double strafe_Y = gamepad.left_stick_y; // forward is pos, backward is neg. (-1 <= magnitude <= 1)
        double strafe_X = -gamepad.right_stick_x; //right is pos, left is neg. (-1 <= magnitude <= 1)

        double blStrafePwr = (strafe_X+strafe_Y);
        double brStrafePwr = -(strafe_X-strafe_Y);
        double flStrafePwr = -(strafe_X-strafe_Y);
        double frStrafePwr = (strafe_X+strafe_Y);

        double rotate = gamepad.left_stick_x;

        //Counter clockwise is positive (right joystick), Counter is neg (joystick left?)

        if (strafe_Y==0 && strafe_X==0) {
            robot.blDrive.setPower(rotate);
            robot.brDrive.setPower(-rotate);
            robot.flDrive.setPower(rotate);
            robot.frDrive.setPower(-rotate);
        }
        else {
            robot.blDrive.setPower(blStrafePwr);
            robot.brDrive.setPower(brStrafePwr);
            robot.flDrive.setPower(flStrafePwr);
            robot.frDrive.setPower(frStrafePwr);
        }

        //telemetry.addData("Motors driving? ", )
        //telemetry.update();


    }
    public boolean stopDrive = true;
}