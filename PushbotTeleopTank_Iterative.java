package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import java.io.*;
@TeleOp(name="Team 4262: Teleop Tank", group="Pushbot")
@Disabled
public class PushbotTeleopTank_Iterative extends OpMode
{
    HardwarePushbot robot = new HardwarePushbot();
    @Override
    public void init()
    {
        robot.init(hardwareMap);
        telemetry.addData("Ladies and Gentlemen", "please fasten your seat belts, and put your chair in the upright position. Please relax and enjoy the ride.");
    }
    @Override
    public void init_loop() {}
    @Override
    public void start() {}
    @Override
    public void loop() {
        double left;
        double right;
        double leftArm;
        double rightArm;

        // Run wheels in tank mode
        right = -gamepad1.right_stick_y /1.25;
        left = -gamepad1.left_stick_y / 1.25;
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);

        //Run lift motors in "tank" formation
        leftArm = -gamepad2.left_stick_y;
        rightArm = gamepad2.right_stick_y;
        robot.armMotor.setPower(leftArm);
        robot.armMotor2.setPower(rightArm);


        //Run lateral motor with left and right triggers
        if (gamepad1.right_bumper)
        {
            robot.centerMotor.setPower(0.6);
        }
        else if(gamepad1.left_bumper)
        {
            robot.centerMotor.setPower(-0.6);
        }
        else
        {
            robot.centerMotor.setPower(0.0);
        }

        if (gamepad1.y)
        {
            robot.leftMotor.setPower(0.4);
            robot.rightMotor.setPower(0.6);
        }
        else if (gamepad1.a)
        {
            robot.leftMotor.setPower(-0.4);
            robot.rightMotor.setPower(-0.6);
        }
        if(gamepad2.start)
        {
            robot.rightBumper.setPosition(1);
            robot.leftBumper.setPosition(0);
        }
        else if(gamepad2.back)
        {
            robot.rightBumper.setPosition(0.5);
            robot.leftBumper.setPosition(0.5);
        }
    }
    @Override
    public void stop()
    {

    }

}
