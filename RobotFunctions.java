package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.external.samples.RobotDeclarations;

/**
 * Created by rajan on 11/29/2016.
 */

public class RobotFunctions
{
    RobotDeclarations robot = new RobotDeclarations(){};























    public void forwards(int ticks, double speed) // function for forwards

    {

        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.right.setTargetPosition(ticks);

        robot.left.setTargetPosition(ticks);

        robot.right.setPower(speed);

        robot.left.setPower(speed);



        while(robot.right.isBusy() && robot.left.isBusy())

        {



        }

        robot.right.setPower(0);

        robot.left.setPower(0);

    }
























    public void backwards(int ticks, double speed) // function for backwards

    {

        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.right.setTargetPosition(-ticks);

        robot.left.setTargetPosition(-ticks);

        robot.right.setPower(speed);

        robot.left.setPower(speed);



        while(robot.right.isBusy() && robot.left.isBusy())

        {



        }

        robot.right.setPower(0);

        robot.left.setPower(0);

    }





























    public void turnright(int ticks, double speed) // function for turning right

    {

        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.right.setTargetPosition(-ticks);

        robot.left.setTargetPosition(ticks);

        robot.right.setPower(speed);

        robot.left.setPower(speed);



        while(robot.right.isBusy() && robot.left.isBusy())

        {



        }

        robot.right.setPower(0);

        robot.left.setPower(0);

    }







































    public void turnleft(int ticks, double speed) // function for turning left

    {

        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.right.setTargetPosition(ticks);

        robot.left.setTargetPosition(-ticks);

        robot.right.setPower(speed);

        robot.left.setPower(speed);



        while(robot.right.isBusy() && robot.left.isBusy())

        {



        }

        robot.right.setPower(0);

        robot.left.setPower(0);

    }






























    public void slideright(int ticks, double speed) // function for sliding right

    {

        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.slide.setTargetPosition(ticks);

        robot.slide.setPower(speed);



        while(robot.slide.isBusy())

        {



        }

        robot.slide.setPower(0);

    }































    public void slideleft(int ticks, double speed) // function for sliding left

    {

        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.slide.setTargetPosition(-ticks);

        robot.slide.setPower(speed);



        while(robot.slide.isBusy())

        {



        }

        robot.slide.setPower(0);

    }





}
