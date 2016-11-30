package org.firstinspires.ftc.robotcontroller.external.samples;


import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotDeclarations;


@Autonomous(name="Blue_100", group="Pushbot")

public class Blue_101 extends LinearOpMode {
    RobotDeclarations robot = new RobotDeclarations(){};
    RobotFunctions function = new RobotFunctions();
    @Override
    public void runOpMode() throws InterruptedException {
        //wait for start
        waitForStart();

        // Drive forwards
        robot.right.setPower(0.4);
        robot.left.setPower(0.4);

        Stopwatch(); // start the timer
        while(hsvlinefront[2] < 0.3) // while front line following sensor does not see the tape
        {

            Color.RGBToHSV(robot.linefront.red() * 8, robot.linefront.green() * 8, robot.linefront.blue() * 8, hsvlinefront); // get color value in HSV

            currenttime = elapsedTime(); // set variable = to current time

            if(currenttime > 2) // if 2 seconds have passed

            {

                fail = true; // program failed due to not finding the line

                break;

            }

        }

        // stop motors

        robot.right.setPower(0);
        robot.left.setPower(0);

        sleep(500);

        robot.right.setPower(-0.2);
        robot.left.setPower(-0.2);

        Stopwatch();

        while(hsvlineback[2] < 0.3) // while front line following sensor does not see the tape

        {

            Color.RGBToHSV(robot.lineback.red() * 8, robot.lineback.green() * 8, robot.lineback.blue() * 8, hsvlineback); // get color value in HSV

            currenttime = elapsedTime(); // set variable = to current time

            if(currenttime > 2) // if 2 seconds have passed

            {

                fail = true; // program failed due to not finding the line

                break;

            }

        }

        // stop motors

        robot.right.setPower(0);
        robot.left.setPower(0);



        /*if(!fail) // if the line was found

        {

            wait(750);// wait 3/4 second

            linefollow(5); // follow the line for max of 5 seconds

            if(!fail) // if the beacon was found

            {

                wait(750);

                alignandpressright(5); // press buttons

                if(!fail) // if the robot is aligned

                {

                    if (colorr.blue() > colorr.red()) // if left button is red

                    {

                        robot.servor.setPosition(0.25); // press left button

                        wait(500);

                        robot.servor.setPosition(0); // release button

                    } else if (robot.colorr.blue() < robot.colorr.red()) // if right button is red

                    {

                        robot.servor.setPosition(0.25); // press left button

                        wait(500);

                        robot.servor.setPosition(0); // release button

                    }



                    wait(500); // wait 1/2 second



                    // Drive forwards

                    robot.right.setPower(1);

                    robot.left.setPower(1);



                    wait(500); // wait 1/2 second



                    Stopwatch(); // start the timer



                    Color.RGBToHSV(linefront.red() * 8, robot.linefront.green() * 8, robot.linefront.blue() * 8, hsvlinefront); // get color value in HSV



                    while (hsvlinefront[1] > 0.3 || hsvlinefront[2] < 0.7) // while front line following sensor does not see the tape

                    {

                        Color.RGBToHSV(robot.linefront.red() * 8, robot.linefront.green() * 8, robot.linefront.blue() * 8, hsvlinefront); // get color value in HSV

                        currenttime = elapsedTime(); // set variable = to current time

                        if (currenttime > 5) // if 5 seconds have passed

                        {

                            fail = true; // program failed due to not finding the line

                            break;

                        }

                    }



                    // stop motors

                    robot.right.setPower(0);

                    robot.left.setPower(0);



                    if (!fail)

                    {

                        alignandpressright(5); // press button



                        wait(500);



                        // Drive forwards

                        robot.right.setPower(-1);

                        robot.left.setPower(-1);



                        wait(500); // wait 1/2 second



                        Stopwatch(); // start the timer



                        Color.RGBToHSV(robot.lineback.red() * 8, robot.lineback.green() * 8, robot.lineback.blue() * 8, hsvlineback); // get color value in HSV



                        while (hsvlineback[1] > 0.3 || hsvlineback[2] < 0.7) // while back line following sensor does not see the tape

                        {

                            Color.RGBToHSV(robot.lineback.red() * 8, robot.lineback.green() * 8, robot.lineback.blue() * 8, hsvlineback); // get color value in HSV

                            currenttime = elapsedTime(); // set variable = to current time

                            if (currenttime > 5) // if 5 seconds have passed

                            {

                                fail = true; // program failed due to not finding the line

                                break;

                            }

                        }

                        if(!fail) // if the line is found

                        {

                            wait(500); // wait 1/2 second



                            // turn right 90 degrees

                            function.turnright(1000, 0.75);



                            wait(500); // wait 1/2 second



                            // drive backwards
*/

                            function.backwards(1000, 0.75);


/*

                            ////////////SCORE PARTICLES IN CENTER VORTEX//////////////////////////////////////////////////



                            wait(500); // wait 1/2 second



                            // slide left

                            slideleft(1000, 0.75);



                            wait(500); // wait 1/2 second



                            // turn left 45 degrees

                            turnleft(1000, 0.75);



                            wait(500); // wait 1/2 second



                            // drive backwards into the ball

                            backwards(1000, 0.75);

                        }

                    }

                }

            }

        }

        change modes of motors and stop to end program or for safety

        robot.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        robot.right.setPower(0);

        robot.left.setPower(0);

        robot.slide.setPower(0);
        */
    }



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long start; // start of stopwatch

    float hsvlinefront[] = {0F,0F,0F};

    float hsvlineback[] = {0F,0F,0F};

    double currenttime;

    boolean fail = false;
    // functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // function to start stopwatch

    public void Stopwatch() {

        start = System.currentTimeMillis();

    }



    // function to read stopwatch

    double elapsedTime() {

        long now = System.currentTimeMillis();

        return (now - start) / 1000.0;

    }



    /*
    public void linefollow(int shutoff) // line following function

    {

        Stopwatch(); // reset timer

        robot.slide.setPower(0.75);

        while(true) // infinite loop to line follow

        {

            Color.RGBToHSV(robot.lineback.red() * 8, robot.lineback.green() * 8, robot.lineback.blue() * 8, hsvlineback); // get color value in HSV

            Color.RGBToHSV(robot.linefront.red() * 8, robot.linefront.green() * 8, robot.linefront.blue() * 8, hsvlinefront); // get color value in HSV

            currenttime = elapsedTime(); // set variable = to current time

            if(robot.rangerl.getDistance(DistanceUnit.CM) <= 4 && robot.rangerr.getDistance(DistanceUnit.CM) <= 4) // if both distance sensors are within 4cm of the beacon

            {

                robot.right.setPower(0);

                robot.left.setPower(0);

                robot.slide.setPower(0);

                break;

            }

            else if (currenttime > shutoff) // if too many seconds have passed

            {

                fail = true;

                break;

            }

            else if((hsvlineback[1] > 0.3 || hsvlineback[2] < 0.7) && (hsvlinefront[1] <= 0.3 && hsvlinefront[2] >= 0.7)) // if back doesn't see tape and front does

            {

                robot.right.setPower(0.75);

                robot.left.setPower(-0.25);

            }

            else if((hsvlinefront[1] > 0.3 || hsvlinefront[2] < 0.7) && (hsvlineback[1] <= 0.3 && hsvlineback[2] >= 0.7)) // if front doesn't see tape and back does

            {

                robot.right.setPower(-0.75);

                robot.left.setPower(0.25);

            }

            else if(((hsvlinefront[1] > 0.3 || hsvlinefront[2] < 0.7) && (hsvlineback[1] > 0.3 || hsvlineback[2] < 0.7)) || ((hsvlineback[1] <= 0.3 && hsvlineback[2] >= 0.7) && (hsvlinefront[1] <= 0.1 && hsvlinefront[2] >= 0.9))) // if neither or both see the tape

            {

                robot.right.setPower(0);

                robot.left.setPower(0);

            }

        }

    }



    public void alignandpressright(int shutoff) // presses buttons on the right side

    {

        Stopwatch();

        while(true) // infinite loop to align to buttons

        {

            currenttime = elapsedTime(); // set variable = to current time

            if(currenttime > shutoff) // if too many seconds have passed

            {

                fail = true; // fail because unknown alignment

                break;

            }

            else if(robot.rangerl.getDistance(DistanceUnit.CM) -  robot.rangerr.getDistance(DistanceUnit.CM) < 1 && robot.rangerl.getDistance(DistanceUnit.CM) -  robot.rangerr.getDistance(DistanceUnit.CM) > -1) // if the robot is angled straight

            {

                if(robot.rangerl.getDistance(DistanceUnit.CM) < 2.5 && robot.rangerl.getDistance(DistanceUnit.CM) > 2) // if the robot is the correct distance

                {

                    robot.slide.setPower(0); // stop sliding

                    if(((hsvlinefront[1] > 0.3 || hsvlinefront[2] < 0.7) && (hsvlineback[1] > 0.3 || hsvlineback[2] < 0.7)) || ((hsvlineback[1] <= 0.3 && hsvlineback[2] >= 0.7) && (hsvlinefront[1] <= 0.3 && hsvlinefront[2] >= 0.7))) // if both or neither line following sensor sees tape

                    {

                        frontright.setPower(0);

                        frontleft.setPower(0);

                        backright.setPower(0);

                        backleft.setPower(0);

                        break;

                    }

                    else if((hsvlineback[1] > 0.3 || hsvlineback[2] < 0.7) && (hsvlinefront[1] <= 0.3 && hsvlinefront[2] >= 0.7)) // if back doesn't see tape and front does

                    {

                        frontright.setPower(0.5);

                        frontleft.setPower(0.5);

                        backright.setPower(0.5);

                        backleft.setPower(0.5);

                    }

                    else if((hsvlinefront[1] > 0.3 || hsvlinefront[2] < 0.7) && (hsvlineback[1] <= 0.3 && hsvlineback[2] >= 0.7)) // if front doesn't see tape and back does

                    {

                        frontright.setPower(-0.5);

                        frontleft.setPower(-0.5);

                        backright.setPower(-0.5);

                        backleft.setPower(-0.5);

                    }

                }

                else if(rangerl.getDistance(DistanceUnit.CM) >= 3) // if the robot is too far away

                {

                    frontright.setPower(0); // stop turning

                    frontleft.setPower(0);

                    backright.setPower(0);

                    backleft.setPower(0);

                    robot.slide.setPower(0.5); // slide right

                }

                else if(rangerl.getDistance(DistanceUnit.CM) <= 2) // if the robot is too close

                {

                    frontright.setPower(0); // stop turning

                    frontleft.setPower(0);

                    backright.setPower(0);

                    backleft.setPower(0);

                    robot.slide.setPower(-0.5); // slide left

                }

            }

            else if(rangerl.getDistance(DistanceUnit.CM) -  rangerr.getDistance(DistanceUnit.CM) <= -1) // if the robot is facing right

            {

                // turn left

                frontright.setPower(0.5);

                frontleft.setPower(-0.5);

                backright.setPower(0.5);

                backleft.setPower(-0.5);

                robot.slide.setPower(0);

            }

            else if(rangerl.getDistance(DistanceUnit.CM) -  rangerr.getDistance(DistanceUnit.CM) >= 1) // if the robot is facing left

            {

                // turn right

                frontright.setPower(-0.5);

                frontleft.setPower(0.5);

                backright.setPower(-0.5);

                backleft.setPower(0.5);

                robot.slide.setPower(0);

            }

        }

    }



    /*public void alignandpressleft(int shutoff) // presses buttons on the left side

    {

        ////////////////// MUST BE FIXED ONCE OTHER FUNCTION WORKS ///////////////

    }*/

}
