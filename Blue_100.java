//rajan's package
//package org.firstinspires.ftc.robotcontroller.external.samples;
//KJ's package
package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Blue_100", group="Pushbot")
public class Blue_100 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        right  = hardwareMap.dcMotor.get("right_drive");
        left = hardwareMap.dcMotor.get("left_drive");
        slide = hardwareMap.dcMotor.get("center_drive");
        leftarm = hardwareMap.dcMotor.get("left_arm");
        rightarm = hardwareMap.dcMotor.get("right_arm");
        servol = hardwareMap.servo.get("right_bumper");
        servor = hardwareMap.servo.get("right_bumper");
        lineback = hardwareMap.colorSensor.get("line back");
        linefront = hardwareMap.colorSensor.get("line front");
        rangesensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        colorr = hardwareMap.colorSensor.get("color r");
        colorr.setI2cAddress(I2cAddr.create7bit(0x1e));
        lineback.setI2cAddress(I2cAddr.create7bit(0x26));
        linefront.setI2cAddress(I2cAddr.create7bit(0x2e));
        //colorl.setI2cAddress(I2cAddr.create7bit(0x36));
        // color r = 0x1e (0x3c in 8 bit)
        // line back = 0x26 (0x4c in 8 bit)
        //line front = 0x2e (0x5c in 8 bit)
        // color l = 0x36 (0x6c in 8 bit)
        // "Reverse" the motor that runs backwards when connected directly to the battery
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        leftarm.setDirection(DcMotor.Direction.REVERSE);
        rightarm.setDirection(DcMotor.Direction.FORWARD);
        // set brakes to on
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // reset encoder positions
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set main drive motos to run using encoders
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // set initial value for servos
        servor.setPosition(0);
        // turn on color sensors
        linefront.enableLed(true);
        lineback.enableLed(true);
        colorr.enableLed(true);
        //colorl.enableLed(true);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        forwards(2400, .4);
        sleep(500);
        turnleft(350, .4);
        sleep(500);
        rightpower = 0.5;
        leftpower = 0.5;
        right.setPower(rightpower);
        left.setPower(leftpower);
        Stopwatch(); // start the timer
        Color.RGBToHSV(linefront.red() * 8, linefront.green() * 8, linefront.blue() * 8, hsvlinefront); // get color value in HSV
        currentdistance = rangesensor.cmUltrasonic();
        while(hsvlinefront[2] < 0.3) // while front line following sensor does not see the tape
        {
            Color.RGBToHSV(linefront.red() * 8, linefront.green() * 8, linefront.blue() * 8, hsvlinefront); // get color value in HSV
            currenttime = elapsedTime(); // set variable = to current time
            if (currenttime > 10) // if 2 seconds have passed
            {
            fail = true; // program failed due to not finding the line
            break;
            }
            if (rangesensor.cmUltrasonic() < currentdistance) {
            leftpower = leftpower - 0.1;
            rightpower = rightpower + 0.1;
            } else if (rangesensor.cmUltrasonic() < currentdistance) {
            leftpower = leftpower + 0.1;
            rightpower = rightpower - 0.1;
            }
            right.setPower(rightpower);
            left.setPower(leftpower);
            currentdistance = rangesensor.cmUltrasonic();
        }
        left.setPower(0);
        right.setPower(0);
        if(!fail) // if the line was found
        {
            sleep(500);// wait 3/4 second
            linefollowright(3); // follow the line for max of 5 seconds
            colorr.enableLed(false);
            if (colorr.blue() > colorr.red()) // if left button is blue
            {
                backwards(200, .4);
                telemetry.addData("color", "blue");
                telemetry.update();
            }
            else if (colorr.blue() < colorr.red()) // if left button is blue
            {
                backwards(350, .4);
                telemetry.addData("color", "red");
                telemetry.update();
            }
            else
            {
                forwards(400, .4);
                telemetry.addData("color", "fail");
                telemetry.update();
            }
            slideright(700, .4);
            slideleft(700, .4);
            if (!fail) // if not failed
            {
                rightpower = 0.7;
                leftpower = 0.7;
                right.setPower(rightpower);
                left.setPower(leftpower);
                sleep(250); // wait 1/2 second
                Stopwatch(); // start the timer
                Color.RGBToHSV(linefront.red() * 8, linefront.green() * 8, linefront.blue() * 8, hsvlinefront); // get color value in HSV
                currentdistance = rangesensor.cmUltrasonic();
                while (hsvlinefront[2] < 0.3) // while front line following sensor does not see the tape
                {
                    Color.RGBToHSV(linefront.red() * 8, linefront.green() * 8, linefront.blue() * 8, hsvlinefront); // get color value in HSV
                    currenttime = elapsedTime(); // set variable = to current time
                    if (currenttime > 10) // if 2 seconds have passed
                    {
                        fail = true; // program failed due to not finding the line
                        break;
                    }
                    if (rangesensor.cmUltrasonic() < currentdistance) {
                        leftpower = leftpower - 0.1;
                        rightpower = rightpower + 0.1;
                    } else if (rangesensor.cmUltrasonic() < currentdistance) {
                        leftpower = leftpower + 0.1;
                        rightpower = rightpower - 0.1;
                    }
                    right.setPower(rightpower);
                    left.setPower(leftpower);
                    currentdistance = rangesensor.cmUltrasonic();
                }
                // stop motors
                right.setPower(0);
                left.setPower(0);
                wait(500);
                if (!fail) {
                    // score on second beacon
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    slideleft(1000, .5);
                    turnleft(400, .5);
                    forwards(2000, .7);
                }
            }
        }
        //change modes of motors and stop to end program or for safety
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setPower(0);
        left.setPower(0);
        slide.setPower(0);
    }
    // declarations
    DcMotor right = null;
    DcMotor left = null;
    DcMotor slide = null;
    DcMotor rightarm = null;
    DcMotor leftarm = null;
    Servo servol = null;
    Servo servor = null;
    ColorSensor lineback;
    ColorSensor linefront;
    ColorSensor colorl;
    ColorSensor colorr;
    ModernRoboticsI2cRangeSensor rangesensor;
    long start; // start of stopwatch
    float hsvlinefront[] = {0F,0F,0F};
    float hsvlineback[] = {0F,0F,0F};
    double currenttime;
    boolean fail = false;
    double currentdistance = 0;
    double rightpower = 0;
    double leftpower = 0;

    public void Stopwatch()
    {
        start = System.currentTimeMillis();
    }

    double elapsedTime()
    {
        long now = System.currentTimeMillis();
        return (now - start) / 1000.0;
    }

    public void forwards(int ticks, double speed) // function for forwards
    {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setTargetPosition(ticks);
        left.setTargetPosition(ticks);
        right.setPower(speed);
        left.setPower(speed);
        while(right.isBusy() && left.isBusy())
        {
        }
        left.setPower(0);
        right.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void backwards(int ticks, double speed) // function for backwards
    {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setTargetPosition(-ticks);
        left.setTargetPosition(-ticks);
        right.setPower(-speed);
        left.setPower(-speed);
        while(right.isBusy() && left.isBusy())
        {
        }
        right.setPower(0);
        left.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnright(int ticks, double speed) // function for turning right
    {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setTargetPosition(-ticks);
        left.setTargetPosition(ticks);
        right.setPower(-speed);
        left.setPower(speed);
        while(right.isBusy() && left.isBusy())
        {
        }
        right.setPower(0);
        left.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnleft(int ticks, double speed) // function for turning left
    {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setTargetPosition(ticks);
        left.setTargetPosition(-ticks);
        right.setPower(speed);
        left.setPower(-speed);
        while(right.isBusy() && left.isBusy())
        {
        }
        right.setPower(0);
        left.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void slideright(int ticks, double speed) // function for sliding right
    {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setTargetPosition(ticks);
        slide.setPower(speed);
        while(slide.isBusy())
        {
        }
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void slideleft(int ticks, double speed) // function for sliding left
    {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setTargetPosition(-ticks);
        slide.setPower(-speed);
        while(slide.isBusy())
        {
        }
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void linefollowright(int shutoff) // line following function
    {
        Stopwatch(); // reset timer
        slide.setPower(0.25);
        left.setPower(0);
        right.setPower(0);
        boolean waveback = true;
        double timenow;
        double lasttime = elapsedTime();
        while(true) // infinite loop to line follow
        {
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

            /*timenow = elapsedTime();
            if(waveback)
            {
                left.setPower(0.5);
                right.setPower(-.5);
                if(timenow - lasttime > .1)
                {
                    waveback = false;
                    lasttime = elapsedTime();
                }
            }
            else
            {
                left.setPower(-0.5);
                right.setPower(.5);
                if(timenow - lasttime > .1)
                {
                    waveback = true;
                    lasttime = elapsedTime();
                }
            }*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
            Color.RGBToHSV(lineback.red() * 8, lineback.green() * 8, lineback.blue() * 8, hsvlineback); // get color value in HSV
            Color.RGBToHSV(linefront.red() * 8, linefront.green() * 8, linefront.blue() * 8, hsvlinefront); // get color value in HSV
            currenttime = elapsedTime(); // set variable = to current time
            if(rangesensor.getDistance(DistanceUnit.CM) <= 13) // if  distance sensor is within 13cm of the wall
            {
                right.setPower(0);
                left.setPower(0);
                slide.setPower(0);
                break;
            }
            else if (currenttime > shutoff) // if too many seconds have passed
            {
                fail = true;
                break;
            }
            else if(hsvlinefront[2] > 0.3 && hsvlineback[2] <= 0.3) // if back doesn't see tape and front does
            {
                right.setPower(0.4);
            }
            else if(hsvlinefront[2] <= 0.3 && hsvlineback[2] > 0.3) // if front doesn't see tape and back does
            {
                right.setPower(-0.4);
                telemetry.addData("things", "are working");
                telemetry.update();
            }
            else if((hsvlinefront[2] <= 0.3 && hsvlineback[2] <= 0.3) || (hsvlinefront[2] > 0.3 && hsvlineback[2] > 0.3)) // if neither or both see the tape
            {
                right.setPower(0);
            }
        }
        slide.setPower(0);
        right.setPower(0);
        left.setPower(0);
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
            else if(rangesensor.getDistance(DistanceUnit.CM) < 6 && rangesensor.getDistance(DistanceUnit.CM) > 4) // if the robot is the correct distance
            {
                slide.setPower(0); // stop sliding
                if((hsvlinefront[2] <= 3 && hsvlineback[2] <= 3) || (hsvlinefront[2] > 3 && hsvlineback[2] > 3)) // if both or neither line following sensor sees tape
                {
                    right.setPower(0);
                    left.setPower(0);
                    break;
                }
                else if(hsvlinefront[2] > 0.3 && hsvlineback[2] <= 0.3) // if back doesn't see tape and front does
                {
                    right.setPower(0.25);
                    left.setPower(0.25);
                }
                else if(hsvlineback[2] > 0.3 && hsvlinefront[2] <= 0.3) // if front doesn't see tape and back does
                {
                    right.setPower(-0.25);
                    left.setPower(-0.25);
                }
            }
            else if(rangesensor.getDistance(DistanceUnit.CM) >= 6) // if the robot is too far away
            {
                right.setPower(0); // stop turning
                left.setPower(0);
                slide.setPower(0.25); // slide right
            }
            else if(rangesensor.getDistance(DistanceUnit.CM) <= 4) // if the robot is too close
            {
                right.setPower(0); // stop turning
                left.setPower(0);
                slide.setPower(-0.25); // slide left
            }
        }
        right.setPower(0);
        left.setPower(0);
        slide.setPower(0);
    }
}
