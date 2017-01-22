package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by KJsMacbookAir on 1/10/17.
 */
@Autonomous(name="Redforia", group="Pushbot")
public class Redforia extends LinearOpMode
{
    DcMotor right;
    DcMotor left;
    DcMotor slide;
    DcMotor launcher;
    Servo pusher;
    ColorSensor linefront;
    ColorSensor colorr;
    ColorSensor colorl;
    @Override
    public void runOpMode() throws InterruptedException
    {
        right  = hardwareMap.dcMotor.get("right_drive");
        left = hardwareMap.dcMotor.get("left_drive");
        slide = hardwareMap.dcMotor.get("center_drive");
        launcher = hardwareMap.dcMotor.get("launch");
        //pusher = hardwareMap.servo.get("push");
        linefront = hardwareMap.colorSensor.get("line front");
        colorr = hardwareMap.colorSensor.get("color r");
        colorl = hardwareMap.colorSensor.get("color l");
        linefront.setI2cAddress(I2cAddr.create7bit(0x2e));
        colorr.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorl.setI2cAddress(I2cAddr.create7bit(0x36));

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AQPkZSv/////AAAAGbPSw+T5LUiCrDYdMN3600Qy2wZe67Pn2XKPmGybiNP4ZtGCtgk56vMUKhmMdG0aXVDqK/+8f1m16rx7hboA9o5uzUxKa+yGcWUru0beihfpaqJDRGrqj1uPPaNZzDaaSAG/aJ3/PlOF26MrZ++TMXwSjWy6tEF5bzSqmQrGVS9JK7p9qp2XArUPh7E5k049sXJWdW2nTZ6/tykCMbR0ZLr4M3+xAyfVwGFQ1RNJzn8xfp/kMDxOjJnPsLn169O8HXHuvZJV/7uAAsZlW1/6LQq9Hrg6+jtdb7H4OqS/2HqX5VGvNAlOTbGOAXCWMoe7NBkSbfQuzvSG5clMrVLsM4NTLFydWo90uCAyT4B2l80t";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        params.useExtendedTracking = false;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        VuforiaTrackableDefaultListener gears = (VuforiaTrackableDefaultListener) beacons.get(3).getListener();

        beacons.activate();

        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linefront.enableLed(true);
        colorl.enableLed(false);
        colorr.enableLed(false);

        int startreading = linefront.alpha();

        //pusher.setPosition(0.3);

        waitForStart();

        forwards(2350, .4);
        sleep(500);
        turnright(425, .4);
        sleep(500);
        right.setPower(0.125);
        left.setPower(0.125);
        while((linefront.alpha()) - startreading < 5 && opModeIsActive())
        {
            idle();
        }
        left.setPower(0);
        right.setPower(0);
        sleep(500);
        /*while(opModeIsActive() && wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 25)
        {
            if(wheels.getPose() != null)
            {
                if(wheels.getPose().getTranslation().get(0) > 25)
                {
                    right.setPower(-0.1);
                    left.setPower(0.1);
                }
                else
                {
                    right.setPower(0.1);
                    left.setPower(-0.1);
                }
            }
            else
            {
                right.setPower(-0.1);
                left.setPower(0.1);
            }
        }*/
        turnleft(800, 0.2);
        right.setPower(0);
        left.setPower(0);
        sleep(500);
        right.setPower(0.1);
        left.setPower(0.1);
        while(opModeIsActive() && gears.getPose().getTranslation().get(2) < -150)
        {
            if(gears.getPose().getTranslation().get(0) > 0)
            {
                slide.setPower(0.25);
                //pusher.setPosition(0);
            }
            else if(gears.getPose().getTranslation().get(0) < -0)
            {
                slide.setPower(-0.25);
                //pusher.setPosition(0.6);
            }
            else
            {
                slide.setPower(0);
            }
            idle();
        }
        right.setPower(0);
        left.setPower(0);
        slide.setPower(0);
        sleep(500);
        if(colorr.red() > colorl.red())
        {
            slideleft(1050, 0.5);
        }
        else
        {
            slideright(1050, 0.5);
        }
        sleep(500);
        forwards(250, 0.1);
        sleep(500);
        backwards(2000, 0.3);
        sleep(500);
        turnleft(900, 0.2);
        sleep(500);
        launcher.setPower(1);
        sleep(100);
        launcher.setPower(0);
        backwards(1250, 0.3);
        sleep(500);
        turnleft(350, 0.4);
        sleep(500);
        forwards(2000, 0.3);
        launcher.setPower(0);
        slide.setPower(0);
        left.setPower(0);
        right.setPower(0);
        stop();
    }

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
        right.setTargetPosition(ticks);
        left.setTargetPosition(ticks);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setPower(speed);
        left.setPower(speed);
        while(right.isBusy() && left.isBusy() && opModeIsActive())
        {
            idle();
        }
        telemetry.addData("this", "is thie");
        telemetry.update();
        left.setPower(0);
        right.setPower(0);
        left.setPower(0);
        right.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void backwards(int ticks, double speed) // function for backwards
    {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setTargetPosition(-ticks);
        left.setTargetPosition(-ticks);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setPower(-speed);
        left.setPower(-speed);
        while(right.isBusy() && left.isBusy() && opModeIsActive())
        {
            idle();
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
        right.setTargetPosition(-ticks);
        left.setTargetPosition(ticks);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setPower(-speed);
        left.setPower(speed);
        while(right.isBusy() && left.isBusy() && opModeIsActive())
        {
            idle();
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
        right.setTargetPosition(ticks);
        left.setTargetPosition(-ticks);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setPower(speed);
        left.setPower(-speed);
        while(right.isBusy() && left.isBusy() && opModeIsActive())
        {
            idle();
        }
        right.setPower(0);
        left.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void slideright(int ticks, double speed) // function for sliding right
    {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(ticks);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(speed);
        while(slide.isBusy() && opModeIsActive())
        {
            idle();
        }
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void slideleft(int ticks, double speed) // function for sliding left
    {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(-ticks);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(-speed);
        while(slide.isBusy() && opModeIsActive())
        {
            idle();
        }
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){ return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){ float [] data = image.getRawPose().getData(); float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}}; double thetaX = Math.atan2(rotation[2][1], rotation[2][2]); double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2])); double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]); return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
}
