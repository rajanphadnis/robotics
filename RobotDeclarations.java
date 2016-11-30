package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by rajan on 11/29/2016.
 */

public class RobotDeclarations
{

    //Motors and Servo declarations
    public DcMotor right;
    public DcMotor left;
    public DcMotor slide;
    public DcMotor rightarm;
    public DcMotor leftarm;
    public Servo servol;
    public Servo servor;

    //Sensor declarations
    public ColorSensor lineback;
    public ColorSensor linefront;
    //public ColorSensor colorl;
    public ColorSensor colorr;
    //public ModernRoboticsI2cRangeSensor rangell;
    //public ModernRoboticsI2cRangeSensor rangelr;
    //public ModernRoboticsI2cRangeSensor rangerl;
    //public ModernRoboticsI2cRangeSensor rangerr;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RobotDeclarations(){}

    //Hardware map
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Define Motors
        right  = hwMap.dcMotor.get("right_drive");
        left = hwMap.dcMotor.get("left_drive");
        slide = hwMap.dcMotor.get("center_drive");
        leftarm = hwMap.dcMotor.get("left_arm");
        rightarm = hwMap.dcMotor.get("right_arm");

        //Define Servos
        servol = hwMap.servo.get("right_bumper");
        servor = hwMap.servo.get("right_bumper");

        //Define Sensors
        lineback = hwMap.colorSensor.get("line back");
        linefront = hwMap.colorSensor.get("line front");
        //colorl = hwMap.colorSensor.get("color l");
        colorr = hwMap.colorSensor.get("color r");
        //rangell = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range ll");
        //rangelr = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range lr");
        //rangerl = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range rl");
        //rangerr = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range rr");

        //I2C definitions
        colorr.setI2cAddress(I2cAddr.create7bit(0x1e));
        lineback.setI2cAddress(I2cAddr.create7bit(0x26));
        linefront.setI2cAddress(I2cAddr.create7bit(0x2e));

        //Forward or reverse for motor directions
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        leftarm.setDirection(DcMotor.Direction.REVERSE);
        rightarm.setDirection(DcMotor.Direction.FORWARD);

        //Brakes
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoder Declarations
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset and stop encoders
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //servo positions
        servol.setPosition(0);
        servor.setPosition(0);
    }


    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }


}
