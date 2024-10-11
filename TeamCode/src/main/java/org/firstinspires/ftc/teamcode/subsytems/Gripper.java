package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static java.lang.Thread.sleep;

@Config // this is so the dashboard will pick up variables
public class Gripper {

    //Define Hardware Objects
    private Servo            gripper     = null;
    private Servo            angler             = null;



    //Constants for gripper
    //larer numbers are more clockwise

    private static final double      GRIPPER_CLOSED      = 0.6; //to close more, increase
    private static final double      GRIPPER_OPEN        = 0.39; //to open more, decrease


    private static final double      ANGLER_UP     = 0.6; // deposit the pixel
    private static final double      ANGLER_DOWN      = 0.45; // Loading the pixel
    Telemetry       telemetry;
    LinearOpMode    opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();


    public double  targetHeight;
    public double  targetAngle;
    public boolean turnerDown = true;
    public boolean inLevelZero = false;

    /// constructor with opmode passed in
    public Gripper(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void init(HardwareMap hwMap)  {

        // Initialize the gripper
        gripper = hwMap.get(Servo.class,"gripper"); //Exp Hub port 1

        angler = hwMap.get(Servo.class,"angler"); // Exp Hub port 0


    }

    public void gripperClosed(){
        gripper.setPosition(GRIPPER_CLOSED);
    }
    public void gripperOpen(){
        gripper.setPosition(GRIPPER_OPEN);
    }

    public void setAnglerUP() {
        angler.setPosition(ANGLER_UP);//fwd
    }
    public void setAnglerDown() {
        angler.setPosition(ANGLER_DOWN);//fwd
    }

}
