package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by samsung on 2017/12/3.
 */
@Autonomous(name="Blue", group="Linear Opmode")
//@Disabled
public class Blue extends LinearOpMode {
    private static final int BLUE = 3;
    private ElapsedTime runtime = new ElapsedTime();

    boolean jewelFinished;
    byte[] colorCcache;
    int colorCNumber;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;

    private DcMotor LeftBackMotor;
    private DcMotor RightBackMotor;
    private DcMotor LeftFrontMotor;
    private DcMotor RightFrontMotor;
    private Servo jewelLeft;
    private Servo jewelRight;
    private DcMotor LiftMotor;
    private DcMotor LiftMotor2;
    private CRServo ClawL;
    private CRServo ClawR;
    private CRServo IntakeR;
    private CRServo IntakeL;
    private Servo IntLnkR;
    private Servo IntLnkL;


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_LIFT = (1120)/(6*3.1415);

    static final double     DRIVE_SPEED             = 0.3;

    static final double jewelDistance = 2.0;
    static final double CENTER_DISTANCE = 28.0;
    static final double OFFSET = 7.5;

    static double distanceTraveled = 0.0;

    public void runOpMode(){
        distanceTraveled=0.0;
        motorInit();
        initializeColorSensors();
        jewelLeft = hardwareMap.servo.get("jewelLeft");
        jewelRight = hardwareMap.servo.get("jewelRight");
        telemetry.update();
        waitForStart();
        telemetry.log().clear();
        //lift(DRIVE_SPEED,4);
        jewelRight.setPosition(1);
        IntLnkL.setPosition(0.9);
        runtime.reset();
        while(!jewelFinished) {
            jewelLeft.setPosition(0.95);
            readColorNumberC();
            if(runtime.milliseconds()>2000) {
                jewelLeft.setPosition(0.25);
                gyroDrive(DRIVE_SPEED,jewelDistance);
                jewelFinished=true;
                break;
            }
            if (colorCNumber != 0) hitJewel();
            displayColorNumber();
            telemetry.update();
        }
        jewelLeft.setPosition(0.25);
        drive(0,0,0);
        telemetry.log().clear();
        telemetry.update();
        gyroDrive(DRIVE_SPEED,3);
        gyroDrive(DRIVE_SPEED,14.5-distanceTraveled);
        distanceTraveled=0.0;
        drive(0,0,0);
        straffe(1500,DRIVE_SPEED);
        drive(0,0,0);
        gyroStraffe(DRIVE_SPEED,CENTER_DISTANCE);
        runtime.reset();
        while(runtime.milliseconds()<1000) {
            drive(0, 0, 0);
        }
        //driveForward(200,DRIVE_SPEED);
        gyroDrive(DRIVE_SPEED,6);
        ClawL.setPower(-1);
        ClawR.setPower(-1);
        runtime.reset();
        while(runtime.milliseconds()<1000) {

        }
        gyroDrive(DRIVE_SPEED,-3);
        drive(0,0,0);
        gyroDrive(DRIVE_SPEED,9);
        drive(0,0,0);
        gyroDrive(DRIVE_SPEED,-8);
        ClawL.setPower(0);
        ClawR.setPower(0);
        telemetry.update();
    }


    public void drive(double x, double y, double r) {
        double frValue = y - x -  r;
        double flValue = y + x + r;
        double rrValue = y + x - r;
        double rlValue = y - x + r;

        double maxValue = Math.max(
                Math.max(Math.abs(flValue), Math.abs(rlValue)),
                Math.max(Math.abs(frValue), Math.abs(rrValue))
        );
        if (maxValue > 1.0) {
            flValue /= maxValue;
            rlValue /= maxValue;
            frValue /= maxValue;
            rrValue /= maxValue;
        }
        LeftFrontMotor.setPower(flValue);
        RightFrontMotor.setPower(frValue);
        LeftBackMotor.setPower(rlValue);
        RightBackMotor.setPower(rrValue);
    }

    public void hitJewel(){
            if (colorCNumber != BLUE) {
                gyroDrive(DRIVE_SPEED, jewelDistance + 1);
                runtime.reset();
                while(runtime.milliseconds()<1000){

                }
                distanceTraveled += 1;
            } else {
                gyroDrive(DRIVE_SPEED, -jewelDistance);
                while(runtime.milliseconds()<5000){
                    drive(0,0,0);
                }
                jewelLeft.setPosition(0.25);
                gyroDrive(DRIVE_SPEED, jewelDistance * 2);
            }
        jewelFinished = true;
        colorCreader.disengage();
    }

    public void displayColorNumber(){
        telemetry.addData("Color C: " ,colorCNumber);
    }

    public void readColorNumberC(){
        colorCcache = colorCreader.read(0x04, 1);
        colorCNumber = colorCcache[0];
    }

    public void initializeColorSensors(){
        colorC = hardwareMap.i2cDevice.get("color sensor A");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3a), false);
        colorCreader.engage();
        colorCreader.write8(3,0);
    }

    public void lift(double speed, double distance){
        int moveCounts;
        int target;

        moveCounts = (int)(distance * COUNTS_PER_INCH_LIFT);
        target = LiftMotor.getCurrentPosition()+moveCounts;
        LiftMotor.setTargetPosition(target);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        LiftMotor.setPower(speed);

        while (opModeIsActive() &&LiftMotor.isBusy()) {
            double max = speed;
            if (max > 1.0)
            {
                speed/=max;
            }
            LiftMotor.setPower(speed);
        }
        LiftMotor.setPower(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gyroDrive ( double speed, double distance)
    {
        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;

        if (opModeIsActive()) {
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = LeftFrontMotor.getCurrentPosition() + moveCounts;
            newLeftBackTarget = LeftBackMotor.getCurrentPosition()+moveCounts;
            newRightFrontTarget = RightFrontMotor.getCurrentPosition()+moveCounts;
            newRightBackTarget = RightBackMotor.getCurrentPosition()+moveCounts;

            LeftFrontMotor.setTargetPosition(newLeftFrontTarget);
            LeftBackMotor.setTargetPosition(newLeftBackTarget);
            RightFrontMotor.setTargetPosition(newRightFrontTarget);
            RightBackMotor.setTargetPosition(newRightBackTarget);

            LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            LeftFrontMotor.setPower(speed);
            LeftBackMotor.setPower(speed);
            RightFrontMotor.setPower(speed);
            RightBackMotor.setPower(speed);

            while (opModeIsActive() &&
                    (LeftFrontMotor.isBusy() && LeftBackMotor.isBusy()&&RightFrontMotor.isBusy()&&RightBackMotor.isBusy())) ;

                max = speed;
                if (max > 1.0)
                {
                    speed/=max;
                }
                drive(0,speed,0);

                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newLeftFrontTarget,  newLeftBackTarget,newRightFrontTarget,newRightBackTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      LeftFrontMotor.getCurrentPosition(),LeftBackMotor.getCurrentPosition(),RightFrontMotor.getCurrentPosition(),RightBackMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f",  speed);
                telemetry.update();
            }

            LeftFrontMotor.setPower(0);
            LeftBackMotor.setPower(0);
            RightFrontMotor.setPower(0);
            RightBackMotor.setPower(0);

            LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    public void gyroStraffe ( double speed,double distance)
    {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;

        if (opModeIsActive()) {

            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = LeftFrontMotor.getCurrentPosition() + moveCounts;
            newLeftBackTarget = LeftBackMotor.getCurrentPosition()-moveCounts;
            newRightFrontTarget = RightFrontMotor.getCurrentPosition()-moveCounts;
            newRightBackTarget = RightBackMotor.getCurrentPosition()+moveCounts;

            LeftFrontMotor.setTargetPosition(newLeftFrontTarget);
            LeftBackMotor.setTargetPosition(newLeftBackTarget);
            RightFrontMotor.setTargetPosition(newRightFrontTarget);
            RightBackMotor.setTargetPosition(newRightBackTarget);

            LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            LeftFrontMotor.setPower(speed);
            LeftBackMotor.setPower(speed);
            RightFrontMotor.setPower(speed);
            RightBackMotor.setPower(speed);

            while (opModeIsActive() &&
                    (LeftFrontMotor.isBusy() && LeftBackMotor.isBusy()&&RightFrontMotor.isBusy()&&RightBackMotor.isBusy())) ;

            max = speed;
            if (max > 1.0)
            {
                speed/=max;
            }

            drive(-speed,0,0);

            telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newLeftFrontTarget,  newLeftBackTarget,newRightFrontTarget,newRightBackTarget);
            telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      LeftFrontMotor.getCurrentPosition(),LeftBackMotor.getCurrentPosition(),RightFrontMotor.getCurrentPosition(),RightBackMotor.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f",  speed);
            telemetry.update();
        }

        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        RightBackMotor.setPower(0);

        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void motorInit()
    {
        LeftBackMotor = hardwareMap.dcMotor.get("rl");
        LeftFrontMotor = hardwareMap.dcMotor.get("fl");

        RightFrontMotor = hardwareMap.dcMotor.get("fr");
        RightBackMotor = hardwareMap.dcMotor.get("rr");

        RightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        RightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        LiftMotor2 = hardwareMap.dcMotor.get("LiftMotor2");

        ClawR = hardwareMap.crservo.get("Clawr");
        ClawL = hardwareMap.crservo.get("Clawl");

        IntLnkL = hardwareMap.servo.get("IntLnkL");
        IntLnkR = hardwareMap.servo.get("IntLnkR");

        IntakeR = hardwareMap.crservo.get("IntakeR");
        IntakeL = hardwareMap.crservo.get("IntakeL");

        LeftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void straffe(int ms, double power){
        runtime.reset();
        while(true) {
            drive(-power, 0, 0);
            if (runtime.milliseconds() >= ms){
                drive(0,0,0);
                runtime.reset();
                break;
            }
        }
    }

    public void driveForward(int ms, double power){
        runtime.reset();
        while(true) {
            drive(0, power, 0);
            if (runtime.milliseconds() >= ms){
                drive(0,0,0);
                runtime.reset();
                break;
            }
        }
    }

    public void driveBackward(int ms, double power){
        runtime.reset();
        while(true) {
            drive(0, -power, 0);
            if (runtime.milliseconds() >= ms){
                runtime.reset();
                drive(0,0,0);
                break;
            }
        }
    }

}
