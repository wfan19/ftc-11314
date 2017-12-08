package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by samsung on 2017/12/3.
 */
@Autonomous(name="Red", group="Linear Opmode")
//@Disabled
public class Red extends LinearOpMode {
    private static final int BLUE = 3;
    private ElapsedTime runtime = new ElapsedTime();

    boolean jewelFinished;
    byte[] colorCcache;
    int colorCNumber;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;

    int VuMarkID = -1;
    boolean VuMarkFinished = false;
    ClosableVuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    private DcMotor LeftBackMotor;
    private DcMotor RightBackMotor;
    private DcMotor LeftFrontMotor;
    private DcMotor RightFrontMotor;
    private Servo jewel;
    private DcMotor LiftMotor;
    private DcMotor LiftMotor2;
    private CRServo ClawL;
    private CRServo ClawR;
    private CRServo IntakeR;
    private CRServo IntakeL;
    private Servo IntLnkR;
    private Servo IntLnkL;


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_LIFT = (1120)/(6*3.1415);

    static final double     DRIVE_SPEED             = 0.3;     // Nominal speed for better accuracy.

    static final double jewelDistance = 2.5;
    static final int CENTER_DISTANCE = 27;
    static final int OFFSET = 7;

    static double distanceTraveled = 0.0;

    public void runOpMode(){
        motorInit();
        initializeVuMark(true);
        initializeColorSensors();
        jewel = hardwareMap.servo.get("jewel");
        telemetry.update();
        waitForStart();
        telemetry.log().clear();
        //lift(DRIVE_SPEED,4);
        IntLnkL.setPosition(0.9);
        runtime.reset();
        while(!jewelFinished) {
            jewel.setPosition(0.35);
            readColorNumberC();
            if(runtime.milliseconds()>2000) {
                jewelFinished=true;
                break;
            }
            if (colorCNumber != 0) hitJewel(false);
            displayColorNumber();
            telemetry.update();
        }
        jewel.setPosition(1);
        drive(0,0,0);
        telemetry.log().clear();
        telemetry.update();
        gyroDrive(DRIVE_SPEED,6);
        while(!VuMarkFinished) {
            gyroDrive(DRIVE_SPEED, 2);
            distanceTraveled += 2;
            if (distanceTraveled > 10) {
                VuMarkFinished = true;
                VuMarkID = 1;
                break;
            }
            runtime.reset();
            while (runtime.milliseconds() < 1000) {
                findVuMark();
            }
            displayVuMarkID();
            telemetry.update();
        }
        gyroDrive(DRIVE_SPEED,11-distanceTraveled);
        drive(0,0,0);
        straffe(1500,DRIVE_SPEED);
        drive(0,0,0);
        if(VuMarkID==1){
            gyroStraffe(DRIVE_SPEED,CENTER_DISTANCE+OFFSET);
        }
        else if(VuMarkID==2){
            gyroStraffe(DRIVE_SPEED,CENTER_DISTANCE);
        }
        else{
            gyroStraffe(DRIVE_SPEED,CENTER_DISTANCE-OFFSET);
        }
        runtime.reset();
        while(runtime.milliseconds()<1000) {
            drive(0, 0, 0);
        }
        //driveForward(200,DRIVE_SPEED);
        gyroDrive(DRIVE_SPEED,6);
        ClawL.setPower(-1);
        ClawR.setPower(-1);
        runtime.reset();
        while(runtime.milliseconds()<2000) {

        }
        gyroDrive(DRIVE_SPEED,-4);
        drive(0,0,0);
        gyroDrive(DRIVE_SPEED,8);
        drive(0,0,0);
        gyroDrive(DRIVE_SPEED,-3);
        ClawL.setPower(0);
        ClawR.setPower(0);
        if(VuMarkID==1){
            gyroStraffe(DRIVE_SPEED,-OFFSET);
        }
        if(VuMarkID==3){
            gyroStraffe(DRIVE_SPEED,OFFSET);
        }
        telemetry.update();
    }

    public void findVuMark(){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            if (vuMark == RelicRecoveryVuMark.LEFT) VuMarkID = 1;
            else if (vuMark == RelicRecoveryVuMark.CENTER) VuMarkID = 2;
            else VuMarkID = 3;
            VuMarkFinished = true;
            deactiveVuMark();
        }
    }

    public void deactiveVuMark(){
        relicTrackables.deactivate();
        vuforia.close();
    }

    public void initializeVuMark(boolean cameraEnabled){;
        VuforiaLocalizer.Parameters parameters;
        if(cameraEnabled)
            parameters = new VuforiaLocalizer.Parameters(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        else
            parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AbvEHy7/////AAAAGRmT0kowlEESsHWO52+1yK4RGV8QifRa3mmkwcy05kelV2/3fGxhhkPmURHfIgTA9Y2lw4zf1WZ5DkPv5DVfLSYy/cafwHkCriSeJep+5xwa2qRpMi3aDAncmXGB5ZoWw3hHK5upIXBj/aX9q9cNKN2ZcE05dMGaJp7ykHLfYXeVhL7fepFvOhLj8szG5kzE7myp2Lc2pgaDb8iGGhijznojUSg0GcSIQC7cEFRacsCUN7f4cIhHdK2c+Lv3sZP+NlyRliGoz8ICS4AgKDTXRVIFIDZ0h96T4rGjFxInT5JsK2tGqilGckVRcDXnGZPG6KR3LIdaa2T72mSoww/GusZhWrYSiS15oge6kr1Vv2UJ";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();
    }

    public void displayVuMarkID(){
        if(VuMarkID==1) telemetry.addData("VuMarkID", "LEFT");
        else if(VuMarkID==2) telemetry.addData("VuMarkID", "CENTER");
        else if(VuMarkID==3) telemetry.addData("VuMarkID", "RIGHT");
        else telemetry.addData("VuMarkID", "UNKNOWN");
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

    public void hitJewel(boolean isBlue){
        if(isBlue) {
            if (colorCNumber == BLUE) {
                gyroDrive(DRIVE_SPEED,jewelDistance+1);
                distanceTraveled+=1;
            }
            else {
                gyroDrive(DRIVE_SPEED,-jewelDistance);
                jewel.setPosition(1);
                gyroDrive(DRIVE_SPEED,jewelDistance*2);
            }
        }
        else{
            if(colorCNumber==BLUE) {
                gyroDrive(DRIVE_SPEED,-jewelDistance);
                jewel.setPosition(1);
                gyroDrive(DRIVE_SPEED,jewelDistance*2);
            }
            else{
                gyroDrive(DRIVE_SPEED,jewelDistance+1);
                distanceTraveled+=1;
            }
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
        colorC = hardwareMap.i2cDevice.get("color sensor C");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
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
            newLeftFrontTarget = LeftFrontMotor.getCurrentPosition() - moveCounts;
            newLeftBackTarget = LeftBackMotor.getCurrentPosition()+moveCounts;
            newRightFrontTarget = RightFrontMotor.getCurrentPosition()+moveCounts;
            newRightBackTarget = RightBackMotor.getCurrentPosition()-moveCounts;

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

            drive(speed,0,0);

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
            drive(power, 0, 0);
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
