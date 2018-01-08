package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "teleOp", group = "Drives")
public class teleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftBackMotor;
    private DcMotor RightBackMotor;
    private DcMotor LeftFrontMotor;
    private DcMotor RightFrontMotor;
    private DcMotor LiftMotor;
    private DcMotor LiftMotor2;
    private CRServo ClawL;
    private CRServo ClawR;
    private CRServo IntakeR;
    private CRServo IntakeL;
    private Servo IntLnkR;
    private Servo IntLnkL;
    private Servo jewel;
    double DriveTrainModifier = 1;
    boolean liftInt = false;
    boolean canLift;

    static final double COUNTS_PER_INCH_LIFT = (1120)/(6*3.1415);

    @Override
    public void init()
    {
        motorInit();
        jewel = hardwareMap.servo.get("jewelRight");
        jewel.setPosition(1);
        IntLnkL.setPosition(0.65);
        IntLnkR.setPosition(0.35);
    }


    public void loop()
    {
        runLift();
        runClaws();
        liftIntake();

        if(gamepad1.right_bumper)
            DriveTrainModifier = 0.5;
        else
            DriveTrainModifier = 1;


        drive((gamepad1.left_stick_x * DriveTrainModifier), (-gamepad1.left_stick_y * DriveTrainModifier), (gamepad1.right_stick_x * DriveTrainModifier));
        telemetry.addData("x: ",gamepad1.left_stick_x);
        telemetry.addData("y: ",-gamepad1.left_stick_y);
        telemetry.addData("r: ",gamepad1.right_stick_x);
        telemetry.update();
    }

    public void drive(double x, double y, double r) {
        double frValue = y - x -  r;
        double flValue = y + x +  r;
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
        telemetry.addData("flPower: ",flValue);
        telemetry.addData("frPower: ",frValue);
        telemetry.addData("rlPower: ",rlValue);
        telemetry.addData("rrPower: ",rrValue);
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
    }

    public void runLift()
    {
        if (gamepad1.left_bumper)
        {
            LiftMotor2.setPower(1);
            LiftMotor.setPower(1);
        }
        else if(gamepad1.left_trigger >= 0.3)
        {
            LiftMotor.setPower(-1);
            LiftMotor2.setPower(-1);
        }
        else
        {
            LiftMotor.setPower(0);
            LiftMotor2.setPower(0);
        }
    }

    public void runClaws()
    {
        if(gamepad2.right_bumper)
            IntakeR.setPower(1);
        else if(gamepad2.right_trigger >= 0.3)
            IntakeR.setPower(-1);
        else
            IntakeR.setPower(0);

        if(gamepad2.left_bumper)
            IntakeL.setPower(1);
        else if(gamepad2.left_trigger >= 0.3)
            IntakeL.setPower(-1);
        else
            IntakeL.setPower(0);


        if(gamepad1.x || gamepad2.b)
        {
            ClawL.setPower(1);
            ClawR.setPower(1);
        }
        else if(gamepad1.b || gamepad2.y)
        {
            ClawL.setPower(-1);
            ClawR.setPower(-1);
        }
        else
        {
            ClawL.setPower(0);
            ClawR.setPower(0);
        }
    }

    public void liftIntake()
    {
        if(!canLift) {
            if (runtime.milliseconds() < 500) return;
            canLift = true;
            runtime.reset();
        }
        if(gamepad2.a||gamepad1.a)
        {
            liftInt = !liftInt;
            canLift = false;
            runtime.reset();
        }

        if(liftInt)
        {
            IntLnkL.setPosition(0.65);
            IntLnkR.setPosition(0.35);
        }
        else
        {
            IntLnkL.setPosition(0);
            IntLnkR.setPosition(1);
        }
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

        while (LiftMotor.isBusy()) {
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
}

