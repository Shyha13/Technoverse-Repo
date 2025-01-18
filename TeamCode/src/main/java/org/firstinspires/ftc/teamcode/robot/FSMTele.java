package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp")
public class FSMTele extends LinearOpMode {
    public static double kP_i = 0.02;
    public static double kP_o = 0.01;
    public DcMotor back;

    public enum States {
        HOME, // where everything is idle and retracted
        SCORE_SPEC,
        INTAKE_SAMPLE,

    }

    States currentState = States.HOME;

    public static double intakeTarget = 0;
    public static double outtakeTarget = 0;

    public static double intakeOut = 750;
    public static double intakeIn = 0;

    public static double outtakeIn = 0;
    public static double outtakeSpec = 2600;

    CRServo bl; // back left gecko
    CRServo br; // back right gecko
    CRServo fl; // front left gecko
    CRServo fr; // front right gecko
    CRServo wrist; // wrist pivot point
    CRServo lm; // left misumi slide
    CRServo rm; // right misumi slide

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "lf_drive");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "lb_drive");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "rf_drive");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "rb_drive");

        // Initialize slide motors
        DcMotor back = hardwareMap.get(DcMotor.class, "back");
        DcMotor lr = hardwareMap.get(DcMotor.class, "lr");
        // Init servos
        bl = hardwareMap.get(CRServo.class, "br");
        br = hardwareMap.get(CRServo.class, "bl");

        fr = hardwareMap.get(CRServo.class, "fr");
        fl = hardwareMap.get(CRServo.class, "fl");
        wrist = hardwareMap.get(CRServo.class, "wrist");

        lm = hardwareMap.get(CRServo.class, "lm");
        rm = hardwareMap.get(CRServo.class, "rm");


        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        back.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure motor resists motion when idle
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            switch (currentState) {
                case HOME:
                    intakeTarget = intakeIn;
                    outtakeTarget = outtakeIn;

                    controlSpec(gamepad2);
                    controlSample(gamepad2);
                    controlIntake(gamepad2);

                    if (gamepad2.y) {
                        currentState = States.SCORE_SPEC;
                    }

                    if (gamepad2.left_bumper) {
                        currentState = States.INTAKE_SAMPLE;
                    }
                    break;

                case SCORE_SPEC:
                    outtakeTarget = outtakeSpec;
                    controlSpec(gamepad2);
                    controlSample(gamepad2);
                    if (gamepad2.right_bumper){
                        outtakeTarget += 200;
                    }

                    if (gamepad2.a) {
                        currentState = States.HOME;
                    }
                    break;

                case INTAKE_SAMPLE:
                    intakeTarget = intakeOut;
                    controlIntake(gamepad2);
                    controlWrist(gamepad2);
                    controlSample(gamepad2);

                    if (gamepad2.a) {
                        upIntake();
                        currentState = States.HOME;
                    }
                    break;

            }

            // drive
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double drive = gamepad1.left_stick_y;

            double lfPower = -Range.clip(-strafe - turn + drive, -1.0, 1.0);
            double rfPower = -Range.clip(-strafe - turn - drive, -1.0, 1.0);
            double lbPower = Range.clip(-strafe + turn - drive, -1.0, 1.0);
            double rbPower = -Range.clip(-strafe + turn + drive, -1.0, 1.0);

            frontLeftMotor.setPower(lfPower);
            backLeftMotor.setPower(lbPower);
            frontRightMotor.setPower(rfPower);
            backRightMotor.setPower(rbPower);

            back.setPower(outtakePid(outtakeTarget, back.getCurrentPosition()));
            lr.setPower(intakePid(intakeTarget, lr.getCurrentPosition()));

            telemetry.addData("Back Slides Position: ", back.getCurrentPosition());
            telemetry.addData("Intake Slides Position: ", lr.getCurrentPosition());
            telemetry.addData("State: ", currentState);
            telemetry.addData("slideL",lm.getPower());
            telemetry.addData("slideR",rm.getPower());
            telemetry.addData("wrist",wrist.getPower());
            telemetry.update();
        }
    }

    public double intakePid(double target, double current) {
        return (target - current) * kP_i;
    }

    public double outtakePid(double target, double current) {
        return (target - current) * kP_o;
    }


    // method cult
    public void intakeSpec() {
        bl.setPower(-1.0);
        br.setPower(1.0);
    }

    public void outtakeSpec() {
        bl.setPower(1.0);
        br.setPower(-1.0);
    }

    public void idleSpec() {
        bl.setPower(0);
        br.setPower(0);
    }

    public void outtakeSample() {
        fl.setPower(-1.0);
        fr.setPower(1.0);
    }

    public void intakeSample() {
        fl.setPower(1.0);
        fr.setPower(-1.0);
    }
    public void idleSample() {
        fl.setPower(0);
        fr.setPower(0);
    }

    public void leftWrist() {
        wrist.setPower(0.25);
    }

    public void rightWrist() {
        wrist.setPower(-0.25);
    }

    public void idleWrist() {
        wrist.setPower(0);
    }


    public void upIntake() {
        lm.setPower(1); // to be tuned cuz idk what the actual positions r
        rm.setPower(-1); // to be tuned cuz idk what the actual positions r
    }

    public void downIntake() {
        rm.setPower(1); // to be tuned cuz idk what the actual positions r
        lm.setPower(-1); // to be tuned cuz idk what the actual positions r

    }


    public void controlSpec(Gamepad gamepad) {
        if (gamepad2.b) {
            intakeSpec();
        } else if (gamepad2.x) {
            outtakeSpec();
        } else {
            idleSpec();
        }
    }

    public void controlSample(Gamepad gamepad) {
        if (gamepad2.right_stick_button) {
            intakeSample();
        } else if (gamepad2.left_stick_button) {
            outtakeSample();
        } else {
            idleSample();
        }
    }

    public void controlWrist(Gamepad gamepad) {
        if (gamepad2.dpad_left) {
            leftWrist();
        } else if (gamepad2.dpad_right) {
            rightWrist();
        } else {
            idleWrist();
        }
    }

    public void controlIntake(Gamepad gamepad) { //check
        if (gamepad2.dpad_down) {
            downIntake();
        } else if (gamepad2.dpad_up) {
            upIntake();
        }
        else{
            lm.setPower(0);
            rm.setPower(0);
        }



    }

}
