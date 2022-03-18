package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class firstClass extends LinearOpMode{


        public DcMotor motorRF = null;
        public DcMotor motorLF = null;
        public DcMotor motorRB = null;
        public DcMotor motorLB = null;
        public DcMotor intake1 = null;
        public DcMotor intake2 = null;
        public DcMotor duckSpinnerLeft = null;
        public  DcMotor duckSpinnerRight = null;
        public Servo  baseRight = null;
        public Servo  armRight = null;
        public Servo  bucketRight = null;

        double speed = 1;
        double zScale = 1;


        public void runOpMode() throws InterruptedException
        {


            motorRF = hardwareMap.dcMotor.get("motorRF");
            motorLF = hardwareMap.dcMotor.get("motorLF");
            motorRB = hardwareMap.dcMotor.get("motorRB");
            motorLB = hardwareMap.dcMotor.get("motorLB");
            intake1 = hardwareMap.dcMotor.get("intake1");
            intake2 = hardwareMap.dcMotor.get("intake2");
            duckSpinnerLeft = hardwareMap.dcMotor.get("duckSpinnerLeft");
            duckSpinnerRight = hardwareMap.dcMotor.get("duckSpinnerRight");
            baseRight = hardwareMap.servo.get("baseRight");
            armRight = hardwareMap.servo.get("armRight");
            bucketRight = hardwareMap.servo.get("bucketRight");


            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            waitForStart();


            while (opModeIsActive())
            {

                motorRF.setPower(speed*((-gamepad1.right_stick_y - gamepad1.right_stick_x) - (zScale * gamepad1.left_stick_x)));
                motorRB.setPower(speed*(-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (zScale * gamepad1.left_stick_x)));
                motorLB.setPower(speed*((gamepad1.right_stick_y + gamepad1.right_stick_x) - (zScale * gamepad1.left_stick_x)));
                motorLF.setPower(speed*((-gamepad1.right_stick_x + gamepad1.right_stick_y)) - (zScale * gamepad1.left_stick_x));


                telemetry.addData("motorRFEncoder", motorRF.getCurrentPosition());
                telemetry.addData("motorRBEncoder", motorRB.getCurrentPosition());
                telemetry.addData("motorLBEncoder", motorLB.getCurrentPosition());
                telemetry.addData("motorLFEncoder", motorLF.getCurrentPosition());


                telemetry.update();

            }
        }



}
