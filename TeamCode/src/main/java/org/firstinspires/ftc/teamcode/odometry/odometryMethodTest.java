package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="odometryMethodTest")

public class odometryMethodTest extends LinearOpMode{

    double speed = 1;
    double zScale = 1;



    public void runOpMode() throws InterruptedException {


        odometryRobotHardware robot = new odometryRobotHardware(hardwareMap);

        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometryMethod odometry = new odometryMethod();

        DcMotor[] odometers = new DcMotor[3];
        {
            odometers[0] = robot.leftEncoder;
            odometers[1] = robot.rightEncoder;
            odometers[2] = robot.perpendicularEncoder;
        }

        waitForStart();

        while (opModeIsActive()) {

            robot.motorRF.setPower(((-gamepad1.right_stick_y - gamepad1.right_stick_x) - (-gamepad1.left_stick_x)));
            robot.motorRB.setPower((-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (-gamepad1.left_stick_x)));
            robot.motorLB.setPower(-((gamepad1.right_stick_y + gamepad1.right_stick_x) - (-gamepad1.left_stick_x)));
            robot.motorLF.setPower(-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (gamepad1.left_stick_x));

            odometry.refresh(odometers);

            telemetry.addData("X", odometry.GlobalX);
            telemetry.addData("Y", odometry.GlobalY);
            telemetry.addData("Heading", Math.toDegrees(odometry.GlobalHeading));

            telemetry.addData("motorRFPower", robot.motorRF.getPower());
            telemetry.addData("motorRBPower", robot.motorRB.getPower());
            telemetry.addData("motorLBPower", robot.motorLB.getPower());
            telemetry.addData("motorLFPower", robot.motorLF.getPower());

            telemetry.update();

        }
    }
}
