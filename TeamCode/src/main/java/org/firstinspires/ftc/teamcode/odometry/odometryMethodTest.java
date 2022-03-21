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
        robot.resetDriveEncoders();
        odometryMethod odometry = new odometryMethod();

        waitForStart();

        while (opModeIsActive()) {

            robot.motorRF.setPower(speed*((-gamepad1.right_stick_y - gamepad1.right_stick_x) - (zScale * gamepad1.left_stick_x)));
            robot.motorRB.setPower(speed*(-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (zScale * gamepad1.left_stick_x)));
            robot.motorLB.setPower(speed*((gamepad1.right_stick_y + gamepad1.right_stick_x) - (zScale * gamepad1.left_stick_x)));
            robot.motorLF.setPower(speed*((-gamepad1.right_stick_x + gamepad1.right_stick_y)) - (zScale * gamepad1.left_stick_x));

            odometry.refresh();

            telemetry.addData("X", odometry.X);
            telemetry.addData("Y", odometry.Y);
            telemetry.addData("Heading", odometry.Heading);

            telemetry.addData("motorRFPower", robot.motorRF.getPower());
            telemetry.addData("motorRBPower", robot.motorRB.getPower());
            telemetry.addData("motorLBPower", robot.motorLB.getPower());
            telemetry.addData("motorLFPower", robot.motorLF.getPower());

            telemetry.update();

        }
    }
}
