package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PullUp {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotor motor;
    private double tempPower = 0;

    public PullUp(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad1;

        motor = hardwareMap.get(DcMotor.class, "motor_up");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("PullUp: ", "Initialized");
    }

    public void tele() {
        if (tempPower != 0) {
            motor.setPower(tempPower);
            telemetry.addData("Pull_Up power: ", tempPower);
        } else {
            motor.setPower(-gamepad.right_stick_y);
            telemetry.addData("Pull_Up power: ", -gamepad.right_stick_y);
        }
        if (gamepad.a) {
            tempPower = -gamepad.right_stick_y;
        }

    }
}
