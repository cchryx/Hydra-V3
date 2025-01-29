package components;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumDTControl {

    public DcMotor FR, FL, BR, BL;
    public Gamepad gamepad1;

    // Constructor to initialize the motors and gamepad
    public MecanumDTControl(DcMotor FR, DcMotor FL, DcMotor BR, DcMotor BL, Gamepad gamepad1) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        FR.setZeroPowerBehavior(BRAKE);
        FL.setZeroPowerBehavior(BRAKE);
        BR.setZeroPowerBehavior(BRAKE);
        BL.setZeroPowerBehavior(BRAKE);
    }


    public void move() {
        // Set controller input
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate setting motor power
        double FR_power = rx + (-y + x) / denominator;
        double BR_power = rx + (-y - x) / denominator;
        double FL_power = rx + (y - x) / denominator;
        double BL_power = rx + (y + x) / denominator;

        // Throttle speed
        if(gamepad1.right_trigger > 0) {
            FR_power *= Values.DT_SLOW_FACTOR;
            BR_power *= Values.DT_SLOW_FACTOR;
            FL_power *= Values.DT_SLOW_FACTOR;
            BL_power *= Values.DT_SLOW_FACTOR;
        }

        // Set motor powers
        FR.setPower(FR_power);
        BR.setPower(BR_power);
        FL.setPower(FL_power);
        BL.setPower(BL_power);
    }
}