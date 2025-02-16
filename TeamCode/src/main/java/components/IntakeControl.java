package components;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
public class IntakeControl {

    public Servo INPIVOT, INROTATE, INWRIST, INTURRET, INCLAW, HSLIDES_F, HSLIDES_B;
    public Gamepad gamepad1, gamepad2;
    public Telemetry telemetry;


    public double
            turretTarget = Values.INTURRET_INIT,
            pivotTarget = Values.INPIVOT_INIT,
            wristTarget = Values.INWRIST_INIT,
            rotateTarget = Values.INROTATE_INIT,
            clawTarget = Values.CLAW_CLOSED,
            slidesTarget = Values.HSLIDES_INIT; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public double
            wristPosition = 0,
            pivotPosition = 0,
            turretPosition = 0,
            slidesFPosition = 0,
            slidesBPosition = 0,
            clawPosition = 0;

    public double throttle = 1;

    // Constructor to initialize the motors and gamepad
    public IntakeControl(
            Servo INTURRET,
            Servo INPIVOT,
            Servo INROTATE,
            Servo INWRIST,
            Servo INCLAW,
            Servo HSLIDES_F,
            Servo HSLIDES_B,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry

    ) {
        this.INTURRET = INTURRET;
        this.INWRIST = INWRIST;
        this.INPIVOT = INPIVOT;
        this.INROTATE = INROTATE;
        this.INCLAW = INCLAW;
        this.HSLIDES_F = HSLIDES_F;
        this.HSLIDES_B = HSLIDES_B;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        INTURRET.setPosition(turretTarget);
        INPIVOT.setPosition(pivotTarget);
        INROTATE.setPosition(rotateTarget);
        INWRIST.setPosition(wristTarget);
        INCLAW.setPosition(clawTarget);

        HSLIDES_F.setDirection(Servo.Direction.REVERSE);
        HSLIDES_B.setPosition(slidesTarget + Values.HSLIDES_OFFSET);
        HSLIDES_F.setPosition(slidesTarget);
    }

    public void move(String autoProcess_i) {
        wristPosition = INWRIST.getPosition();
        pivotPosition = INPIVOT.getPosition();
        turretPosition = INTURRET.getPosition();
        slidesFPosition = HSLIDES_F.getPosition();
        slidesBPosition = HSLIDES_B.getPosition();
        clawPosition = INCLAW.getPosition();


        if(gamepad2.left_bumper) {
            throttle = 0.1;
        } else {
            throttle = 1;
        }

        boolean slidesOut = gamepad2.dpad_up;
        boolean slidesIn = gamepad2.dpad_down;
        double pivot = -gamepad2.right_stick_y;
        double turret = -gamepad2.left_stick_x;
        double slides = -gamepad2.left_stick_y;

        if (slidesIn) {
            double i = Values.HSLIDES_INCR * throttle;
            double newPos = slidesFPosition - i;
            slidesTarget = Math.max(newPos, Values.HSLIDES_MIN);
        } else if (slidesOut) {
            double i = Values.HSLIDES_INCR * throttle;
            double newPos = slidesFPosition + i;
            slidesTarget = Math.min(newPos, Values.HSLIDES_MAX);
        }

        if(autoProcess_i.equals("submersible")) {
            if (slides < 0) {
                double i = slides * Values.HSLIDES_INCR * throttle;
                double newPos = slidesFPosition + i;
                slidesTarget = Math.max(newPos, Values.HSLIDES_MIN);
            } else if (slides > 0) {
                double i = slides * Values.HSLIDES_INCR * throttle;
                double newPos = slidesFPosition + i;
                slidesTarget = Math.min(newPos, Values.HSLIDES_MAX);
            }

            if (pivot > 0) {
                double i = Values.OUTPIVOT_INCR * throttle * pivot;
                double newPos = pivotPosition + i;
                pivotTarget = Math.min(newPos, Values.INPIVOT_MAX);
            } else if (pivot < 0) {
                double i = Values.INPIVOT_INCR * throttle * pivot;
                double newPos = pivotPosition + i;
                pivotTarget = Math.max(newPos, Values.INPIVOT_MIN);
            }

            if (turret > 0) {
                double i = Values.INTURRET_INCR * throttle * turret;
                double newPos = turretPosition + i;
                turretTarget = Math.min(newPos, Values.INTURRET_MAX);
            } else if (turret < 0) {
                double i = Values.INTURRET_INCR * throttle * turret;
                double newPos = turretPosition + i;
                turretTarget = Math.max(newPos, Values.INTURRET_MIN);
            }
        }

        if (gamepad2.right_trigger > 0) {
            double i = Values.INWRIST_INCR * throttle * gamepad2.right_trigger;
            double newWristPos = wristPosition + i;
            wristTarget = Math.min(newWristPos, Values.INWRIST_MAX);
        } else if (gamepad2.left_trigger > 0) {
            double i = Values.INWRIST_INCR * throttle * -gamepad2.left_trigger;
            double newWristPos = wristPosition + i;
            wristTarget = Math.max(newWristPos, Values.INWRIST_MIN);
        }

        INTURRET.setPosition(turretTarget);
        INPIVOT.setPosition(pivotTarget);
        INROTATE.setPosition(rotateTarget);
        INWRIST.setPosition(wristTarget);
        INCLAW.setPosition(clawTarget);
        HSLIDES_B.setPosition(slidesTarget + Values.HSLIDES_OFFSET);
        HSLIDES_F.setPosition(slidesTarget);
    }

    public void telemetry() {
        telemetry.addData("pivotPosition", pivotPosition);
        telemetry.addData("turretPosition", turretPosition);
        telemetry.addData("slidesFPosition", slidesFPosition);
        telemetry.addData("slidesBPosition", slidesBPosition);
        telemetry.addData("clawPosition", clawPosition);

    }
}