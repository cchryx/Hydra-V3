package opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import components.HardwareInitializer;
import components.IntakeControl;
import components.MecanumDTControl;
import components.OuttakeControl;
import components.Values;

@TeleOp(group = "Actual", name = "WorldsOPMode_SPECIMEN")
public class WorldsOPMode_SPECIMEN extends OpMode {
    private HardwareInitializer hardwareInitializer;
    private MecanumDTControl mecanumDrive;
    private OuttakeControl outake;
    private IntakeControl intake;

    // Initialize hardware
    DcMotor FR, FL, BR, BL, VSLIDES_L, VSLIDES_R;
    Servo OUTPIVOT_L, OUTPIVOT_R, OUTROTATE, OUTWRIST, OUTCLAW, INTURRET, INPIVOT, INROTATE, INWRIST, INCLAW, HSLIDES_F, HSLIDES_B;

    private ElapsedTime autoTime = new ElapsedTime();
    private String autoProcess_d = "none", autoProcess_i = "none";
    private int autoStep_d = 0, autoStep_i = 0;
    boolean pChamber = false;
    boolean pBasket = false;
    boolean pHome = false;
    boolean pTransfer = false;
    boolean pSubmersible = false;
    boolean pDropoff = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        FR = hardwareInitializer.getMotor("FR");
        FL = hardwareInitializer.getMotor("FL");
        BR = hardwareInitializer.getMotor("BR");
        BL = hardwareInitializer.getMotor("BL");
        VSLIDES_L = hardwareInitializer.getMotor("VSLIDESL");
        VSLIDES_R = hardwareInitializer.getMotor("VSLIDESR");
        OUTPIVOT_L = hardwareInitializer.getServo("OUTPIVOTL");
        OUTPIVOT_R = hardwareInitializer.getServo("OUTPIVOTR");
        OUTROTATE = hardwareInitializer.getServo("OUTROTATE");
        OUTWRIST = hardwareInitializer.getServo("OUTWRIST");
        OUTCLAW = hardwareInitializer.getServo("OUTCLAW");
        INTURRET = hardwareInitializer.getServo("INTURRET");
        INPIVOT = hardwareInitializer.getServo("INPIVOT");
        INROTATE = hardwareInitializer.getServo("INROTATE");
        INWRIST = hardwareInitializer.getServo("INWRIST");
        INCLAW = hardwareInitializer.getServo("INCLAW");
        HSLIDES_F = hardwareInitializer.getServo("HSLIDESF");
        HSLIDES_B = hardwareInitializer.getServo("HSLIDESB");

        mecanumDrive = new MecanumDTControl(FR, FL, BR, BL, gamepad1);
        outake = new OuttakeControl(
                OUTPIVOT_L,
                OUTPIVOT_R,
                OUTROTATE,
                OUTWRIST,
                OUTCLAW,
                VSLIDES_L,
                VSLIDES_R,
                gamepad1,
                gamepad2,
                telemetry
        );

        intake = new IntakeControl(
                INTURRET,
                INPIVOT,
                INROTATE,
                INWRIST,
                INCLAW,
                HSLIDES_F,
                HSLIDES_B,
                gamepad1,
                gamepad2,
                telemetry
        );
        mecanumDrive.init();
        outake.init();
        intake.init();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        mecanumDrive.move();
        outake.move();
        intake.move(autoProcess_i, autoStep_i);

        // Auto Stuff
        boolean home = gamepad1.ps || gamepad2.ps;
        boolean chamber = gamepad1.y;
        boolean basket = gamepad2.b;
        boolean transfer = gamepad2.a;
        boolean submersible = gamepad2.x;
        boolean dropoff = gamepad2.y;


        if (chamber && !pChamber && autoStep_d == 0) {
            autoStep_d = 1;
            autoProcess_d = "chamber";
        }

        if (basket && !pBasket && autoStep_d == 0) {
            autoStep_d = 1;
            autoProcess_d = "basket";
        }

        if (submersible && !pSubmersible && autoStep_i == 0) {
            autoStep_i = 1;
            autoProcess_i = "submersible";
        }

        if (dropoff && !pDropoff && autoStep_i == 0) {
            autoStep_i = 1;
            autoProcess_i = "dropoff";
        }

        if (home && !pHome) {
            autoStep_d = 1;
            autoProcess_d = "home";
        }

        if (transfer && !pTransfer) {
            autoStep_d = 1;
            autoProcess_d = "transfer";
            autoStep_i = 1;
            autoProcess_i = "transfer";
        }

        switch (autoProcess_i) {
            case "home":
                switch (autoStep_i) {
                    case 1:
                        intake.clawTarget = Values.CLAW_OPENED;
                        intake.wristTarget = Values.INWRIST_INIT;
                        intake.pivotTarget = Values.INPIVOT_SUB + 0.1;
                        intake.rotateTarget = Values.INROTATE_SUB - 0.2;
                        intake.slidesTarget = Values.HSLIDES_MIN;

                        setAutoStep_i(10001);

                        break;
                    case 10001:
                        if(autoTime.milliseconds() > 300) {
                            intake.turretTarget = Values.INTURRET_INIT;

                            setAutoStep_i(0);
                            autoProcess_i = "none";
                        }
                        break;
                }
                break;
            case "dropoff":
                switch (autoStep_i) {
                    case 1:
                        if (dropoff && !pDropoff) {
                            intake.turretTarget = Values.INTURRET_DROP;
                            intake.pivotTarget = Values.INPIVOT_DROP;
                            intake.rotateTarget = Values.INROTATE_DROP;
                            intake.slidesTarget = Values.HSLIDES_MIN;

                            setAutoStep_i(10001);
                        }
                        break;
                    case 10001:
                        if (autoTime.milliseconds() > 100) {
                            autoProcess_d = "chamber";
                            setAutoStep_d(2);
                            outake.slidesTarget = Values.OUTSLIDES_MIN;
                            outake.clawTarget = Values.CLAW_OPENED;
                            outake.wristTarget = Values.OUTWRIST_GRAB;
                            outake.rotateTarget = Values.OUTROTATE_GRAB;
                            outake.pivotTarget = Values.OUTPIVOT_GRAB;

                            setAutoStep_i(2);
                        }
                        break;
                    case 2:
                        if (dropoff && !pDropoff) {
                            intake.clawTarget = Values.CLAW_OPENED;

                            setAutoStep_i(20001);
                        }
                        break;
                    case 20001:
                        if(autoTime.milliseconds() > 300) {
                            setAutoStep_i(1);
                            autoProcess_i = "home";
                        }
                        break;

                }
                break;
            case "transfer":
                switch (autoStep_i) {
                    case 1:
                        if (transfer && !pTransfer) {
                            intake.clawTarget = Values.CLAW_CLOSED;
                            intake.wristTarget = Values.INWRIST_INIT;
                            intake.rotateTarget = Values.INROTATE_INIT;
                            intake.pivotTarget = Values.INPIVOT_TRANSFER;
                            intake.turretTarget = Values.INTURRET_INIT;
                            intake.slidesTarget = Values.HSLIDES_MIN;


                            setAutoStep_i(2);
                        }
                        break;
                    case 2:
                        if (autoTime.milliseconds() > 100) {
                            setAutoStep_i(0);
                            autoProcess_i = "none";
                        }
                        break;
                }
                break;
            case "submersible":
                switch (autoStep_i) {
                    case 1:
                        if (submersible && !pSubmersible && outake.pivotRPosition < 0.6) {
                            intake.wristTarget = Values.INWRIST_INIT;
                            intake.clawTarget = Values.CLAW_OPENED;
                            intake.turretTarget = Values.INTURRET_INIT;
                            intake.pivotTarget = Values.INPIVOT_SUB;
                            intake.rotateTarget = Values.INROTATE_SUB;
                            intake.slidesTarget = Values.HSLIDES_SUB;

                            setAutoStep_i(2);
                        }
                        break;
                    case 2:
                        if (submersible && !pSubmersible) {
                            intake.clawTarget = Values.CLAW_CLOSED;
                            intake.pivotTarget = Values.INPIVOT_SUB_G;
                            intake.rotateTarget = Values.INROTATE_SUB_G;

                            setAutoStep_i(3);
                        }
                        break;
                    case 3:
                        if (submersible && !pSubmersible) {
                            intake.wristTarget = Values.INWRIST_INIT;
                            intake.turretTarget = Values.INTURRET_INIT;
                            intake.pivotTarget = Values.INPIVOT_SUB + 0.1;
                            intake.rotateTarget = Values.INROTATE_SUB - 0.2;

                            setAutoStep_i(0);
                            autoProcess_i = "none";
                        }
                        break;
                }
                break;
        }
        switch (autoProcess_d) {
            case "home":
                switch (autoStep_d) {
                    case 1:
                        outake.slidesTarget = Values.OUTSLIDES_MIN - 60;
                        outake.clawTarget = Values.CLAW_CLOSED;
                        outake.wristTarget = Values.OUTWRIST_INIT;
                        outake.rotateTarget = Values.OUTROTATE_INIT;
                        outake.pivotTarget = Values.OUTPIVOT_INIT;

                        setAutoStep_d(10001);
                        break;
                    case 10001:
                        if(autoTime.milliseconds() > 300 && outake.slidesPosition <= 0) {
                            VSLIDES_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            VSLIDES_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            VSLIDES_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            VSLIDES_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            outake.slidesTarget = Values.OUTSLIDES_MIN;

                            setAutoStep_d(10002);
                        }

                        if(autoTime.milliseconds() > 350) {
                            setAutoStep_d(10002);
                        }

                        break;
                    case 10002:
                        setAutoStep_d(0);
                        autoProcess_d = "none";
                        break;

                }

                break;
            case "transfer":
                switch (autoStep_d) {
                    case 1:
                        if (transfer && !pTransfer) {
                            outake.slidesTarget = Values.OUTSLIDES_MIN;
                            outake.clawTarget = Values.CLAW_OPENED;
                            outake.wristTarget = Values.OUTWRIST_MAX;
                            outake.rotateTarget = Values.OUTROTATE_TRANSFER;

                            setAutoStep_d(10001);
                        }
                        break;
                    case 10001:
                        if(autoTime.milliseconds() > 100) {
                            outake.pivotTarget = Values.OUTPIVOT_TRANSFER;

                            setAutoStep_d(10002);
                        }
                        break;
                    case 10002:
                        if (autoTime.milliseconds() > 100) {
                            setAutoStep_d(0);
                            autoProcess_d = "none";
                        }
                        break;
                }
                break;
            case "chamber":
                switch (autoStep_d) {
                    case 1:
                        if (chamber && !pChamber && autoTime.milliseconds() > 100) {
                            outake.slidesTarget = Values.OUTSLIDES_MIN;
                            outake.clawTarget = Values.CLAW_OPENED;
                            outake.wristTarget = Values.OUTWRIST_GRAB;
                            outake.rotateTarget = Values.OUTROTATE_GRAB;
                            outake.pivotTarget = Values.OUTPIVOT_GRAB;
                            setAutoStep_d(2);
                        }
                        break;
                    case 2:
                        if (chamber && !pChamber && autoTime.milliseconds() > 100) {
                            outake.clawTarget = Values.CLAW_CLOSED;

                            intake.turretTarget = Values.INTURRET_INIT;
                            intake.pivotTarget = Values.INPIVOT_INIT;
                            intake.wristTarget = Values.INWRIST_INIT;
                            intake.rotateTarget = Values.INROTATE_INIT;
                            intake.clawTarget = Values.CLAW_CLOSED;
                            intake.slidesTarget = Values.HSLIDES_INIT;
                            setAutoStep_d(20001);
                        }
                        break;
                    case 20001:
                        if (autoTime.milliseconds() > 100) {
                            outake.slidesTarget = Values.OUTSLIDES_GRAB;
                            setAutoStep_d(3);
                        }
                        break;
                    case 3:
                        if (chamber && !pChamber && autoTime.milliseconds() > 100) {
                            outake.slidesTarget = Values.OUTSLIDES_HCHAM;
                            outake.wristTarget = Values.OUTWRIST_MAX;
                            outake.rotateTarget = Values.OUTROTATE_HCHAM;
                            outake.pivotTarget = Values.OUTPIVOT_HCHAM;
                            setAutoStep_d(4);
                        }
                        break;
                    case 4:
                        if (chamber && !pChamber && autoTime.milliseconds() > 100) {
                            outake.pivotTarget = Values.OUTPIVOT_HCHAM_S;
                            outake.slidesTarget = outake.slidesPosition - 60;
                            setAutoStep_d(5);
                        }
                        break;
                    case 5:
                        if (chamber && !pChamber) {
                            outake.clawTarget = Values.CLAW_OPENED;
                            setAutoStep_d(50001);
                        }
                        break;
                    case 50001:
                        if(autoTime.milliseconds() > 500) {
                            setAutoStep_d(1);
                            autoProcess_d = "home";
                        }

                        break;
                }
                break;
            case "basket":
                switch (autoStep_d) {
                    case 1:
                        if (basket && !pBasket) {
                            intake.clawTarget = Values.CLAW_OPENED;
                            outake.clawTarget = Values.CLAW_CLOSED;
                            setAutoStep_d(2);
                        }
                        break;
                    case 2:
                        if (basket && !pBasket && autoTime.milliseconds() > 300) {
                            outake.pivotTarget = Values.OUTPIVOT_HBASK;
                            outake.slidesTarget = Values.OUTSLIDES_MAX;
                            outake.wristTarget = Values.OUTWRIST_MAX;

                            setAutoStep_d(20002);
                        }
                        break;
                    case 20002:
                        if (autoTime.milliseconds() > 100) {
                            outake.rotateTarget = Values.OUTROTATE_HBASK;
                            setAutoStep_d(3);
                        }
                        break;
                    case 3:
                        if (basket && !pBasket && autoTime.milliseconds() > 100) {
                            outake.clawTarget = Values.CLAW_OPENED;
                            setAutoStep_d(4);
                        }
                        break;
                    case 4:
                        if (basket && !pBasket) {
                            outake.pivotTarget = Values.OUTPIVOT_HBASK + 0.3;
                            setAutoStep_d(40001);
                        }
                        break;
                    case 40001:
                        if(autoTime.milliseconds() > 300) {
                            setAutoStep_d(1);
                            autoProcess_d = "home";
                        }
                        break;
                }
                break;
        }
        pChamber = chamber;
        pBasket = basket;
        pTransfer = transfer;
        pSubmersible = submersible;
        pDropoff = dropoff;
        pHome = home;

        outake.telemetry();
        intake.telemetry();

        telemetry.addData("autoStep_d", autoStep_d);
        telemetry.addData("autoProcess_d", autoProcess_d);
        telemetry.addData("autoStep_i", autoStep_i);
        telemetry.addData("autoProcess_i", autoProcess_i);
    }

    // Change step for deposit
    public void setAutoStep_d(int step) {
        autoStep_d = step;
        autoTime.reset();
    }

    // Change step for intake
    public void setAutoStep_i(int step) {
        autoStep_i = step;
        autoTime.reset();
    }
}