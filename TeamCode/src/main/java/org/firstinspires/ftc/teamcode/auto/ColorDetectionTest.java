package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionTryPipeline;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Color Detection", group = "Test")
public class ColorDetectionTest extends LinearOpMode {

    private Bot bot;
    private double cycleTime = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gp1 = new GamepadEx(gamepad1);
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error code:", errorCode);
            }
        });
        camera.setPipeline(new ColorDetectionPipeline(telemetry));


        bot = Bot.getInstance(this);
        bot.initializeImus();
        boolean choice = false;
        while (!isStarted()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (choice) {
                    ColorDetectionTryPipeline colorDetectionPipeline = new ColorDetectionTryPipeline(telemetry);
                    camera.setPipeline(colorDetectionPipeline);
                } else {
                    ColorDetectionPipeline colorDetectionPipeline = new ColorDetectionPipeline(telemetry);
                    camera.setPipeline(colorDetectionPipeline);
                }
                choice = !choice;
            }

            //dpad up down does H high
            //left right does s high
            //bumper/trigger does v

//            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                colorDetect
//            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))  {
//
//            } if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//
//            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
//
//            } if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//
//            } else if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//
//            }



            telemetry.addLine("Press A to toggle pipeline");
            telemetry.addLine("");
            if (choice) {
                telemetry.addLine("Current Pipeline: Vig's ColorDetectionPipeline");
                telemetry.addData("Spikemark Status", ColorDetectionTryPipeline.spikeMark);
                telemetry.addData("width", ColorDetectionTryPipeline.width);
            } else {
                telemetry.addLine("Current Pipeline: ColorDetectionPipeline");
                telemetry.addData("Spikemark Status", ColorDetectionPipeline.spikeMark);
                telemetry.addData("width", ColorDetectionPipeline.width);

            }
            telemetry.update();
        }

        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) { }
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (choice) {
                telemetry.addLine("Current Pipeline: Vig's ColorDetectionPipeline");
                telemetry.addData("Spikemark Status", ColorDetectionTryPipeline.spikeMark);
                telemetry.addData("width", ColorDetectionTryPipeline.width);
                telemetry.addData("cycle", time - cycleTime);
                cycleTime = time;
            } else {
                telemetry.addLine("Current Pipeline: ColorDetectionPipeline");
                telemetry.addData("Spikemark Status", ColorDetectionPipeline.spikeMark);
                telemetry.addData("width", ColorDetectionPipeline.width);
                telemetry.addData("cycle", time - cycleTime);
                cycleTime = time;

            }
            telemetry.update();
        }







    }
}