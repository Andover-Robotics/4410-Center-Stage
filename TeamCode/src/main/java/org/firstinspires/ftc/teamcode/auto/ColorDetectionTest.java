package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Test Color Detection", group = "Test")
public class ColorDetectionTest extends LinearOpMode {

    private Bot bot;
    private double cycleTime = 1;

    int alliance = 2;

//    while () {
//        telemetry.addData("moveDiff (positive is more ???)", moveDiff);
//        if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//            moveDiff -= 0.5;
//        } else if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//            moveDiff += 0.5;
//        }
//    }

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gp1 = new GamepadEx(gamepad1);
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        ColorDetectionPipeline colorDetectionPipeline = new ColorDetectionPipeline(telemetry);

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
        camera.setPipeline(colorDetectionPipeline);

        bot = Bot.getInstance(this);
        bot.initializeImus();

        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Spikemark Status", ColorDetectionPipeline.spikeMark);
            telemetry.addData("minwidth", ColorDetectionPipeline.minwidth);
            telemetry.addData("width", ColorDetectionPipeline.width);

            telemetry.addData("cycle", time - cycleTime);
            cycleTime = time;

            telemetry.update();
        }

        while (!isStarted()) {
            telemetry.addData("Spikemark Status", ColorDetectionPipeline.spikeMark);
            telemetry.addData("minwidth", ColorDetectionPipeline.minwidth);
            telemetry.addData("width", ColorDetectionPipeline.width);
            telemetry.update();
        }



        if (isStarted()) {
//            camera.stopStreaming();
            camera.closeCameraDevice();
        }

    }
}