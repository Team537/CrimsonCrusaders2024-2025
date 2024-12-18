package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto", group="Linear OpMode")
public class Auto extends LinearOpMode {
    /**
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void runOpMode() {
        Robot.getInstance().run(this, Constants.OpModes.AUTO);
    }
}
