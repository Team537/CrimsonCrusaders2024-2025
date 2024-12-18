package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="PreMatchInit", group="Linear OpMode")
public class PreMatchInit extends LinearOpMode {
    /**
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void runOpMode() {
        Robot.getInstance().run(this, Constants.OpModes.PRE_MATCH_INIT);
    }
}
