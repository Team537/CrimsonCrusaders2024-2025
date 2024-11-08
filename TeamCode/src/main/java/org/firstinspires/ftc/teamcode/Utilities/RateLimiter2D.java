package org.firstinspires.ftc.teamcode.Utilities;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RateLimiter2D {

    //timer used to keep track of deltaTime
    private ElapsedTime timer = new ElapsedTime();
    private double previousTime = timer.seconds();

    //Fields for the rate limiter
    private double maxRate;

    private Vector2d value;

    /**
     * Constructor for the class
     * @param maxRate The rate at which the maximum increases
     * @param initialValue The initial value of the limiter
     */
    public RateLimiter2D(double maxRate, Vector2d initialValue) {
        this.maxRate = maxRate;
        this.value = initialValue;
    }

    public Vector2d limit(Vector2d target) {
        // Calculate the step size by multiplying the maxRate by deltaTime.
        double step = maxRate * getDeltaTime();

        Vector2d offset = target.minus(value);

        if (offset.magnitude() < step) {
            value = target;
        } else {
            value = value.plus(offset.normalize().times(step));
        }

        return value;

    }

    /**
     * Getting the deltaTime for the rate limiter
     * @return The delta Time
     */
    private double getDeltaTime() {
        double time = timer.seconds();
        double deltaTime = time - previousTime;
        previousTime = time;
        return deltaTime;
    }

}
