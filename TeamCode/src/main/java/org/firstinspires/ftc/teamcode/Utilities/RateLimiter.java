package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class RateLimiter {

    //timer used to keep track of deltaTime
    private ElapsedTime timer = new ElapsedTime();
    private double previousTime = timer.seconds();

    // Fields for the rate limiter.
    private double lowerBound;  // The lower bound for rate limiting.
    private double upperBound;  // The upper bound for rate limiting.
    private double maxRate;     // The maximum rate allowed.
    private boolean continuous; // Whether the rate limiter is continuous (The upper bound IS the lower bound and can be crossed over)

    private double value;

    // Private constructor to initialize the fields through the builder.
    private RateLimiter(double lowerBound, double upperBound, double maxRate, boolean continuous, double initialValue) {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
        this.maxRate = maxRate;
        this.continuous = continuous;
        this.value = initialValue;
    }

    // Optional: Getters for each field, if needed externally.
    public double getLowerBound() {
        return lowerBound;
    }

    public double getUpperBound() {
        return upperBound;
    }

    public double getMaxRate() {
        return maxRate;
    }

    public boolean isContinuous() {
        return continuous;
    }

    public double getValue() {
        return value;
    }

    // Setter for lowerBound
    public void setLowerBound(double lowerBound) {
        this.lowerBound = lowerBound;
    }

    // Setter for upperBound
    public void setUpperBound(double upperBound) {
        this.upperBound = upperBound;
    }

    // Setter for maxRate
    public void setMaxRate(double maxRate) {
        this.maxRate = maxRate;
    }

    // Setter for continuous
    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    //Setter for value
    public void setValue(double value) {
        this.value = value;
    }


    // Static nested Builder class for constructing a RateLimiter instance.
    public static class Builder {

        // Builder variables to store values before building.
        private double lowerBound = 0.0;  // Default or user-provided lower bound.
        private double upperBound = 0.0;  // Default or user-provided upper bound.
        private double maxRate = 0;     // Default or user-provided max rate.
        private boolean continuous = false; // Default or user-provided continuity setting.
        private double value = 0.0;

        // Method to set the lower bound value.
        public Builder setLowerBound(double lowerBound) {
            this.lowerBound = lowerBound;
            return this;  // Allows chaining of methods.
        }

        // Method to set the upper bound value.
        public Builder setUpperBound(double upperBound) {
            this.upperBound = upperBound;
            return this;
        }

        // Method to set the maximum rate.
        public Builder setMaxRate(double maxRate) {
            this.maxRate = maxRate;
            return this;
        }

        // Method to set whether the rate limiter is continuous.
        public Builder setContinuous(boolean continuous) {
            this.continuous = continuous;
            return this;
        }

        public Builder setInitialValue(double value) {
            this.value = value;
            return this;
        }

        // Build method to create the final RateLimiter object.
        public RateLimiter build() {
            return new RateLimiter(lowerBound, upperBound, maxRate, continuous, value);
        }
    }

    public double limit(double target) {
        // Calculate the step size by multiplying the maxRate by deltaTime.
        double step = maxRate * getDeltaTime();

        // If the limiter is continuous, handle wrapping around the bounds.
        if (continuous) {
            // Calculate the midpoint for the continuous wrapping.
            double range = upperBound - lowerBound;
            double midPoint = (lowerBound + upperBound) / 2.0;

            // Normalize the difference between the target and current value within the bounds.
            double diff = ((target - value + midPoint) % range) - midPoint;

            // Apply the step in the direction of the target.
            if (diff > 0) {
                value = Math.min(value + step, target);
            } else {
                value = Math.max(value - step, target);
            }

            // Wrap the value if it goes beyond the bounds.
            if (value > upperBound) {
                value = lowerBound + (value - upperBound);
            } else if (value < lowerBound) {
                value = upperBound - (lowerBound - value);
            }
        } else {
            // Non-continuous behavior: move toward the target without wrapping.
            if (target > value) {
                value = Math.min(value + step, target);
            } else {
                value = Math.max(value - step, target);
            }

            // Clamp the value within the bounds.
            value = Math.max(lowerBound, Math.min(upperBound, value));
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