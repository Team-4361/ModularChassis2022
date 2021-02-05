package frc.Library.Controllers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;

public class TalonEncoder
{
    WPI_TalonSRX usedTalon;
    int kTimeoutMs = 30;
    boolean kDiscontinuityPresent = false;
    int kBookEnd_0 = 910;
    int kBookEnd_1 = 1137;
    int pulsesPerRotation;
    double theDistancePerPulse;
    double theGearRatio;

    //Includes the position of the talon with the Encoder.
    public TalonEncoder(WPI_TalonSRX tal)
    {
        this.usedTalon = tal;
    }
    public void initQuadrature()
    {
		/* get the absolute pulse width position */
		int pulseWidth = usedTalon.getSensorCollection().getPulseWidthPosition();

		/**
		 * If there is a discontinuity in our measured range, subtract one half
		 * rotation to remove it
		 */
        if (kDiscontinuityPresent)
        {

			/* Calculate the center */
			int newCenter;
			newCenter = (kBookEnd_0 + kBookEnd_1) / 2;
			newCenter &= 0xFFF;

			/**
			 * Apply the offset so the discontinuity is in the unused portion of
			 * the sensor
			 */
			pulseWidth -= newCenter;
		}

		/**
		 * Mask out the bottom 12 bits to normalize to [0,4095],
		 * or in other words, to stay within [0,360) degrees 
		 */
		pulseWidth = pulseWidth & 0xFFF;

		/* Update Quadrature position */
		usedTalon.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
    }
    public void resetEncoder()
    {
        usedTalon.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    }
    public void setDistancePerPulse(double distance)
    {
        theDistancePerPulse = distance;
    }
    public void setPulses(int pulses)
    {
        this.pulsesPerRotation = pulses;
    }
    public double getDistancePerPulse()
    {
        return theDistancePerPulse;
    }
    public double getDistance()
    {
        return ((double)usedTalon.getSelectedSensorPosition()/(double)pulsesPerRotation)*(theDistancePerPulse);
    }
}