package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase{

    private final AddressableLED m_led; 
    private final AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private int m_firstPixel = LEDConstants.kStrobeLength;
    private int m_lastPixel = 1;

    public LED(){
        m_led = new AddressableLED(0);

        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kStripLength);
        m_led.setLength(m_ledBuffer.getLength());
    }

    /**
     * Sets the color of the lights
     * @param r The red value (0-255)
     * @param g The green value (0-255)
     * @param b The blue value (0-255)
     */
    public void setColor(int r, int g, int b){
        //System.out.println("Target Color: (r: "+r+" g: "+g+" b: "+b+")");
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, r, g, b);
         }
        // Set the data
        SmartDashboard.putString("LED RGB", "r:" + r + " g: " + g + " b: " + b);
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * Blinks the lights with the set color
     * @param r The red value (0-255)
     * @param g The green value (0-255)
     * @param b The blue value (0-255)
     * @param time The total cycle time -start of on to end of off- in ms
     */
    public void blink(int r, int g, int b, double time){
        int br = r * (System.currentTimeMillis() % time > time/2 ? 1 : 0);
        int bg = g * (System.currentTimeMillis() % time > time/2 ? 1 : 0);
        int bb = b * (System.currentTimeMillis() % time > time/2 ? 1 : 0);
        setColor(br,bg,bb);
    }

    /**
     * Sets the lights to a shifting rainbow.
     * Good to use for a default command
     */
    public void rainbow() {
        
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;

        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    
    public void redShift() {
        final var hue = (m_rainbowFirstPixelHue + (m_firstPixel * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
        m_ledBuffer.setHSV(m_firstPixel, hue, 255, 128);
        m_ledBuffer.setRGB(m_lastPixel, 0, 0, 0);

        m_firstPixel++;
        m_firstPixel %= LEDConstants.kStripLength;
        m_lastPixel++;
        m_lastPixel %= LEDConstants.kStripLength;
    
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    
}
