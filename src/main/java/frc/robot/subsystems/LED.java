package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    private final AddressableLED m_led; 
    private final AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private int m_firstPixel = 40;
    private int m_lastPixel = 1;
    private int m_length = 300;

    public LED(){
        m_led = new AddressableLED(0);

        m_ledBuffer = new AddressableLEDBuffer(300);
        m_led.setLength(m_ledBuffer.getLength());
    }

    public void setColor(int r, int g, int b){
        //System.out.println("Target Color: (r: "+r+" g: "+g+" b: "+b+")");
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, r, g, b);
         }
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
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
        m_firstPixel %= m_length;
        m_lastPixel++;
        m_lastPixel %= m_length;
    
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    
}
