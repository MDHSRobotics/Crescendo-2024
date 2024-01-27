package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    private final AddressableLED m_led; 
    private final AddressableLEDBuffer m_ledBuffer;


    public LED(){
        m_led = new AddressableLED(5);

        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());
    }

    public void setColor(int r, int g, int b){
        System.out.println("Target Color: (r: "+r+" g: "+g+" b: "+b+")");
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, r, g, b);
         }
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
}
