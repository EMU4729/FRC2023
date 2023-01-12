package frc.robot.utils;

import java.util.Optional;

import javax.swing.text.html.Option;

import com.fasterxml.jackson.annotation.OptBoolean;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class LEDControl {
  private static Optional<LEDControl> inst = Optional.empty();
  public static LEDControl getInstance() {
    if(!inst.isPresent()) inst = Optional.of(new LEDControl());
    return inst.get();
  }

  private Constants cnst = Constants.getInstance();

  private AddressableLED leds = new AddressableLED(cnst.LED_STRING_PORT);
  private AddressableLEDBuffer ledsBuff = new AddressableLEDBuffer(cnst.LED_STRING_LENGTH);

  private LEDControl(){
    leds.setLength(ledsBuff.getLength());
    leds.setData(ledsBuff);
    leds.start();
  }

  public void setLeds(int[] ledsToSet, int[] colour, int durationn, int piority){
    if(ledsToSet.length != 2) throw new IllegalArgumentException("ledsToSet length != 2");
    if(ledsToSet[0] < 0) throw new IllegalArgumentException("ledsToSet min < 0");
    if(ledsToSet[0] > cnst.LED_STRING_LENGTH) 
        throw new IllegalArgumentException("ledsToSet min > cnst.LED_STRING_LENGTH");
    if(colour.length != 3) throw new IllegalArgumentException("colour length != 3");
    if(colour.length != 3) throw new IllegalArgumentException("colour length != 3");
  }

  public void setAll(int[] colour, int duration, int priority){
    setLeds(new int[] {0,cnst.LED_STRING_LENGTH}, colour, duration, priority);
  }
}