<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="8.1.1">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="ArduinoNanoV30">
<packages>
<package name="ARDUINO_NANO">
<description>&lt;b&gt;Arduino Nano V3.0 Module&lt;/b&gt;

&lt;p&gt;The Nano was designed and is being produced by &lt;a href="http://www.gravitech.us/arna30wiatp.html"&gt;Gravitech&lt;/a&gt;.&lt;/p&gt;&lt;br&gt;</description>
<hole x="-22.86" y="-7.62" drill="1.778"/>
<pad name="1" x="-20.32" y="-7.62" drill="0.8" shape="square"/>
<pad name="2" x="-17.78" y="-7.62" drill="0.8"/>
<pad name="3" x="-15.24" y="-7.62" drill="0.8"/>
<pad name="4" x="-12.7" y="-7.62" drill="0.8"/>
<pad name="5" x="-10.16" y="-7.62" drill="0.8"/>
<pad name="6" x="-7.62" y="-7.62" drill="0.8"/>
<pad name="7" x="-5.08" y="-7.62" drill="0.8"/>
<pad name="8" x="-2.54" y="-7.62" drill="0.8"/>
<pad name="9" x="0" y="-7.62" drill="0.8"/>
<pad name="10" x="2.54" y="-7.62" drill="0.8"/>
<pad name="11" x="5.08" y="-7.62" drill="0.8"/>
<pad name="12" x="7.62" y="-7.62" drill="0.8"/>
<pad name="13" x="10.16" y="-7.62" drill="0.8"/>
<pad name="14" x="12.7" y="-7.62" drill="0.8"/>
<pad name="15" x="15.24" y="-7.62" drill="0.8"/>
<pad name="16" x="15.24" y="7.62" drill="0.8"/>
<pad name="17" x="12.7" y="7.62" drill="0.8"/>
<pad name="18" x="10.16" y="7.62" drill="0.8"/>
<pad name="19" x="7.62" y="7.62" drill="0.8"/>
<pad name="20" x="5.08" y="7.62" drill="0.8"/>
<pad name="21" x="2.54" y="7.62" drill="0.8"/>
<pad name="22" x="0" y="7.62" drill="0.8"/>
<pad name="23" x="-2.54" y="7.62" drill="0.8"/>
<pad name="24" x="-5.08" y="7.62" drill="0.8"/>
<pad name="25" x="-7.62" y="7.62" drill="0.8"/>
<pad name="26" x="-10.16" y="7.62" drill="0.8"/>
<pad name="27" x="-12.7" y="7.62" drill="0.8"/>
<pad name="28" x="-15.24" y="7.62" drill="0.8"/>
<pad name="29" x="-17.78" y="7.62" drill="0.8"/>
<pad name="30" x="-20.32" y="7.62" drill="0.8"/>
<hole x="-22.86" y="7.62" drill="1.778"/>
<hole x="17.78" y="7.62" drill="1.778"/>
<hole x="17.78" y="-7.62" drill="1.778"/>
<wire x1="-24.13" y1="8.89" x2="17.78" y2="8.89" width="0.127" layer="21"/>
<wire x1="17.78" y1="8.89" x2="19.05" y2="8.89" width="0.127" layer="21"/>
<wire x1="19.05" y1="8.89" x2="19.05" y2="7.62" width="0.127" layer="21"/>
<wire x1="19.05" y1="7.62" x2="19.05" y2="3.81" width="0.127" layer="21"/>
<wire x1="19.05" y1="3.81" x2="19.05" y2="-3.81" width="0.127" layer="21"/>
<wire x1="19.05" y1="-3.81" x2="19.05" y2="-7.62" width="0.127" layer="21"/>
<wire x1="19.05" y1="-7.62" x2="19.05" y2="-8.89" width="0.127" layer="21"/>
<wire x1="19.05" y1="-8.89" x2="17.78" y2="-8.89" width="0.127" layer="21"/>
<wire x1="17.78" y1="-8.89" x2="-22.86" y2="-8.89" width="0.127" layer="21"/>
<wire x1="-22.86" y1="-8.89" x2="-24.13" y2="-8.89" width="0.127" layer="21"/>
<wire x1="-24.13" y1="-8.89" x2="-24.13" y2="-7.62" width="0.127" layer="21"/>
<wire x1="-24.13" y1="-7.62" x2="-24.13" y2="2.54" width="0.127" layer="21"/>
<wire x1="-24.13" y1="2.54" x2="-24.13" y2="8.89" width="0.127" layer="21"/>
<wire x1="-24.13" y1="2.54" x2="-25.4" y2="2.54" width="0.127" layer="21"/>
<wire x1="-25.4" y1="2.54" x2="-25.4" y2="-2.54" width="0.127" layer="21"/>
<wire x1="-19.05" y1="-2.54" x2="-19.05" y2="2.54" width="0.127" layer="21"/>
<wire x1="-19.05" y1="2.54" x2="-24.13" y2="2.54" width="0.127" layer="21"/>
<wire x1="-19.05" y1="-2.54" x2="-25.4" y2="-2.54" width="0.127" layer="21"/>
<text x="-17.78" y="0" size="1.27" layer="21" font="vector">&gt;NAME</text>
<text x="-17.78" y="-2.54" size="1.27" layer="21" font="vector">&gt;VALUE</text>
<circle x="0" y="0" radius="1.79605" width="0.127" layer="21"/>
<text x="2.54" y="0" size="0.8128" layer="21" font="vector" rot="R90" align="center">Reset</text>
<text x="-20.32" y="-6.35" size="1.016" layer="21" font="vector">1</text>
<text x="-21.59" y="0" size="0.6096" layer="21" font="vector" rot="R270" align="center">Mini-B
USB</text>
<wire x1="19.05" y1="3.81" x2="13.97" y2="3.81" width="0.127" layer="21"/>
<wire x1="13.97" y1="3.81" x2="13.97" y2="-3.81" width="0.127" layer="21"/>
<wire x1="13.97" y1="-3.81" x2="19.05" y2="-3.81" width="0.127" layer="21"/>
<circle x="17.78" y="-2.54" radius="0.8" width="0.127" layer="21"/>
<circle x="17.78" y="0" radius="0.8" width="0.127" layer="21"/>
<circle x="17.78" y="2.54" radius="0.8" width="0.127" layer="21"/>
<circle x="15.24" y="2.54" radius="0.8" width="0.127" layer="21"/>
<circle x="17.78" y="0" radius="0.8" width="0.127" layer="21"/>
<circle x="15.24" y="0" radius="0.8" width="0.127" layer="21"/>
<circle x="15.24" y="-2.54" radius="0.8" width="0.127" layer="21"/>
<text x="12.7" y="1.27" size="0.8128" layer="21" font="vector" rot="SR270">ICSP</text>
<text x="17.78" y="-4.445" size="1.016" layer="21" font="vector" rot="SR270">1</text>
<polygon width="0.127" layer="21">
<vertex x="15.24" y="6.35"/>
<vertex x="13.97" y="5.08"/>
<vertex x="16.51" y="5.08"/>
</polygon>
<polygon width="0.127" layer="21">
<vertex x="12.7" y="5.08"/>
<vertex x="13.97" y="6.35"/>
<vertex x="11.43" y="6.35"/>
</polygon>
<text x="-22.86" y="-1.27" size="0.0508" layer="21" font="vector" ratio="1" rot="R90">This library was created by Esp. Marco-Luis SALCEDO TOVAR.
Please report any errors or suggestions to: YV1HX@CANTV.NET
Thanks you!!!</text>
<wire x1="-24.13" y1="7.62" x2="-21.59" y2="7.62" width="0.015875" layer="21"/>
<wire x1="-22.86" y1="8.89" x2="-22.86" y2="6.35" width="0.015875" layer="21"/>
<wire x1="-24.13" y1="-7.62" x2="-21.59" y2="-7.62" width="0.015875" layer="21"/>
<wire x1="-22.86" y1="-6.35" x2="-22.86" y2="-8.89" width="0.015875" layer="21"/>
<wire x1="17.78" y1="-8.89" x2="17.78" y2="-6.35" width="0.015875" layer="21"/>
<wire x1="16.51" y1="-7.62" x2="19.05" y2="-7.62" width="0.015875" layer="21"/>
<wire x1="17.78" y1="6.35" x2="17.78" y2="8.89" width="0.015875" layer="21"/>
<wire x1="16.51" y1="7.62" x2="19.05" y2="7.62" width="0.015875" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="ARDUINO_NANO">
<description>&lt;b&gt;Arduino Nano V3.0 Module&lt;/b&gt;&lt;br&gt;&lt;br&gt;

&lt;b&gt;Overview:&lt;/b&gt;&lt;br&gt;

&lt;p&gt;The Arduino Nano is a small, complete, and breadboard-friendly board based on the ATmega328 (Arduino Nano 3.x) or ATmega168 (Arduino Nano 2.x). It has more or less the same functionality of the Arduino Duemilanove, but in a different package. It lacks only a DC power jack, and works with a Mini-B USB cable instead of a standard one.&lt;br&gt;
The Nano was designed and is being produced by &lt;a href="http://www.gravitech.us/arna30wiatp.html"&gt;Gravitech&lt;/a&gt;.&lt;/p&gt;&lt;br&gt;

&lt;table border="1" style="width:auto"&gt;
&lt;tr&gt;
  &lt;th colspan="2"&gt;&lt;b&gt;General specifications:&lt;/b&gt;&lt;/th&gt;
&lt;/tr&gt;
&lt;tr&gt;
    &lt;th&gt;Item&lt;/th&gt;
    &lt;th&gt;Value&lt;/th&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Microcontroller&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;Atmel ATmega168 or ATmega328&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Operating Voltage (logic level)&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;5 V&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Input Voltage (recommended)&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;7-12 V&lt;/center&gt;&lt;/td&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Input Voltage (limits)&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;6-20 V&lt;/center&gt;&lt;/td&gt;		
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Digital I/O Pins&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;14 (of which 6 provide PWM output)&lt;/center&gt;&lt;/td&gt;		
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Analog Input Pins&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;8&lt;/center&gt;&lt;/td&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;DC Current per I/O Pin&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;40 mA&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Flash Memory&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;16 KB (ATmega168) or 32 KB (ATmega328) of which 2 KB used by bootloader&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;SRAM&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;1 KB (ATmega168) or 2 KB (ATmega328)&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;EEPROM&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;512 bytes (ATmega168) or 1 KB (ATmega328)&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Clock Speed&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;16 MHz&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Dimensions (Inches) &lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;0.73" x 1.70"&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Dimensions (mm) &lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;18mm x 45mm&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Overall height&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;About 9 mm (not including pins)&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Weigth&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;5 g&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
&lt;/table&gt;
&lt;br&gt;&lt;br&gt;

&lt;b&gt;Power:&lt;/b&gt;&lt;br&gt;

The Arduino Nano can be powered via the Mini-B USB connection, 6-20V unregulated external power supply (pin 30), or 5V regulated external power supply (pin 27).&lt;br&gt;
The power source is automatically selected to the highest voltage source.&lt;br&gt;&lt;br&gt;

&lt;b&gt;Memory:&lt;/b&gt;&lt;br&gt;

The ATmega168 has 16 KB of flash memory for storing code (of which 2 KB is used for the bootloader); the ATmega328 has 32 KB, (also with 2 KB used for the bootloader).&lt;br&gt;
The ATmega168 has 1 KB of SRAM and 512 bytes of EEPROM (which can be read and written with the EEPROM library); the ATmega328 has 2 KB of SRAM and 1 KB of EEPROM.&lt;br&gt;&lt;br&gt;

&lt;b&gt;Input and Output:&lt;/b&gt;&lt;br&gt;

Each of the 14 digital pins on the Nano can be used as an input or output, using pinMode(), digitalWrite(), and digitalRead() functions.&lt;br&gt;
They operate at 5 volts.&lt;br&gt;
Each pin can provide or receive a maximum of 40 mA and has an internal pull-up resistor (disconnected by default) of 20-50 kOhms. In addition, some pins have specialized functions.&lt;br&gt;&lt;br&gt;

&lt;a href="https://www.arduino.cc/en/Main/ArduinoBoardNano"&gt;Visit Arduino - ArduinoBoardNano&lt;/a&gt;</description>
<wire x1="-15.24" y1="-15.24" x2="-15.24" y2="25.4" width="0.254" layer="94"/>
<wire x1="-15.24" y1="25.4" x2="-5.08" y2="25.4" width="0.254" layer="94"/>
<wire x1="-5.08" y1="25.4" x2="5.08" y2="25.4" width="0.254" layer="94"/>
<wire x1="5.08" y1="25.4" x2="15.24" y2="25.4" width="0.254" layer="94"/>
<wire x1="15.24" y1="25.4" x2="15.24" y2="-15.24" width="0.254" layer="94"/>
<wire x1="15.24" y1="-15.24" x2="-15.24" y2="-15.24" width="0.254" layer="94"/>
<pin name="D1/TX" x="-20.32" y="22.86" length="middle"/>
<pin name="D0/RX" x="-20.32" y="20.32" length="middle"/>
<pin name="!RESET@1" x="-20.32" y="17.78" length="middle" direction="in" function="dot"/>
<pin name="GND@1" x="-20.32" y="15.24" length="middle" direction="pwr"/>
<pin name="D2/INT0" x="-20.32" y="12.7" length="middle"/>
<pin name="D3/INT1" x="-20.32" y="10.16" length="middle"/>
<pin name="D4/T0" x="-20.32" y="7.62" length="middle"/>
<pin name="D5/T1" x="-20.32" y="5.08" length="middle"/>
<pin name="D6/AIN0" x="-20.32" y="2.54" length="middle"/>
<pin name="D7/AIN1" x="-20.32" y="0" length="middle"/>
<pin name="D8/CLK0" x="-20.32" y="-2.54" length="middle"/>
<pin name="D9/OC1A" x="-20.32" y="-5.08" length="middle"/>
<pin name="D10/OC1B/!SS" x="-20.32" y="-7.62" length="middle"/>
<pin name="D11/MOSI" x="-20.32" y="-10.16" length="middle"/>
<pin name="D12/MISO" x="-20.32" y="-12.7" length="middle"/>
<pin name="SCK/D13" x="20.32" y="-12.7" length="middle" rot="R180"/>
<pin name="3V3" x="20.32" y="-10.16" length="middle" direction="out" rot="R180"/>
<pin name="AREF" x="20.32" y="-7.62" length="middle" direction="in" rot="R180"/>
<pin name="ADC0/A0" x="20.32" y="-5.08" length="middle" rot="R180"/>
<pin name="ADC1/A1" x="20.32" y="-2.54" length="middle" rot="R180"/>
<pin name="ADC2/A2" x="20.32" y="0" length="middle" rot="R180"/>
<pin name="ADC3/A3" x="20.32" y="2.54" length="middle" rot="R180"/>
<pin name="ADC4/SDA/A4" x="20.32" y="5.08" length="middle" rot="R180"/>
<pin name="ADC5/SCL/A5" x="20.32" y="7.62" length="middle" rot="R180"/>
<pin name="ADC6/A6" x="20.32" y="10.16" length="middle" rot="R180"/>
<pin name="ADC7/A7" x="20.32" y="12.7" length="middle" rot="R180"/>
<pin name="5V" x="20.32" y="15.24" length="middle" direction="pwr" rot="R180"/>
<pin name="!RESET@2" x="20.32" y="17.78" length="middle" direction="in" function="dot" rot="R180"/>
<pin name="GND@2" x="20.32" y="20.32" length="middle" direction="pwr" rot="R180"/>
<pin name="VIN" x="20.32" y="22.86" length="middle" direction="pwr" rot="R180"/>
<wire x1="2.54" y1="-10.16" x2="-2.54" y2="-10.16" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-10.16" x2="-2.54" y2="-16.51" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-16.51" x2="2.54" y2="-16.51" width="0.254" layer="94"/>
<wire x1="2.54" y1="-16.51" x2="2.54" y2="-10.16" width="0.254" layer="94"/>
<text x="0" y="-12.7" size="0.8128" layer="94" font="vector" align="center">Mini-B
USB</text>
<circle x="0" y="0" radius="2.54" width="0.254" layer="94"/>
<text x="0" y="0" size="0.8128" layer="94" font="vector" rot="R180" align="center">RESET
BUTTON</text>
<text x="-15.24" y="-20.32" size="1.778" layer="95">&gt;NAME</text>
<text x="-15.24" y="-22.86" size="1.778" layer="96">&gt;VALUE</text>
<wire x1="-5.08" y1="25.4" x2="-5.08" y2="20.32" width="0.254" layer="94"/>
<wire x1="-5.08" y1="20.32" x2="5.08" y2="20.32" width="0.254" layer="94"/>
<wire x1="5.08" y1="20.32" x2="5.08" y2="25.4" width="0.254" layer="94"/>
<text x="-2.54" y="17.78" size="1.6764" layer="94" font="vector">ICSP</text>
<circle x="-2.54" y="21.59" radius="0.762" width="0.254" layer="94"/>
<circle x="0" y="24.13" radius="0.762" width="0.254" layer="94"/>
<circle x="2.54" y="24.13" radius="0.762" width="0.254" layer="94"/>
<circle x="-2.54" y="24.13" radius="0.762" width="0.254" layer="94"/>
<circle x="2.54" y="21.59" radius="0.762" width="0.254" layer="94"/>
<circle x="0" y="21.59" radius="0.762" width="0.254" layer="94"/>
<polygon width="0.254" layer="94">
<vertex x="-15.24" y="22.86"/>
<vertex x="-13.97" y="24.13"/>
<vertex x="-13.97" y="21.59"/>
</polygon>
<polygon width="0.254" layer="94">
<vertex x="-15.24" y="21.59"/>
<vertex x="-13.97" y="20.32"/>
<vertex x="-15.24" y="19.05"/>
</polygon>
<text x="6.35" y="24.13" size="0.8128" layer="94" font="vector" align="bottom-right">1</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="ARDUINO_NANO" prefix="ARDUINO_NANO" uservalue="yes">
<description>&lt;b&gt;Arduino Nano V3.0 Module&lt;/b&gt;&lt;br&gt;&lt;br&gt;

&lt;b&gt;Overview:&lt;/b&gt;&lt;br&gt;

&lt;p&gt;The Arduino Nano is a small, complete, and breadboard-friendly board based on the ATmega328 (Arduino Nano 3.x) or ATmega168 (Arduino Nano 2.x). &lt;br&gt;It has more or less the same functionality of the Arduino Duemilanove, but in a different package.&lt;br&gt;
It lacks only a DC power jack, and works with a Mini-B USB cable instead of a standard one.&lt;br&gt;
The Nano was designed and is being produced by &lt;a href="http://www.gravitech.us/arna30wiatp.html"&gt;Gravitech&lt;/a&gt;.&lt;/p&gt;&lt;br&gt;

&lt;table border="1" style="width:auto"&gt;
&lt;tr&gt;
  &lt;th colspan="2"&gt;&lt;b&gt;General specifications:&lt;/b&gt;&lt;/th&gt;
&lt;/tr&gt;
&lt;tr&gt;
    &lt;th&gt;Item&lt;/th&gt;
    &lt;th&gt;Value&lt;/th&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Microcontroller&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;Atmel ATmega168 or ATmega328&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Operating Voltage (logic level)&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;5 V&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Input Voltage (recommended)&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;7-12 V&lt;/center&gt;&lt;/td&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Input Voltage (limits)&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;6-20 V&lt;/center&gt;&lt;/td&gt;		
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Digital I/O Pins&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;14 (of which 6 provide PWM output)&lt;/center&gt;&lt;/td&gt;		
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Analog Input Pins&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;8&lt;/center&gt;&lt;/td&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;DC Current per I/O Pin&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;40 mA&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Flash Memory&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;16 KB (ATmega168) or 32 KB (ATmega328) of which 2 KB used by bootloader&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;SRAM&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;1 KB (ATmega168) or 2 KB (ATmega328)&lt;/center&gt;&lt;/td&gt;		
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;EEPROM&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;512 bytes (ATmega168) or 1 KB (ATmega328)&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Clock Speed&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;16 MHz&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Dimensions (Inches) &lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;0.73" x 1.70"&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Dimensions (mm) &lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;18mm x 45mm&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Overall height&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;About 9 mm (not including pins)&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
  &lt;tr&gt;
    &lt;td&gt;&lt;center&gt;Weigth&lt;/center&gt;&lt;/td&gt;
    &lt;td&gt;&lt;center&gt;5 g&lt;/center&gt;&lt;/td&gt;
  &lt;/tr&gt;
&lt;/table&gt;
&lt;br&gt;&lt;br&gt;

&lt;b&gt;Power:&lt;/b&gt;&lt;br&gt;

&lt;p&gt;The Arduino Nano can be powered via the Mini-B USB connection, 6-20V unregulated external power supply (pin 30), or 5V regulated external power supply (pin 27).&lt;br&gt;
The power source is automatically selected to the highest voltage source.&lt;/p&gt;&lt;br&gt;

&lt;b&gt;Memory:&lt;/b&gt;&lt;br&gt;

&lt;p&gt;The ATmega168 has 16 KB of flash memory for storing code (of which 2 KB is used for the bootloader); the ATmega328 has 32 KB, (also with 2 KB used for the bootloader).&lt;br&gt;
The ATmega168 has 1 KB of SRAM and 512 bytes of EEPROM (which can be read and written with the EEPROM library); the ATmega328 has 2 KB of SRAM and 1 KB of EEPROM.&lt;/p&gt;&lt;br&gt;

&lt;b&gt;Input and Output:&lt;/b&gt;&lt;br&gt;

&lt;p&gt;Each of the 14 digital pins on the Nano can be used as an input or output, using pinMode(), digitalWrite(), and digitalRead() functions.&lt;br&gt;
They operate at 5 volts.&lt;br&gt;
Each pin can provide or receive a maximum of 40 mA and has an internal pull-up resistor (disconnected by default) of 20-50 kOhms. In addition, some pins have specialized functions.&lt;/p&gt;&lt;br&gt;

&lt;a href="https://www.arduino.cc/en/Main/ArduinoBoardNano"&gt;Visit Arduino - ArduinoBoardNano&lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="ARDUINO_NANO" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ARDUINO_NANO">
<connects>
<connect gate="G$1" pin="!RESET@1" pad="18"/>
<connect gate="G$1" pin="!RESET@2" pad="13"/>
<connect gate="G$1" pin="3V3" pad="2"/>
<connect gate="G$1" pin="5V" pad="12"/>
<connect gate="G$1" pin="ADC0/A0" pad="4"/>
<connect gate="G$1" pin="ADC1/A1" pad="5"/>
<connect gate="G$1" pin="ADC2/A2" pad="6"/>
<connect gate="G$1" pin="ADC3/A3" pad="7"/>
<connect gate="G$1" pin="ADC4/SDA/A4" pad="8"/>
<connect gate="G$1" pin="ADC5/SCL/A5" pad="9"/>
<connect gate="G$1" pin="ADC6/A6" pad="10"/>
<connect gate="G$1" pin="ADC7/A7" pad="11"/>
<connect gate="G$1" pin="AREF" pad="3"/>
<connect gate="G$1" pin="D0/RX" pad="17"/>
<connect gate="G$1" pin="D1/TX" pad="16"/>
<connect gate="G$1" pin="D10/OC1B/!SS" pad="28"/>
<connect gate="G$1" pin="D11/MOSI" pad="29"/>
<connect gate="G$1" pin="D12/MISO" pad="30"/>
<connect gate="G$1" pin="D2/INT0" pad="20"/>
<connect gate="G$1" pin="D3/INT1" pad="21"/>
<connect gate="G$1" pin="D4/T0" pad="22"/>
<connect gate="G$1" pin="D5/T1" pad="23"/>
<connect gate="G$1" pin="D6/AIN0" pad="24"/>
<connect gate="G$1" pin="D7/AIN1" pad="25"/>
<connect gate="G$1" pin="D8/CLK0" pad="26"/>
<connect gate="G$1" pin="D9/OC1A" pad="27"/>
<connect gate="G$1" pin="GND@1" pad="19"/>
<connect gate="G$1" pin="GND@2" pad="14"/>
<connect gate="G$1" pin="SCK/D13" pad="1"/>
<connect gate="G$1" pin="VIN" pad="15"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="ARDUINO_NANO1" library="ArduinoNanoV30" deviceset="ARDUINO_NANO" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="ARDUINO_NANO1" gate="G$1" x="50.8" y="43.18"/>
</instances>
<busses>
</busses>
<nets>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="6.3" minversion="6.2.2" severity="warning">
Since Version 6.2.2 text objects can contain more than one line,
which will not be processed correctly with this version.
</note>
</compatibility>
</eagle>
