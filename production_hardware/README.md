# Build Instructions

You can make your very own Wrist Zapper - just be careful, you can really hurt yourself with one of these. 

If you want to get the cool stuff working first, follow this guide step-by-step. Otherwise go for it in any order - the steps aren't very dependent on one another. If you make your own Wrist Zapper please give me a shout - I'd love to see it!

# Table of Contents
1. [Device Electronics](#1-device-electronics)
2. [Microcontroller Code](#2-microcontroller-code)
3. [Smartphone App](#3-smartphone-app)
4. [Sleeve](#4-sleeve)
5. [Electrode](#5-electrode)
6. [Charger Electronics](#6-charger-electronics)
7. [Enclosures](#7-enclosures)
8. [Putting It Together](#8-putting-it-together)

## 1. Device Electronics

### Parts required

| Item                                                  | Part Number | Supplier  | Cost    | Quantity | Total Cost |
|-------------------------------------------------------|-------------|-----------|---------|----------|------------|
| Microcontroller (Adafruit ESP32 Feather)              | 3405        | Adafruit  | R299.25 | 1        | R299.25    |
| IMU (6 DOF)                                           | MPU6050     | Netram    | R126.05 | 2        | R252.10    |
| Boost Convertor (12-200V, 60mA)                       | NCH6100HV   | Amazon    | R284.85 | 1        | R284.85    |
| Buck Convertor (12-3.3V, 2A)                          | MP1584      | Communica | R45.00  | 1        | R45.00     |
| NPN Transistor (300V, 500mA, 20W)                     | MJE340      | Farnell   | R10.66  | 1        | R10.66     |
| P-Channel MOSFET (400V, 3.5A)                         | FQP4P40     | Farnell   | R22.72  | 3        | R68.15     |
| SD Card Reader (Wemos D1 Mini RTC/Data Logger Shield) |             | Communica | R82.61  | 1        | R82.61     |
| Battery Manager (Comidox 3S, 12.6V, 10A)              |             | Amazon    | R38.95  | 1        | R38.95     |
| Resistor (Through-hole, 25mW, 1k$\Omega$)             |             | UCT       | R0.00   | 4        | R0.00      |
| Resistor (Through-hole, 25mW, 2.2k$\Omega$)           |             | UCT       | R0.00   | 2        | R0.00      |
| Resistor (Through-hole, 25mW, 3.3k$\Omega$)           |             | UCT       | R0.00   | 1        | R0.00      |
| Resistor (Through-hole, 25mW, 10k$\Omega$)            |             | UCT       | R0.00   | 1        | R0.00      |
| LED (Red, 5mm, 20mA)                                  |             | UCT       | R0.00   | 1        | R0.00      |
| LED (Green, 5mm, 20mA)                                |             | UCT       | R0.00   | 1        | R0.00      |
| Fuse (glass, 30mA)                                    |             | UCT       | R0.00   | 1        | R0.00      |
| Fuse holder                                           |             | UCT       | R0.00   | 1        | R0.00      |
| 3.7V Li-Ion Battery                                   |             | UCT       | R0.00   | 3        | R0.00      |
| On/Off Switch                                         | COM-00652   | Netram    | R15.00  | 1        | R15.00     |
| JST Connector (1A, 2 pin, female, 14cm)               |             | Communica | R3.04   | 4        | R12.17     |
| JST Connector (1A, 2 pin, male, 14cm)                 |             | Communica | R3.04   | 4        | R12.17     |
| Heat Shrink (Various colours, 1/8")                   |             | Communica | R0.44   | 3        | R1.32      |
| Header Connector (5 pin, female, 2.54mm, L, locking)  | XY136-05RT  | Communica | R0.91   | 2        | R1.82      |
| Header Connector (5 pin, male 2.54mm, locking)        | XY136-05HT  | Communica | R1.45   | 2        | R2.90      |
| Header Connector (2 pin, female, 2.54mm, L, locking)  | XY136-02RT  | Communica | R0.31   | 2        | R0.62      |
| Header Connector (2 pin, male 2.54mm, locking)        | XY136-02HT  | Communica | R0.55   | 2        | R1.10      |
| Ribbon Cable (10-way, 1m)                             |             | Communica | R7.36   | 0.1      | R0.74      |
| Any small jumper cable (comms core will do, 1m)       |             | UCT       | R0.00   | 1        | R0.00      |
| Strip Board (2.54mm pitch, 100x300mm)                 |             | UCT       | R70.00  | 1        | R70.00     |
| | | | | | |
| **Total** |  |  |  |  | **R1,199.40** |

### Preparing the stripboard

Before soldering, you will need to remove portions of the stripboard in order to build the final circuit. The easiest way to mark out your stripboard is to download a copy of [this image](https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/electronics/circuit_production_inverted_flipped.jpeg) and crop it so that the grid spacing on your screen matches that of your stripboard. The image at the link perfectly matches 2.54mm stripboard on a 9.7"-sized iPad screen. 

Once you have the image sized up, place your stripboard over the screen (metal side up) and mark any spots with purple crosses for removal with a marker pen, as below (this is easier to see in real life than in the image...). The task is best done in a dark room with screen brightness maximised.

<p align="center">
  <img src=https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/images/circuit_marking.jpeg alt="Circuit Marking" width="40%">
  <img src=https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/images/circuit_marked.jpeg alt="Circuit Marked" width="40%">
</p>

Once marked, use a small drill bit to remove the contact strip at the relevant holes. I would highly recommend checking that you've broken continuity at the marked spots - you could easily destroy some expensive components by getting this wrong, and the further you go the harder it becomes to find these errors.

<p align="center">
  <img src=https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/images/circuit_removed.jpeg alt="Circuit Removed" width="40%">
</p>

The next step is to do a similar thing with [this image](https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/electronics/circuit_production_inverted.jpeg) to mark out the positions of joining wires. This time place the stripboard metal side down on your screen and draw lines over the red areas. 

### Soldering it up

Once you've marked the wires out, neatly solder the thin comms wires into place at these spots. As far as possible, keep the wires flat - there's not much vertical room for manoeuvre here. If you'd like to make your circuit board black (it just looks so much more professional!), now is the time to do so - I used a thick marker pen.

Prepare the buck and boost convertors by tuning them to give 3.3V and 200V respectively when supplied with a 12V input.

From this point it's just a matter of soldering things into place following the layout below. The board loayout is extremely cramped, and some components or breakout boards may need to be (carefully) filed or sanded down to fit. I would recommend starting with the shortest components first, then moving on to the tallest. Note that I've used female headers for the ESP32 and microSD card boards, while the MPU6050 (IMU) and NCH6100HV (Boost Convertor) were soldered straight on with male headers to save space (I know, I know...).

Before you solder in the NCH6100HV - probably the last part you'll add - test that things are on track by connecting a 12V supply to the input jack and switching on the board. If the current spikes as you do so switch it off immediately. If not, good job! Next, test that you're getting 3.3V - and no more! - at all the places you're expecting it, and 0 at all the ground pins. If that's all looking good, continue!

<p align="center">
  <img src=https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/electronics/circuit_production.jpeg" alt="Circuit Layout" width="40%">
</p>

The finished product should look something like this:
<p align="center">
  <img src=https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/images/circuit_complete_top.jpeg alt="Circuit Top" width="40%">
  <img src=https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/images/circuit_complete_side.jpeg alt="Circuit Side" width="40%">
</p>

If you've made it this far give someone a high five - you're doing really well! 

## 2. Microcontroller Code

## 3. Smartphone App

## 4. Sleeve

## 5. Electrode

## 6. Charger Electronics

## 7. Enclosures

## 8. Putting It Together
