# Wrist Zapper
A portable wrist pose controlled electrocutaneous stimulator. Part of an undergraduate thesis.

Operates on an ESP32, and communicates with a companion app built on Blynk via BLE.

![The Wrist Zapper](https://github.com/devsticks/wrist-zapper/raw/master/images/device_top.jpg "The Wrist Zapper")

If you'd like to make your own, see the build instructions [here](https://github.com/devsticks/wrist-zapper/tree/master/production_hardware/README.md).

## System Diagram

The following is a basic system diagram showing the various elements of the design and how they relate to one another:
<p align="center">
<img src="https://github.com/devsticks/wrist-zapper/raw/master/images/wiring_flow.jpg" alt="System diagram" width="80%"/>
</p>

## Bill of Materials

| Item                                                  | Part Number | Supplier  | Cost    | Quantity | Total Cost |
|-------------------------------------------------------|-------------|-----------|---------|----------|------------|
| Microcontroller (Adafruit ESP32 Feather)              | 3405        | Adafruit  | R299.25 | 1        | R299.25    |
| IMU (6 DOF)                                           | SEN-00138   | Netram    | R126.05 | 2        | R252.10    |
| Boost Convertor (12-200V, 60mA)                       | NCH6100HV   | Amazon    | R284.85 | 1        | R284.85    |
| Buck Convertor (12-3.3V, 2A)                          | MP1584      | Communica | R45.00  | 1        | R45.00     |
| NPN Transistor                                        | MJE340      | Farnell   | R10.66  | 1        | R10.66     |
| P-Channel MOSFET (400V, 3.5A)                         | FQP4P40     | Farnell   | R22.72  | 3        | R68.15     |
| SD Card Reader (Wemos D1 Mini RTC/Data Logger Shield) |             | Communica | R82.61  | 1        | R82.61     |
| Battery Manager (Comidox 3S, 12.6V, 10A)              |             | Amazon    | R38.95  | 1        | R38.95     |
| 3.7V Li-Ion Battery                                   |             | UCT       | R0.00   | 3        | R0.00      |
| Strip Board                                           |             | UCT       | R70.00  | 1        | R70.00     |
| On/Off Switch                                         | COM-00652   | Netram    | R15.00  | 1        | R15.00     |
| JST Connector (1A, 2 pin, female, 14cm)               |             | Communica | R3.04   | 4        | R12.17     |
| JST Connector (1A, 2 pin, male, 14cm)                 |             | Communica | R3.04   | 4        | R12.17     |
| Heat Shrink (Various colours, 1/8")                   |             | Communica | R0.44   | 3        | R1.32      |
| Header Connector (5 pin, female, 2.54mm, L, locking)  | XY136-05RT  | Communica | R0.91   | 2        | R1.82      |
| Header Connector (5 pin, male 2.54mm, locking)        | XY136-05HT  | Communica | R1.45   | 2        | R2.90      |
| Header Connector (2 pin, female, 2.54mm, L, locking)  | XY136-02RT  | Communica | R0.31   | 2        | R0.62      |
| Header Connector (2 pin, male 2.54mm, locking)        | XY136-02HT  | Communica | R0.55   | 2        | R1.10      |
| Ribbon Cable (10-way, 1m)                             |             | Communica | R7.36   | 0.1      | R0.74      |
| Neoprene (0.25m, 2mm)                                 |             | Coral     | R0.00   | 1        | R0.00      |
| 3D Printing                                           |             |           | R120.00 | 1        | R120.00    |
|                                                       |             |           |         |          |            |
| Total                                                 |             |           |         |          | R1,319.40  |
