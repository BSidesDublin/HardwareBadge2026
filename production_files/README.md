# Production Files

This folder contains all the necessary files to manufacture and assemble the PCB through JLCPCB.

## File Overview

- **`PCB/`**: Contains the Gerber files (`gerber.zip`)
- **`Assembly/`**: Includes the Bill of Materials (BOM) and Pick and Place (CPL) files required for SMT assembly.
- **`Firmware-JLC/`**: Contains the firmware binary formatted specifically for JLCPCB flashing services.

## PCB Specifications

| Feature | Specification |
| :--- | :--- |
| **Base Material** | FR-4 (TG135) |
| **Layers** | 2 |
| **Dimensions** | 55.72 mm * 93.91 mm |
| **Thickness** | 1.6mm |
| **PCB Color** | Black (White Silkscreen) |
| **Surface Finish** | ENIG (Gold Thickness: 1U") |
| **Outer Copper** | 1 oz |
| **Min Via Size** | 0.3mm / 0.45mm diameter |
| **Panel Format** | 1x3 Column/Row (71.72mm x 287.73mm) |
| **Edge Rails** | 8mm on left and right sides |
| **Via Covering** | Tented |

## JLCPCB Ordering Instructions

1. **Upload Gerbers**: Upload `PCB/gerber.zip` to the JLCPCB quote page.
2. **Select Specs**: Ensure the specifications match the table above (specifically **Black Color**, **ENIG Finish**, and **1.6mm Thickness**).
3. **SMT Assembly**: 
   - Enable the "SMT Assembly" option.
   - Upload the BOM: `Assembly/Bill of Materials-Harp.csv`.
   - Upload the CPL/Pick & Place: `Assembly/Pick Place for PCB1.csv`.
4. **Firmware Flashing**:
   - Enable "Function testing" during the assembly configuration, also mention it as a PCBA Remark.
   - Provide the files from the `Firmware-JLC/` folder. These files are pre-formatted for JLC requirements.
