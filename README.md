# Personal_Plastic_Extruder
This project implements the control system for a plastic extruder using an Arduino Mega. The system is designed to extrude recycled plastic into a filament of precise diameter by controlling various components such as motors, sensors, and cooling fans.

# Objective
The objective of this document is to present and explain in detail the software developed for the "Plastic Extruder" project. The purpose of this code is to control the various components of the system to achieve the extrusion of recycled shredded plastic while precisely adjusting the diameter of the extruded material. Throughout the report, each section of the code will be analyzed, demonstrating how it contributes to optimizing the extrusion process, ensuring efficiency, and properly controlling temperature, motors, and sensors involved.

# Main Functions
The primary goal of the plastic extruder is to produce a filament with a uniform and consistent diameter from shredded plastic. The process involves heating the material inside a cylinder, which contains a screw that pushes the molten plastic toward the exit. The cylinder is equipped with heating zones at both ends to ensure the plastic reaches an appropriate liquid state for extrusion. The solidification of the filament occurs immediately upon exiting the cylinder, thanks to environmental cooling and the action of two fans that help stabilize the process.

This process is carried out with the following functions in mind:

    Data Input: The system is calibrated with key data such as the desired fusion temperature and filament diameter.
    Plastic Loading: Shredded plastic is loaded into the machine via a hopper that feeds it into the cylinder, and the code starts once the user presses a button.
    Plastic Fusion: The plastic is heated to its melting point to prepare it for extrusion.
    Temperature Control: The temperature is regulated using sensors (thermocouples and thermistors) to maintain the optimal range for extrusion.
    Extrusion of Molten Plastic: The molten plastic is pushed through a nozzle to form a continuous filament of controlled size.
    Diameter Adjustment: The speed of the motor driving the screw is controlled to adjust the amount of plastic being extruded, thus managing the filament's diameter.
    Motor Synchronization: Stepper motors and the three-phase motor are synchronized to ensure smooth extrusion and avoid irregularities in the process.
    Filament Cooling: To maintain the appropriate diameter, the filament must be cooled immediately after exiting the nozzle. Two fans controlled by the Arduino are used for this purpose.
    Real-time Data Display: Key variables, such as the cylinder temperature and the speed of the three-phase motor, are displayed in real time on a 16x2 I2C LCD screen.
    Current Flow Display: Information regarding current flow is also monitored and displayed.
    Filament Winding: Two stepper motors are used in this section—one to guide the filament towards the winder, and the second to control the winding of the plastic onto a bobbin at the correct pace.
    Temperature and Speed Control: A 10k linear rotary potentiometer is used to control the temperature and speed, and a control panel-style button is used to confirm the settings.
