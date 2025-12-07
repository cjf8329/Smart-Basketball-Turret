# Smart Basketball Return Turret

A mechatronic system that detects player position using IR sensors and automatically rotates a **ball-return turret** toward the shooter.  
Built with an STM32 microcontroller, stepper motor drive system, and a custom GT2 belt–driven rotating platform.

This project went through three major mechanical design iterations before reaching a reliable working prototype.

---

## Features

- **Left/Right player detection** using modulated IR + TSOP receivers  
- **Automatic turret rotation** using NEMA 17 stepper + A4988 driver  
- **GT2 belt drive** around a circular turret frame  
- **Turntable bearing** for smooth and rigid rotation  
- **Embedded C firmware** with simple state machine  
- **24V power system** with local buck regulation for MCU + sensors  

---

## How It Works

1. IR receivers detect which side of the hoop the shooter is on  
2. STM32 interprets the sensor state  
3. Stepper motor rotates the turret to the corresponding angle  
4. System returns to idle until the next shot attempt  
