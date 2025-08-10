# 🚀 Closed-Loop Stepper Motor Control System  

A **high-performance closed-loop stepper motor system** powered by **Field-Oriented Control (FOC)** and **Space Vector Modulation (SVM)**, delivering **precise**, **smooth**, and **efficient motion control**.  

Real-time **encoder feedback** ensures **zero step loss**, even under heavy dynamic loads – making this system ideal for **robotics**, **CNC machines**, and **advanced automation** applications.  

---

## ✨ Features  

✅ **Closed-Loop Feedback** – Quadrature encoder eliminates step loss by constantly correcting position in real time.  
✅ **FOC (Field-Oriented Control)** – Decoupled torque & flux control for buttery-smooth motion.  
✅ **SVM (Space Vector Modulation)** – Generates efficient, optimized PWM waveforms.  
✅ **Dead-Time Insertion** – Prevents MOSFET shoot-through for safe, reliable switching.  
✅ **Multi-Mode Control**:  
   - 🎯 **Torque Mode** – Direct torque regulation via Q-axis current.  
   - ⚡ **Speed Mode** – Outer PI speed loop for consistent velocity.  
   - 📍 **Position Mode** – Expandable to precision positioning applications.  
✅ **Load Disturbance Compensation** – Maintains performance under sudden mechanical shocks.  

---

## 🛠 Hardware Setup  

- **Stepper Motor**: NEMA17 w/ integrated magnetic encoder  
- **Encoder**: Quadrature rotary encoder  
- **MCU**: TI **TMS320F28069M** (C2000 series – optimized for motor control)  
- **Driver**: Toshiba **TB67H400A** H-bridge driver  
- **Power Supply**: 24V SMPS  
- **Dev Board**: TI **LAUNCHXL-F28069M** + XDS110 Debugger  

---

## 🧠 Software Architecture  

- 🌀 **FOC Engine** – Implements **Clarke & Park transforms** for current regulation.  
- ⚙️ **Current Control Loop** – Inner PI loop for **Id/Iq** current control.  
- 🚀 **Speed Control Loop** – Outer PI loop regulates speed based on encoder feedback.  
- 🔺 **SVM** – Generates smooth PWM signals for precise voltage vector control.  
- ⏱ **Dead-Time Management** – Ensures MOSFET safety during commutation.  
- 🎯 **Encoder Interface (eQEP)** – Real-time speed & position tracking.  
- 📊 **Current Sensing & Calibration** – Phase current monitoring with ADC correction.  
- ⚡ **Interrupt-Driven Control** – High-speed ISR-based execution for ultra-low latency.  
- 🎛 **PWM Generation** – Configured for **10 kHz switching** using TI’s ePWM modules.  
- 🔧 **Utility Layer** – IQmath-based helpers for sign detection, clamping & timing.  

---

## 🎮 Control Modes  

### 🔴 Torque Mode  
Controls motor torque **directly** by regulating Q-axis current. Perfect for force-sensitive applications like **robot arms** or **actuators**.  

### 🟠 Speed Mode  
A **cascaded PI loop** sets torque (Iq) based on speed error – ensuring **steady, ripple-free velocity**.  

### 🟢 Position Mode *(Scalable)*  
An **outer position loop** feeds commands to the speed loop – enabling high-precision motion like **CNC**, **pick-and-place machines**, or **3D printers**.  

---

## 🔬 Prototype  

🖼 *Compact design built for performance testing*  

![Prototype](./Assets/Enclosure_!.jpg)  
![Prototype](./Assets/Enclosure_2.jpg)  

---

## ⚡ PCB Design  

A **4-layer PCB** designed for **thermal efficiency**, **signal integrity**, and **high-current handling**.  

- ✅ Optimized power & ground planes for **low-noise FOC**.  
- ✅ Carefully routed encoder & current sense lines for **accurate feedback**.  
- ✅ Built-in dead-time safe switching logic.  

![PCB](./Assets/pcb.jpg)  
![PCB](./Assets/pcb_2.jpg)  

---

## 📚 References  

📘 [TI – Field Oriented Control Overview](https://www.ti.com/video/6296584406001)  
📘 [TI – C2000 Motor Control User Guide](https://www.tij.co.jp/jp/lit/ug/spru556/spru556.pdf)  
📘 [TI – InstaSPIN FOC Documentation](https://www.ti.com/lit/pdf/sprui11)  
