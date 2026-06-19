<img width="800" height="450" alt="IMG_6344-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/e2323455-e232-491c-b63e-09754b9f0e52" />

# 4-Bar Crank-Rocker Linkage Simulation

A kinematic simulation and 3D CAD model of a classic 4-bar crank-rocker mechanism designed and analyzed in SolidWorks.

## Mechanism Overview
<img width="800" height="450" alt="IMG_6344-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/761fb97d-fb27-4e2b-baf4-f4897946bfb7" />


This repository features a 4-bar linkage mechanism satisfying **Grashof's Criterion**. It operates as a **crank-rocker**, transforming continuous rotational input into oscillating angular output.

### Kinematic Specifications
* **Input Speed:** 100 RPM (Constant)
* **Linkage Dimensions:**
  * Crank (Shortest Link): 40 mm
  * Ground Link / Base: 160 mm
  * Coupler Link: 120 mm
  * Rocker (Output Link): 100 mm

### Grashof Verification
According to Grashof's Law, for a continuous relative motion between links, the sum of the shortest ($s$) and longest ($l$) links must be less than or equal to the sum of the remaining two links ($p + q$):

$$s + l \le p + q$$

Given our dimensions, this mechanism satisfies the inequality, enabling the 40mm link to act as a fully rotating crank ($360^\circ$).

## Repository Structure
* `/CAD`: Contains original SolidWorks assembly (`.SLDASM`) and part files (`.SLDPRT`).
* `/Exports`: Universal CAD formats (`.STEP`, `.x_t`) for cross-platform compatibility.

## Software Used
* **CAD & Simulation:** SolidWorks 2025(Motion Analysis Toolset)
