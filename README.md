# PET-Bottle-Recycler
<p align="center">
This project is meant to automate teadious aspects of the PET-bottle to-> 3D printer filament recycling process, commonly done via pultrusion.
</p>


## Background 
  I'm personally not very fond of single-use plastics, but fell inlove with rapid prototyping the moment I got my first 3D printer. Watching your designs being brought to life in a matter of minutes is highly-fulfilling and an incredible time saver. This fulfilling process became a bit adictive and worriesome however, when all the small amounts of wasted/unusable plastic prints started to stack up in a shoe box. Even though the PLA is "biodegradable" it still takes at least 80 years and perfect soil/environmental conditions for it to go away. Otherwise, if sent to landfil, it still behaves like any other plastic, slowly degrading in a 400 year timeframe. 
I thought, what if I made my prototypes fully recyclable instead? 
Print and iterate within a circular economy, knowing the prints were not being totally wasted sitting on a landfill for hundreds of yearsm, waiting for nothing to happen.

<p float="left">
  <img src= https://github.com/user-attachments/assets/6c8c6a1f-3834-42d0-b46f-9de8d0ae8431 width=30% height=10%>
  <img src= https://github.com/user-attachments/assets/6f958cd6-d4ad-4a85-b3c5-b9d49d7c2cd4 width=30% height=10%>  
  <img src= https://github.com/user-attachments/assets/d2438c79-360a-4039-8ee3-c3c59162be0f width=30% height=10%>
</p>


### PET-Pulling Resources 
- [CNC Kitchen Video](https://www.youtube.com/watch?v=N06FWr06iOI)
- [Bottle expansion (the process this machine has automated)](https://www.youtube.com/watch?v=xf8QNsDyYhE)
- Bottle Cutting (the hardest part to automate so far)

### Printables: 
- [Recreator 3D](https://www.printables.com/model/179820-the-recreator-3d-mk5kit-ender3-diy-3d-printer-fila)
- [funtion.3d](https://www.printables.com/@function3d)

### Online communities:
- [Recreator 3D](https://discord.gg/jwyxvpSK)
- [Polyformer](https://discord.gg/QkczNAeE)



## Project
__________________________________________
### CAD 

![image](https://github.com/user-attachments/assets/8637a0ce-67e1-45bc-95e3-14f4144887b1)




[INSERT VIDEO HERE]

__________________________________________
### Electronics 
To control the machine, I've been using an Arduino Mega 2560 board with the Ramps 1.4 shield (for convenience) as it provides an organized connection to all the necessary motor controllers, servo ports, as well as any 12V components like fans and heating elements. 

![image](https://github.com/user-attachments/assets/3d748647-cb8f-4ecc-8605-c2dd4e585f63)

### Software
  Code is writton on C++ through the Arduino IDE (for convenience), and using a couple supporting libraries.
#### Libraries
Aditional libraries used: 
- math.h
- Wire.h
- SparkFun_MicroPressure.h
- LiquidCrystal_I2C.h
- Servo.h.
./ Only one change was necessary in the libraries, and that was in the Servo.h timer selection (seen below):

<img src= https://github.com/user-attachments/assets/b09f10bd-e775-4755-91bb-31cdcb6c4958 width=50% height=50%>

Reason being, the stepper motors are connected on the Ramps 1.4 to specific timers (Timers 4, 3, and 1) leaving the servos to only be allowed for timers 5, 2 and 0. 

Feel free to re-arrange the timer alocation as you see fit, this just hppens to be what I've been doing, and is by no means the better alternative.





__________________________________________
### B.O.M
Here's a helpful [Google Sheets](https://docs.google.com/spreadsheets/d/1O_pJlcNmvoCJK8XPin97Yl5kds8EvCdPII7h_A2TWiM/edit?gid=0#gid=0) (pic for refference) serving as the BOM and spending tracker including every part I'm using. 

The expenses are calculated as a product sum of, the _estimated cost_, and the _quantity purchased_ + _shipping costs_. The _quantity needed_ is meant to signify how many of the items named you need for the project, regardless of how many come when prchased via the price listed. 

#### Example)
  The project requires 3 stepper motor controllers sensor, even though you get 4 on the listed Amazon order. Quantity needed is 3 but since the Amazon order includes 4, quantity purchase at $x.xx price from Amazon is then 1. Point of this   is to track what you need and what you purchase. 

Feel free to copy the file to your own drive and make any edits that best fit your use case. For instance, if you already have a part, I tend to mark it as "Purchased" with a $0.00 price as I've either borrowed it from older projects or dumpster-dove for it (lol)
 

![image](https://github.com/user-attachments/assets/c7400cf9-e3f5-4fa3-a4e6-ebf111968afb) 













