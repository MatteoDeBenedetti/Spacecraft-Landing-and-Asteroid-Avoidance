#### For a detailed explanation see "LEM_DeBenedetti.pdf" 
  
  
### -- INSTALLATION --  
Enter in terminal "sudo apt‐get install liballegro4.2 liballegro4.2‐dev"
  
  
### -- FILES --  
"lem.c" contains the main faunction, tasks functions and a few user defined functions used  
"pthread.h and pthread.c" contain pthread related functions, structures and constants  
"allegro.h and allegro.c" contain allegro related functions, structures and constants  
"space.h and space.c" contain scenario related functions, structures and constants  
"data_log.txt" is the file generated running the application with lem telemetry data  
"CMakeLists.txt" is used to compile all the files, objects and the excutable  
  
  
### -- COMPILING --  
Move to the folder: /src  
Enter "Make -f CMakeLists.txt"
  
  
### -- EXECUTION --  
After compiling enter "sudo ./lem 0"
  
  
### -- SCENARIO SELECTION --  
entire mission "sudo ./lem 0"  
descent asteroid belt "sudo ./lem 1"  
landing "sudo ./lem 2"  
ascent asteroid belt "sudo ./lem 3"  
orbital insertion "sudo ./lem 4"  
docking "sudo ./lem 5"  
