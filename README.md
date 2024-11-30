# BoilerOptimiser - retired
This is no longer an active project - see Boiler Control for replacement

Exploratory development code. Uses ESP32 to improve performance of a traditional gas boiler connected to a Tado heating control system.
Features:
- reads alalogue demand output from Tado to determine heating demand and boiler set temperature
- rate of change compensation on measured boiler temperature so no overheat
- anti cycling timer of boiler on/off
- shutdown sequence to dump residual heat in boiler to heating system
- opportunistic heating of hot water.
- 
All in progress at present.  
