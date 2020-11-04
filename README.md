# Soil sensor
Everything is auto-configured from the SOC-Empty project from the BG22 Thunderboard Example Projects.

# Components to add
1. Simple Timer Service
2. Simple Button -> btn0 (PB01)
3. Simple LED -> led0 (PB00)
4. USART -> vcom (RX:PA06, TX:PA05)
5. Log
6. USTIMER
7. IADC

# Bluetooth GATT Configurator
Add a new service with a Analog Characteristic

1. New Service (Soil Sensor)
   1. Analog (2 bytes, Notify, Read)

# How to use
Replace the app.c file with the app.c file provided in the repo.

Compile the code and upload to a BG22 Thunderboard. You should see an output on a serial terminal of the ADC value, as well as the ADC counts.