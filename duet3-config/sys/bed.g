M290 R0 S0                 ; Reset baby stepping
M561                       ; Disable any Mesh Bed Compensation
G1 H2 Z50 F5000            ; Lower the bed
G30 P0 X152.5 Y5 Z-99999   ; probe near back leadscrew
G1 H2 Z50 F5000            ; Lower the bed
G30 P1 X295 Y295 Z-99999   ; probe near front left leadscrew
G1 H2 Z50 F5000            ; Lower the bed
G30 P2 X5 Y295 Z-99999 S3  ; probe near front right leadscrew and calibrate 3 motors
G1 H2 Z50 F5000            ; Lower the bed
G29 S1                     ; Enable Mesh Bed Compensation
