; Home V Axis

;G90                     ; Set absolute mode
;G92 V0                  ; Define current position as 0 to enable move without homing
;G1 V300 H1 F3000        ; Move the axis to an endstop
;G1 V-6 F600
;G1 V15 H1 F300

G91                     ; Set relative mode
G1 V-360 F3000 H1       ; Big negative move to search for home endstop
G1 V6 F600              ; Back off the endstop
G1 V-15 F300 H1         ; Find endstop again slowly
G90                     ; Set absolute mode

