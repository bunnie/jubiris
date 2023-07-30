# Jubilee3D Duet Firmware Config Files

[Jubilee3D](https://github.com/machineagency/jubilee/) is the 3D
platform used to implement the IRIS reference scanner.

The specific instance of the Jubilee3D built for this research is a kit
purchased from [Filastruder](https://www.filastruder.com/collections/jubilee).
The exact parts in the kit seem to vary slightly over time, so for clarity,
the configuration used for this work has:

- A [Duet3 Mini 5+](https://www.filastruder.com/collections/jubilee) with a
[3HC expansion](https://www.filastruder.com/collections/jubilee).
- X/Y steppers that do 0.9 degrees per step (instead of the 1.8 indicated
in the `main` branch config files)
- X/Y steppers have their connector order swapped (this might just be a
quirk of this particular build, but the wiring order matches the drawing
in the Jubilee repo as of July 30, 2023).

This directory contains the Duet3D config files, adjusted for use with
an IRIS payload.
