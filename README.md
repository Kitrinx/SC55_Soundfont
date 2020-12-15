# SC55 to Soundfont Converter
Written by Kitrinx and NewRisingSun.


# Requirements
You must have at least one set of SC55 wave roms and an SC55 control rom that match. Currently this has only been tested with two sets of roms:

The MKI 1.21 roms named:

```
roland_r15209363.ic23
roland-gss.a_r15209276.ic28
roland-gss.b_r15209277.ic27
roland-gss.c_r15209281.ic26
```

and the SCB (MKII) roms named:

```
SCB-55_R15279828_(program).BIN
R15209359_(samples1).BIN
R15279813_(samples2).BIN
```

To use the SCB-55 ROMs, uncomment MKII at the top.


# Building
Your compiler must be able to correctly handle packed structs, so if you are having difficulties with this, ensure that packed structs are working correctly. Windows GCC compilers may require the `-mno-ms-bitfields` option to do handle these structs correctly. This project has been primarily built in Linux, so windows builds may require extra attention.

To build, simply type:

`gcc -o sc55sf main.c -lm`

There is currently no makefile. If all the files are needed are present, executing the program should automagically generate a soundfont in the output folder.


# Known Issues
There are many known issues as this is a WIP and currently rapidly changing code only intended for experimentation and discovery and not yet cleaned up or prepared for an end user. Currently the TVP and TVF and many as-of-yet unknown parameters are not mapped out fully or handled.


# Other Documentation
There is currently a google spreadsheet [here](https://docs.google.com/spreadsheets/d/13LyKT-0czQfSz1jF42GhiQShXHAfGFzOqpFD5J0UZf0/edit?usp=sharing) in which we are documentating various discoveries of parameters. It's currently unfinished, but may be helpful to others exploring the parameters.

