# Starberry-Sense
[![DOI](https://data.caltech.edu/badge/110025475.svg)](https://doi.org/10.48550/arXiv.2207.03087)


[![DOI](https://data.caltech.edu/badge/110025475.svg)](
https://doi.org/10.1117/1.JATIS.8.3.036002)


Raspberry Pi-based star sensor development for small satellites and CubeSat missions in C/C++. The code is part of StarberrySense development at the Indian Institute of Astrophysics.

## Pre-requisite

1. OpenCV2 

    ```
    $ sudo apt install libopencv-dev
    ```
2.  [Astro-RaspiCam](https://github.com/Bharatcs/Astro-RaspiCam) 

## How to modify and use

1. Determine the field-of-view(FOV) followed by the magnitude limit of the instrument optics for the desired exposure time.
2. Using the catalogue, generate the search tables using the following procedures

    1. Modify the `mag_limit` (magnitude limit)value in `Catalog_generation/A_cat_gen.m` and run the program in Matlab/Octave to generate the star list `Catalog_generation/mag.txt`
        ```matlab
        %Enter magnitude limit
        mag_limit=4
        ```
    
    2. Use `Catalog generation/B_unit_vector_table.m` to generate the unit vector array file `unit_vector_table_for_cpp.txt` and `unit_vector_table.txt` inside `Catalog generation/unit_vector`.

    3. Now, in `C_lookup_tab.m` modify the minimum angular separation(`min_separation_arc_sec`) and maximum angular separation value(`max_separation_degree`) based on the system field-of-view(FOV) and run the code; this will generate the distance search table and store it as `angle_table_mag.txt` inside `Catalog generation/angle_table`.
        ```matlab
        %Specify the minimum and maximum separation
        min_separation_arc_sec=60
        max_separation_degree=50
        ```


    4. Now, using the `D_lookup_table_to_C_array.m`, convert the table to c array, which will be stored as `search_table_angle_for_cpp.txt` (angle values) and `search_table_id_for_cpp.txt` (star id pairs corresponding to angle values).

3. Now modify the `unit_vector_table.cpp` with the contents of `unit_vector_table_for_cpp.txt` (change array length and array) and modify the array length in unit_vector_table.hpp

4. Modify `angle_id.cpp` with contents of `search_table_id_for_cpp.txt` (change array length and array) and modify the array length in `angle_id.hpp` .

5. Modify `angle_table.cpp` with contents of `search_table_angle_for_cpp.txt` (change array length and array) and modify the array length in `angle_table.hpp`.

6. Now, in `basic_functions.cpp`, modify the camera parameters such as plate scale, pixel size and astrometry parameters obtained from `Astrometry.NET` obtained for the particular camera configuration with the centre pixel as reference.
[Astrometry.Net](https://nova.astrometry.net/upload)
    ```c
    #define number_of_stars_in_catalog 8874
    #define pixel_size_x_um 0.00112
    #define pixel_size_y_um 0.00112
    #define x_center (3280/4) 
    #define y_center (2464/4)
    // plate scale is 20.4446 arcsec/pix
    #define platescale_in_arcsec_per_pixel 20.4446
    //Astrometry correction
    const float A_0_0 = 0;
    const float A_0_1 = 0;
    const float A_0_2 = 0;
    const float A_1_0 = 0;
    const float A_1_1 = 0;
    const float A_2_0 = 0;

    const float B_0_0 = 0;
    const float B_0_1 = 0;
    const float B_0_2 = 0 ;
    const float B_1_0 = 0;
    const float B_1_1 = 0;
    const float B_2_0 = 0;

    const float CRPIX1 = 820.5;
    const float CRPIX2 = 616.5; 
    ```
7. Now, in `cam.cpp`, modify the parameters such as fov, catalogue size, exposure time and geometric voting error. Other parameters, if needed.
    ```c
    #define fov 9
    #define number_of_stars_in_catalog 8874
    #define EXPOSURE_TIME_IN_MS 5000
    #define COMPRESSION_FACTOR 50
    #define ERROR_IN_ARC_SEC 20
    ```
8. Modify the `angle_table_size` and `number_of_stars_in_catalog` in `hash.cpp` with angle search table size and catalog size.   
    ```c
    #define number_of_stars_in_catalog 8874
    #define angle_table_size 275876
    ```
9. Upload the code onto RPI and compile using the following command

    ```
    $ make star cam
    ```
## File Structure
* Catalog generation (use Matlab/Octave)
    * `A_cat_gen.m`   
        Program to generate a list of stars below a set magnitude with its RA and DEC (generates mag.txt)
    
    * `B_unit_vector_table.m` 
        
        Calculates the unit vector of stars from the list (mag.txt) generated by A_cat_gen.m and also converts it to C/C++ array format.
        Outputs two files in the unit_vector folder 


    * `C_lookup_tab.m`  

        Calculates the distance between all possible star pairs (Constrained by the Field of view) and sorts them in the order of the distances
        ```matlab
        %Specify the minimum and maximum separation 
        min_separation_arc_sec=40
        max_separation_degree=9
        ```
    * `D_lookup_table_to_C_array`  
        Converts the search table to a C/C++ array.

* Main (Requires GCC or g++)
    * `matrix.hpp/CPP`  
        Matrix operation functions
    * `makefile`             
        make file recipe 
    * `basic_function.hpp/CPP`
         Basic functions (unit vector calculation)
    * `cam.cpp`      
         Main program
    * unit_vector/
        * `unit_vector_table.hpp/CPP`
            
            contains the unit vector table for the catalogue stars
    * hash_table/
        * `angle_id.hpp/CPP`           
            Star pair ID search table mapped to the distance search table
        * `angle_table.hpp/CPP`          
            Distance search table for star pairs 
        * `hash.CPP/hpp`                 
            Binary search algorithm

##  References