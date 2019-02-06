
Focus API Examples
==================

Example programs using libfrcam and libfrdev to communicate with
nDepth device and camera systems.  The 'eagle' Linux driver
must be loaded.  'eagle' will indicate success or failure detecting
nDepth devices when loaded; use 'dmesg' to check.

If the host computer has multiple nDepth devices (multiple camera
systems), then which device to use can be selected via a -d command
line option when running opencv_simple or opencv_simple_color.
Other examples can be modified by specifying device index in the
call to FrOpenChannel(), and also using the appropriate calibration
file with FrOpenCalibMap().


opencv_simple_color   Displays depth map in color, one of the raw images, and
                      control panel for some parameters. Image display and GUI
                      implemented using OpenCV. Use this to test the system.

opencv_simple         Displays depth map in grey, one of the raw images, and
                      control panel for some parameters. Image display and GUI
                      implemented using OpenCV. Use this to test the system.

avi_simple            Constructs a short movie from images obtained from stereo
                      system. Raw depth.

avi_convert           Converts movie to nicer color display indicating depth.

rectcheck             Used by manufacturer to generate calibrations for
                      individual cameras. (Resulting calibration files have
                      been installed in /etc/fr.)

vcam_corr_img         Shows how images from source other than attached camera
                      can be uploaded to nDepth processor for processing.  
                      (In this example, they are static images loaded from files.)

v4l2_simple           Used older V4L2 interface to stereo system.
