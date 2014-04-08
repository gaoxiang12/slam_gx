==================================
  Running in Client/Server Mode
==================================

Running in client/server mode allows you to use FabMap from inside any C++ program.
To use this mode you'll need to install MOOS, a framework for interprocess communication.

=============================
     Installing MOOS
=============================
Installing MOOS should be painless!

First you'll need CMake, a cross-platform build system.
Get it here:
http://www.cmake.org/HTML/Download.html

Then get the latest version of MOOS
  preferably via subversion 
     svn://login2.robots.ox.ac.uk/MOOS/trunk
  alternatively from  
     http://www.robots.ox.ac.uk/~pnewman/TheMOOS/

To build MOOS then follow Part 1 of the brief install guide here:
http://www.robots.ox.ac.uk/~pnewman/TheMOOS/Build.htm

=========================================================================
Please note:
The binaries provided have been linked against MOOS version 2300.
=========================================================================

============================
      Running FabMap
============================

Your client program can post images and receive loop-closure signals via a simple API.
An example client program called FabMapClientExample.cpp is provided in the ClientServer directory.
Windows users will need to ensure that their compiler can find the MOOS header and library files.

To run in client-server mode, you will also need to start the FabMap server components.
Open a prompt in the ClientServer directory and run
   
    MOOSDB                                  - This provides inter-process comms.

You will also need to run the other FabMap components from the bin directory:

    WordMaker WordMaker_OnlineConfig.moos   - This listens for images and produces bag-of-words.
    FabMapV2    FabMapV2_OnlineConfig.moos      - This calculates the pdf.

Please note that the FabMap API has remained the same for version 2. However, the MOOS message received by the Client program has changed slightly from the last version. 
