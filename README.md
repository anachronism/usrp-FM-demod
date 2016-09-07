# usrp-FM-demod
A real-time FM demodulator with audio output for the Ettus Research USRP E310.

#Cross-Compilation Instructions
All instructions are assuming you are using Linux and already have UHD installed.

1. Set up paths. This needs to be done every time a new terminal window is opened.
   ```
   source /usr/local/oecore-x86_64/environhment-setup-armv7ahf-vfp-neon-oe-linux-gnueabi
   ```
2. Navigate to this folder.
3. Make a build folder, and navigate into it.
   ```
   mkdir build
   cd build
   ```
4. Run Cmake.
   ```
  cmake -DCMAKE_TOOLCHAIN_FILE=/usr/local/oecore-x86_64/sysroots/x86_64-oesdk-linux/usr/share/cmake/OEToolchainConfig.cmake -DCMAKE_INSTALL_PREFIX=/usr -DENABLE_E300=ON ..
  
  OR
  
  cmake -DCMAKE_TOOLCHAIN_FILE=~/uhd/host/cmake/Toolchains/oe-sdk_cross.cmake -DCMAKE_INSTALL_PREFIX=/usr -DENABLE_E300=ON ..
   ```
5. Make the file.
6. Copy the file to the USRP.
