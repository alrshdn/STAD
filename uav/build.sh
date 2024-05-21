export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:control/MAVSDK/build

cc engine.c sincere/sincere.c -L. -l:control/MAVSDK/build/libcontrol_mavsdk.so -o STAD
