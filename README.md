# IndyDualArm_Sim_cpp build

```

mkdir build
cd build
cp ../MR_info.json .
cmake ..
make -j16

```

# Start (Terminal 1)
```
  chmod 777  start_pybullet_server.sh
 ./start_pybullet_server.sh
```

# Start (Terminal 2)
```
  cd build
  ./pybullet_cpp_IndyDualArm
```
