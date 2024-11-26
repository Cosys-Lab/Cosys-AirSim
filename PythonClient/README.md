# Python API for AirSim

This package contains simple Python client for [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim). 
It can also be installed as a Python module. This integrates most API functions over RPC.

Note that this is renamed `cosysairsim` from the original `airsim` module. 

## Dependencies
This package depends on `numpy` and `msgpack` and would automatically install `numpy` and `rpc-msgpack` (this may need administrator/sudo prompt):
```
pip install numpy
pip install rpc-msgpack
```

## Installing from pip

You can install the Cosys-AirSim Python client from pip with `pip install cosysairsim`

## Installing Python Module from source
For this go into the _PythonClient_ folder of the Cosys-AirSim repository and use pip to install it to your python environment with `pip install .`

## More Info

More information on AirSim can be found at:
https://cosys-lab.github.io/

