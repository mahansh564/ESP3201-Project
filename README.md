# ESP3201-Project

## SSH Connection:

ssh robot@ev3dev.local
maker

## Python and lib setup:

Install virtual enviroment with Pyhton 3.5.10 and rpyc 3.3.0 
Install virtual enviroment with Pyhton 3.11


## Running the training:

Connect via shh to EV3

Then run:
```
sh rpyc_server.sh
```

In the Python 3.11 env run:
```
python plot.py
```

In the Python 3.5.10 env run:
```
python RunTraining.py
```



### Things tested:

big negative reward
negative reward on no movement actions
big statespace

varying all constants
