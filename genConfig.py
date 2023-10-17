
import pandas as pd
import json



if __name__ == '__main__':

    fileName = 'orbital_0.0'
    path = '/home/tuna/Documents/driving/control/Comet/configFiles/' + fileName + '.json'


    info = dict({
        'support':{
            'length':           2.2,        #meter x
            'width':            1.4,        #meter y
            'height':           1.0,        #meter z
            },

        'robot':{
            'mass':             6.0,        #kg
            'baseWidth':        0.17,       #m
            'baseLength':       0.17,       #m
            'homePos':          [0, 0, 0],  #xyz
            'homeGim':          [0, 0],     #yaw, pitch
            'radius':           0.17,       #meters
            'pulleyID':         0.02,
            'drivePitchRadius': 0.126,      #m
            'pulleyPitchRadius':0.5,
            'cableThickness':   0.016
            },
        })

    j = json.dumps(info, indent=4)
    
    file = open(path, 'w') 

    file.write(j)

    file.close()
