
import pandas as pd
import json



if __name__ == '__main__':

    fileName = 'orbital_0.0'
    path = '/home/tuna/Documents/driving/control/Comet/configFiles/' + fileName + '.json'


    info = dict({
        'support':{
            'length':           2.2,        #meter x length of table
            'width':            1.4,        #meter y length of table
            'height':           0.73,        #meter z height of support
            },

        'robot':{
            'mass':             1.7,        #kg
            'baseWidth':        0.17,       #m    
            'baseLength':       0.17,       #m
            'homePos':          [0, 0, 0],  #xyz
            'homeGim':          [0, 0],     #yaw, pitch
            'radius':           0.17,       #m radius of robot
            'pulleyID':         0.02,
            'drivePitchRadius': 0.126,      #m The radius of the drive gear
            'pulleyPitchRadius':0.5,        #m The radius at which pulley teeth mesh with drive gear
            'cableThickness':   0.016,
            'dampCoeff_x':      1,          #SYSTEM damping, this should be calulated through experiments
            'dampCoeff_y':      1,          #NOTE: this is the damping that will be inherent to the system,
            'dampCoeff_z':      6.,          #      these values will be used to solve for critically damped motion
            'dampCoeff_theta':  3.,
            'dampCoeff_phi':    3.,
            'dampCoeff_psi':    3.,
            'Ixx':              3.,
            'Iyy':              3.,
            'Izz':              3.,
            'spoolCoeffs':      [-131.9843, 64.01262, -0.059969, 0.000798947]
            },
        })

    j = json.dumps(info, indent=4)
    
    file = open(path, 'w') 

    file.write(j)

    file.close()
