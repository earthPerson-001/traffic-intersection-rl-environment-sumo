from genericpath import exists
import random
import os
from pathlib import Path
import sys

'''
This is specifically for double lane 4 way intersection.

The generated route file is network file dependent.
The network file, small-map-double-lane.rou.xml is used for now
'''

# constants
NUMBER_OF_TIME_STEPS: int = 100000

def generate_routefile(routefile=None):

    if(routefile == None):

        # to change to project relative paths and properly resolve paths in different platforms
        FILE = Path(__file__).resolve()
        ROOT = FILE.parents[1]  # traffic-intersection-rl-environment-sumo root directory

        routefilePath = ROOT.joinpath("sumo-files/").resolve()

        if not exists(routefilePath):
            os.mkdir(routefilePath)
        
        routefile = routefilePath.joinpath("random-route.rou.xml").resolve().__str__()

    # random.seed(42)  # make tests reproducible
    N = NUMBER_OF_TIME_STEPS  # number of time steps
    # demand per second from different directions
    
    pWN = 1. / 21
    pWE = 1. / 18
    pWS = 1. / 9
    pNE = 1. / 28
    pNS = 1. / 15
    pNW = 1. / 14
    pES = 1. / 13
    pEW = 1. / 10.5
    pEN = 1. / 16
    pSW = 1. / 22
    pSN = 1. / 7
    pSE = 1. / 11

    with open(routefile, "w") as routes:
        print("""<routes>
        <vType id="typeWN" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="3" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeWS" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2" maxSpeed="16.67" guiShape="motorcycle"/>
        <vType id="typeNE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="3" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeNW" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeES" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeEW" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="3" maxSpeed="16.67" guiShape="truck"/>
        <vType id="typeEN" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeSW" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeSN" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>   
        <vType id="typeSE" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>
        
        <route id="left_up" edges="E9 E8" />
        <route id="left_right" edges="E9 E10" />
        <route id="left_down" edges="E9 E11" />
        <route id="up_right" edges="-E8 E10" />
        <route id="up_down" edges="-E8 E11" />
        <route id="up_left" edges="-E8 -E9" />
        <route id="right_down" edges="-E10 E11" />
        <route id="right_left" edges="-E10 -E9" />
        <route id="right_up" edges="-E10 E8" />
        <route id="down_left" edges="-E11 -E9" />
        <route id="down_up" edges="-E11 E8" />
        <route id="down_right" edges="-E11 E10" />""", file=routes)
        lastVeh = 0
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWN:
                print('    <vehicle id="left_up_%i" type="typeWN" route="left_up" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="left_right_%i" type="typeWE" route="left_right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pWS:
                print('    <vehicle id="left_down_%i" type="typeWS" route="left_down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNE:
                print('    <vehicle id="up_right_%i" type="typeNE" route="up_right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="up_down_%i" type="typeNS" route="up_down" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNW:
                print('    <vehicle id="up_left_%i" type="typeNW" route="up_left" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pES:
                print('    <vehicle id="right_down_%i" type="typeES" route="right_down" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="right_left_%i" type="typeEW" route="right_left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEN:
                print('    <vehicle id="right_up_%i" type="typeEN" route="right_up" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pSW:
                print('    <vehicle id="down_left_%i" type="typeWS" route="down_left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pSN:
                print('    <vehicle id="down_up_%i" type="typeWN" route="down_up" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pSE:
                print('    <vehicle id="down_right_%i" type="typeWE" route="down_right" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
        print("</routes>", file=routes)
    return routefile

if __name__=="__main__":
    generate_routefile()