"""
Feb 2022
CoppeliaSim Line Follower
Asa Branca Aerospace  - Voyager

@author: Leonardo Silvino Brito
"""

import sim 


class Robot:
    def __init__(self, client_id, opmode):
        self.cid = client_id
        self.op = opmode

        # Wheel joints.
        self.wj_locs = ["Left","Right"]
        self.wjs = {}
        for loc in self.wj_locs:
            name = "Dynamic{0}Joint".format(loc)
            _,h = sim.simxGetObjectHandle(client_id, name, opmode)
            self.wjs[loc] = h

        # Sensores.
        self.vs_locs = ["Left","Middle","Right"]
        self.vss = {} # vss = vision sensors
        for loc in self.vs_locs:
            name = "{0}Sensor".format(loc)
            _,h = sim.simxGetObjectHandle(client_id, name, opmode)
            self.vss[loc] = h

    def set_wheel_vel(self, left, right):
        """Velocidade das rodas."""
        sim.simxSetJointTargetVelocity(self.cid, self.wjs['Left'], left, self.op)
        sim.simxSetJointTargetVelocity(self.cid, self.wjs['Right'], right, self.op)

    def get_wheel_vel(self):
        """Retorna as velocidades das rodas como uma lista (left,right)."""
        vels = []
        for loc in self.wj_locs:
            _,vel = sim.simxGetObjectFloatParameter(self.cid, self.wjs[loc], sim.sim_jointfloatparam_velocity, self.op)
            vels.append(vel)
        return vels

    def get_vis_sensors(self):
        """Retorna uma lista contendo uma media dos valores red/green/blue dos sensores"""
        aves = []
        for loc in self.vs_locs:
            h = self.vss[loc]
            _,_,pkt = sim.simxReadVisionSensor(client_id, h, opmode)
            aves.append(pkt[0][11:14])
        return aves


def is_black(sensor):
    """Retorna verdadeiro quando o sensor identifica o pathing"""
    #print(sensor[0],sensor[1],sensor[2]) #Identificar o valores das cores que o sensor esta lendo
    return (sensor[0] < 0.4 and sensor[2] < 0.4 and sensor[1] < 0.4) 



if __name__ == '__main__':
    sim.simxFinish(-1) # Stop any running simulation.
    client_id = sim.simxStart("127.0.0.1", 19997, True, True, 5000, 5)
    if client_id == -1:
        print("Failed to connect.")
        exit()
    print("Connected to CoppeliaSim.")
    print("Running robot logic, will exit when simulation is ended...")
    try:
        opmode = sim.simx_opmode_blocking
        sim.simxStartSimulation(client_id, opmode)
        robot = Robot(client_id, opmode)
        robot.set_wheel_vel(3.0, 3.0)
        sim.simxSynchronousTrigger(client_id)
        while True:
            # Very simple robot line following logic.
            l,m,r = robot.get_vis_sensors()
            if is_black(m):
                robot.set_wheel_vel(3.0, 3.0)
            elif is_black(l):
                robot.set_wheel_vel(0.6, 4.0)
            elif is_black(r):
                robot.set_wheel_vel(3.0, 0.6)
            sim.simxSynchronousTrigger(client_id)
    except:
        sim.simxFinish(client_id)
        print("Simulation ended.")
