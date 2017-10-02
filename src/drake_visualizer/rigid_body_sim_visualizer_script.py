import json
from director import lcmUtils
from director.utime import getUtime
import robotlocomotion as lcmrl

channel = 'RIGID_BODY_SIM_CONTROL'

def sendControlMessage(contents):
    msg = lcmrl.viewer2_comms_t()
    msg.utime = getUtime()
    msg.format = 'rigid_body_sim_json'
    msg.format_version_major = 0
    msg.format_version_minor = 1
    data = dict(**contents)
    msg.data = bytearray(json.dumps(data), encoding='utf-8')
    msg.num_bytes = len(msg.data)
    lcmUtils.publish(channel, msg)

def stopSimulation():
    sendControlMessage({'simulate': False})

toolBar = app.addToolBar("RigidBodySim.jl")
app.addToolBarAction(toolBar, 'Stop Simulation', icon=':/images/media/media-playback-stop.png', callback=stopSimulation)
