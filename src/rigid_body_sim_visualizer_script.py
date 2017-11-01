import json
from director import lcmUtils
from director.utime import getUtime
import robotlocomotion as lcmrl
import bot_core as lcmbotcore
import inspect

controlChannel = 'RIGID_BODY_SIM_CONTROL'
timeChannel = 'RIGID_BODY_SIM_TIME'

def sendControlMessage(contents):
    msg = lcmrl.viewer2_comms_t()
    msg.utime = getUtime()
    msg.format = 'rigid_body_sim_json'
    msg.format_version_major = 1
    msg.format_version_minor = 1
    data = dict(**contents)
    msg.data = bytearray(json.dumps(data), encoding='utf-8')
    msg.num_bytes = len(msg.data)
    lcmUtils.publish(controlChannel, msg)

def terminate():
    sendControlMessage({'terminate': None})

class TimeDisplay:

    def __init__(self):
        self.widget = QtGui.QLabel('')
        self.widget.setFixedWidth(60)
        self.widget.setToolTip('Simulation time')
        self.widget.setAlignment(QtCore.Qt.AlignRight)
        self.setTime(0)
        self.subscriber = lcmUtils.addSubscriber(timeChannel, callback=self.onMessage)

    def setTime(self, t):
        self.widget.setText('{:.2f}'.format(t))

    def onMessage(self, msgBytes, channel):
        msg = lcmbotcore.utime_t.decode(msgBytes.data())
        self.setTime(msg.utime / 1000.)


toolBar = app.addToolBar("RigidBodySim.jl")
timeDisplay = TimeDisplay()
toolBar.addWidget(timeDisplay.widget)
app.addToolBarAction(toolBar, 'Stop Simulation/Playback', icon=':/images/media/media-playback-stop.png', callback=terminate)
# timeDisplay.setTime(1)
