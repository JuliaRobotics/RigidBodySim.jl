from director import lcmUtils

def stopSimulation():
    print('stop')

toolBar = applogic.findToolBar('Main Toolbar')
app.addToolBarAction(toolBar, 'Stop', icon='icons/Farm-Fresh_control_stop.png', callback=stopSimulation)
