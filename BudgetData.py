from agi.stk12.stkengine import *
from agi.stk12.stkobjects import *
from agi.stk12.stkutil import *
from agi.stk12.vgt import *

stk = STKEngine.StartApplication(noGraphics=False)

root = stk.NewObjectRoot()

# Create a new scenario
root.NewScenario('BudgetData')

# ground station setup
ground_station = root.CurrentScenario.Children.New(AgESTKObjectType.eFacility, "GroundStation")

root.UnitPreferences.Item('LatitudeUnit').SetCurrentUnit('deg')
root.UnitPreferences.Item('LongitudeUnit').SetCurrentUnit('deg')
ground_station.Position.AssignPlanetodetic(40,-95,0)

positions = ground_station.Position.QueryPlanetodeticArray()
gnd_location = [x for x in positions]


# satellite setup
refuler_sat = root.CurrentScenario.Children.New(AgESTKObjectType.eSatellite, 'Refueler')



# add sensor to ground station
sensor = ground_station.Children.New(AgESTKObjectType.eSensor, 'GndSensor')
sensor.CommonTasks.SetPointingTargetedTracking(AgETrackModeType.eTrackModeTranspond, AgEBoresightType.eBoresightLevel, '*/Satellite/Refueler')

# add uplink and downlink to sensor
