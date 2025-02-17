from agi.stk12.stkengine import *
from agi.stk12.stkdesktop import STKDesktop
from agi.stk12.stkobjects import *
from agi.stk12.stkutil import *
from agi.stk12.vgt import *

import json

network_json = "atlasNetwork/atlas.json"

stk = STKEngine.StartApplication(noGraphics=False)
root = stk.NewObjectRoot()

#stk = STKDesktop.StartApplication(visible=True, userControl=True)
#root = stk.Root

# Create a new scenario
root.NewScenario('NGSP_Link_Budget')

root.UnitPreferences.SetCurrentUnit("Distance", "km")
root.UnitPreferences.SetCurrentUnit("Time", "sec")
root.UnitPreferences.SetCurrentUnit("Angle", "deg")
root.UnitPreferences.SetCurrentUnit("Latitude", "deg")
root.UnitPreferences.SetCurrentUnit("Longitude", "deg")
root.UnitPreferences.SetCurrentUnit("DateFormat", "EpSec")
root.UnitPreferences.SetCurrentUnit("Duration", "sec")


print("Preparing Satellite...")
# satellite setup
refueler_sat = root.CurrentScenario.Children.New(AgESTKObjectType.eSatellite, 'Refueler')
refueler_sat.SetPropagatorType(AgEVePropagatorType.ePropagatorTwoBody)
propagator = refueler_sat.Propagator

orbitState = propagator.InitialState.Representation
orbitStateClassical = orbitState.ConvertTo(AgEOrbitStateType.eOrbitStateClassical)

orbitStateClassical.SizeShapeType = AgEClassicalSizeShape.eSizeShapeSemimajorAxis
sizeshape = orbitStateClassical.SizeShape
sizeshape.Eccentricity = 0.001
sizeshape.SemiMajorAxis = 6378 + 700

orientation = orbitStateClassical.Orientation
orientation.Inclination = 98
orientation.ArgOfPerigee = 0

orientation.AscNodeType = AgEOrientationAscNode.eAscNodeRAAN
raan = orientation.AscNode
raan.Value = 0

orbitStateClassical.LocationType = AgEClassicalLocation.eLocationTrueAnomaly
trueAnomaly = orbitStateClassical.Location
trueAnomaly.Value = 0

orbitState.Assign(orbitStateClassical)
propagator.Propagate()


sat_tx_freq = 2120
uplinkDataRate = 16 # Mbps
downlinkDataRate = 16 # Mbps

sat_tx = refueler_sat.Children.New(AgESTKObjectType.eTransmitter, 'SatTransmitter')
sat_tx.SetModel('Complex Transmitter Model')
sat_tx_model = sat_tx.Model
sat_tx_model.Frequency = sat_tx_freq/1000
sat_tx_model.DataRate = downlinkDataRate
sat_tx_model.Power = 0.1 # dBW

sat_rx = refueler_sat.Children.New(AgESTKObjectType.eReceiver, 'SatReceiver')
sat_rx.SetModel('Simple Receiver Model')
sat_rx_model = sat_rx.Model




with open(network_json) as f:
    data = json.load(f)

ground_stations = []

print("Preparing Ground Stations...")
for i, station in enumerate(data['ground-stations']['enterprise']):
    name = station['location'].replace(", ", "_").replace(" ", "_")
    lat = station['latitude']
    lon = station['longitude']

    print("\t", name, lat, lon)

    tx_frequency_range = station['frequency']['s_band']['tx_frequency_range']
    tx_frequency = (sum(tx_frequency_range) / len(tx_frequency_range))

    rx_frequency_range = station['frequency']['s_band']['rx_frequency_range']
    rx_frequency = (sum(rx_frequency_range) / len(rx_frequency_range))

    # ground station setup
    ground_stations.append(root.CurrentScenario.Children.New(AgESTKObjectType.eFacility, name))

    ground_stations[i].Position.AssignPlanetodetic(lat, lon,0)
    ground_stations[i].UseTerrain = False

    # add servo to ground station
    servo = ground_stations[i].Children.New(AgESTKObjectType.eSensor, 'Servo')
    servo.SetLocationType(AgESnLocation.eSnFixed)
    servo.LocationData.AssignCartesian(0, 0, -0.03)
    servo.CommonTasks.SetPointingTargetedTracking(AgETrackModeType.eTrackModeTranspond, AgEBoresightType.eBoresightLevel, '*/Satellite/Refueler')
    servo.AccessConstraints.RemoveConstraint(AgEAccessConstraints.eCstrLineOfSight)

    gnd_tx = servo.Children.New(AgESTKObjectType.eTransmitter, 'GndTransmitter')
    gnd_tx.SetModel('Simple Transmitter Model')
    gnd_tx_model = gnd_tx.Model
    gnd_tx_model.Frequency = tx_frequency/1000
    gnd_tx_model.Eirp = station['frequency']['s_band']['EIRP']
    gnd_tx_model.DataRate = uplinkDataRate

    gnd_rx = servo.Children.New(AgESTKObjectType.eReceiver, 'GndReceiver')
    gnd_rx.SetModel('Simple Receiver Model')
    gnd_rx_model = gnd_rx.Model
    gnd_rx_model.GOverT = station['frequency']['s_band']['G/T_db_K']



print("Saving...")
root.Save()




