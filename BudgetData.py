from multiprocessing import process
from os import error
import time
from agi.stk12.stkengine import *
from agi.stk12.stkdesktop import STKDesktop
from agi.stk12.stkobjects import *
from agi.stk12.stkutil import *
from agi.stk12.vgt import *

import json
from colorama import init
import matplotlib.pyplot as plt
import bisect
import math
from numpy import linspace, array
from pathlib import Path
import multiprocessing
from tqdm.contrib.concurrent import process_map
from tqdm import tqdm

DOWNLINK_BER_THRESHOLD = 1e-15
# +12dB SNR for strong connection uplink (from NASA)

def chunks(l, n):
    """Yield n number of sequential chunks from l."""
    d, r = divmod(len(l), n)
    for i in range(n):
        si = (d+1)*(i if i < r else r) + d*(0 if i < r else i - r)
        yield l[si:si+(d+1 if i < r else d)]

def link_budget_threaded(network_json, uplinkDataRate, downlinkDataRate, sat_tx_freq, gain_list, power_list):
  
    stk = STKEngine.StartApplication(noGraphics=False)
    root = stk.NewObjectRoot()

    #stk = STKDesktop.StartApplication(visible=True, userControl=True)
    #root = stk.Root


    # Create a new scenario
    root.NewScenario('NGSP_Link_Budget')

    root.UnitPreferences.SetCurrentUnit("Distance", "km")
    root.UnitPreferences.SetCurrentUnit("Time", "min")
    root.UnitPreferences.SetCurrentUnit("Angle", "deg")
    root.UnitPreferences.SetCurrentUnit("Latitude", "deg")
    root.UnitPreferences.SetCurrentUnit("Longitude", "deg")
    root.UnitPreferences.SetCurrentUnit("DateFormat", "UTCG")
    root.UnitPreferences.SetCurrentUnit("Duration", "sec")

    root.CurrentScenario.SetTimePeriod("21 Feb 2025 00:00:00", "22 Feb 2025 00:00:00")
    root.UnitPreferences.SetCurrentUnit("DateFormat", "EpMin")


    # satellite setup
    refueler_sat = root.CurrentScenario.Children.New(AgESTKObjectType.eSatellite, 'Refueler')
    refueler_sat.SetPropagatorType(AgEVePropagatorType.ePropagatorTwoBody)
    propagator = refueler_sat.Propagator

    orbitState = propagator.InitialState.Representation
    orbitStateClassical = orbitState.ConvertTo(AgEOrbitStateType.eOrbitStateClassical)

    orbitStateClassical.SizeShapeType = AgEClassicalSizeShape.eSizeShapeSemimajorAxis
    sizeshape = orbitStateClassical.SizeShape

    orientation = orbitStateClassical.Orientation

    orientation.AscNodeType = AgEOrientationAscNode.eAscNodeRAAN
    raan = orientation.AscNode

    orbitStateClassical.LocationType = AgEClassicalLocation.eLocationTrueAnomaly
    trueAnomaly = orbitStateClassical.Location

    sat_tx = refueler_sat.Children.New(AgESTKObjectType.eTransmitter, 'SatTransmitter')
    sat_tx.SetModel('Medium Transmitter Model')
    sat_tx_model = sat_tx.Model

    sat_rx = refueler_sat.Children.New(AgESTKObjectType.eReceiver, 'SatReceiver')
    sat_rx.SetModel('Simple Receiver Model')
    sat_rx_model = sat_rx.Model


    with open(network_json) as f:
        data = json.load(f)

    ground_stations = []
    band = 's_band'

    for i, station in enumerate(data['ground-stations']):
        name = station['location'].replace(", ", "_").replace(" ", "_")
        lat = station['latitude']
        lon = station['longitude']

        #print("\t", name, lat, lon)

        tx_frequency_range = station['frequency']['s_band']['tx_frequency_range']
        tx_frequency = (sum(tx_frequency_range) / len(tx_frequency_range))

        try:
            rx_frequency_range = station['frequency'][band]['rx_frequency_range']
        except:
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
        try:
            gnd_rx_model.GOverT = station['frequency'][band]['G/T_db_K']
        except:
            gnd_rx_model.GOverT = station['frequency']['s_band']['G/T_db_K']


    downlink_time_connected = []
    costs = []
    for gain in tqdm(gain_list, desc="Gain"):
        downlink_time_connected_inner = []
        for power in tqdm(power_list, desc="Power", leave=False):
            Path(network_json.split("/")[0] + "/downlink/gain_" + str(gain) + "/power_" + str(power)).mkdir(parents=True, exist_ok=True)

            #print("Preparing Satellite...")
            
            sizeshape.Eccentricity = 0.001
            sizeshape.SemiMajorAxis = 6378 + 700

            orientation.Inclination = 98
            orientation.ArgOfPerigee = 0

            raan.Value = 0

            trueAnomaly.Value = 0

            orbitState.Assign(orbitStateClassical)
            propagator.Propagate()
            
            sat_tx_model.Frequency = sat_tx_freq/1000
            sat_tx_model.DataRate = downlinkDataRate
            sat_tx_model.Power = power # dBW
            sat_tx_model.AntennaGain = gain # dB

            #print("Antenna Gain: ", gain)
            #print("Power: ", power)
            
            downlink = [[],[],[]]
            for ground_station in ground_stations:

                gnd_rx = ground_station.Children.Item('Servo').Children.Item('GndReceiver')

                downlinkAccess = sat_tx.GetAccessToObject(gnd_rx)
                downlinkAccess.ComputeAccess()

                downlink_accessIntervals = downlinkAccess.ComputedAccessIntervalTimes.ToArray(0, -1)
                downlink_data_provider = downlinkAccess.DataProviders.Item("Link Information")

                link_budget_values = [["Time"], ["Eb/No"], ["BER"]]
                downlink_time_values = []
                downlink_ebno = []
                downlink_ber = []

                for interval in downlink_accessIntervals:
                    data = downlink_data_provider.ExecElements(interval[0], interval[1], 1, link_budget_values)
                    downlink_time_values.append(data.DataSets[0].GetValues())
                    downlink_ebno.append(data.DataSets[1].GetValues())
                    downlink_ber.append(data.DataSets[2].GetValues())
                
                #downlink.append([time_values, ebno, ber])

                contactsPerDay = len(downlink_time_values)
                for interval in downlink_time_values:

                    contactMin = interval[-1] - interval[0]
                    contactHours = contactMin / 60

                    apertureWeight = 1 # 1 for 34m station
                    baseRate = 1400 # dollars

                    contactCost = (contactHours + 1)*baseRate*apertureWeight*(0.9/7  + contactsPerDay)

                    costs.append(contactCost)

                    for i, time in enumerate(interval):
                        if time == interval[-1]:
                            interval[i] = math.ceil(time)
                        elif time == interval[0]:
                            interval[i] = math.floor(time)
                        else:
                            interval[i] = round(time)

                for i, value in enumerate(downlink_time_values):
                    for j, x in enumerate(value):
                        if x not in downlink[0]:
                            index = bisect.bisect_left(downlink[0], x)
                            downlink[0].insert(index, x)
                            downlink[1].insert(index, downlink_ebno[i][j])
                            downlink[2].insert(index, downlink_ber[i][j])
                        else:
                            index = downlink[0].index(x)
                            if downlink_ebno[i][j] > downlink[1][index]:
                                downlink[1][index] = downlink_ebno[i][j]
                                downlink[2][index] = downlink_ber[i][j]

            res_time, res_ebno, res_ber = [[]], [[]], [[]]
            last = None
            for i, x in enumerate(downlink[0]):
                if last is None or abs(last - x) <= 1:
                    res_time[-1].append(x)
                    res_ebno[-1].append(downlink[1][i])
                    res_ber[-1].append(downlink[2][i])
                else:
                    res_time.append([x])
                    res_ebno.append([downlink[1][i]])
                    res_ber.append([downlink[2][i]])
                last = x
            downlink[0] = res_time
            downlink[1] = res_ebno
            downlink[2] = res_ber

            plt.figure(1)
            plt.title("Downlink Eb/No")
            plt.xlabel("Time (EpSec)")
            plt.ylabel("Eb/No (dB)")

            plt.figure(2)
            plt.title("Downlink Bit Error Rate")
            plt.xlabel("Time (EpSec)")
            plt.ylabel("BER")

            totalTime = 0
            for j, interval in enumerate(downlink[0]):
                for i, time in enumerate(interval):
                    if float(downlink[2][j][i]) <= DOWNLINK_BER_THRESHOLD:
                        totalTime += 1

            percentConnected = totalTime / (24*60)
            #print("\tConnected Time:", totalTime, "seconds")
            #print("\tConnected Time:", percentConnected*100, "%")
            downlink_time_connected_inner.append(percentConnected)


            plt.figure(1)
            for i, _ in enumerate(downlink[0]):
                plt.plot(downlink[0][i], downlink[1][i])
            plt.gcf().savefig(network_json.split("/")[0] + "/downlink/gain_" + str(gain) + "/power_" + str(power) + "/" + band + "_ebno.pdf")
            plt.clf()
            
            plt.figure(2)
            for i, _ in enumerate(downlink[0]): 
                plt.semilogy(downlink[0][i], downlink[2][i])
            plt.gcf().savefig(network_json.split("/")[0] + "/downlink/gain_" + str(gain) + "/power_" + str(power) + "/" + band + "_ber.pdf")
            plt.clf()

            #plt.show()

        downlink_time_connected.append(downlink_time_connected_inner)
    return gain_list, downlink_time_connected, costs

if __name__ == '__main__':

        
    network_json = "awsNetwork/aws.json"
    network_json = "atlasNetwork/atlas.json"

    sat_tx_freq = 2120
    uplinkDataRate = 16 # Mbps
    downlinkDataRate = 16 # Mbps

    GAINS = linspace(0, 30, 31)
    POWERS = linspace(-16, 20, 37)

    p = []

    num_workers = 11

    if num_workers > len(GAINS):
        raise ValueError("Number of workers must be less than or equal to the number of gains.")

    split_gains = []
    for gains in chunks(GAINS,num_workers):
        split_gains.append(gains)

    pool = multiprocessing.Pool(processes=num_workers)

    time_connected_unsorted = pool.starmap(link_budget_threaded, [(network_json, uplinkDataRate, downlinkDataRate, sat_tx_freq, gains, POWERS) for gains in split_gains])

    downlink_reverse_sorted = [z for y in sorted(time_connected_unsorted, key=lambda x: x[0][0]) for z in y[1]]
    downlink_time_connected = array(downlink_reverse_sorted[::-1])


    plt.figure(1)
    plt.close()
    plt.figure(2)
    plt.close()

    plt.figure(3)
    plt.title("S-Band Atlas Connection Time")
    plt.ylabel("Gain (dB)")
    plt.xlabel("Power (dBW)")
    plt.ylim(GAINS[0], GAINS[-1])
    plt.xlim(POWERS[0], POWERS[-1])
    plt.yticks(GAINS[::2], fontsize = 7)
    plt.xticks(POWERS[::2], fontsize = 7)
    plt.imshow(downlink_time_connected, cmap='viridis_r', interpolation='nearest', origin='upper', vmin=0, vmax=1, extent=[POWERS[0], POWERS[-1], GAINS[0], GAINS[-1]])
    cbar = plt.colorbar()
    cbar.set_label("Downlink Time Connected (%)")
    plt.gcf().set_size_inches(9, 6)
    plt.gcf().savefig('/'.join(network_json.split("/")[:-1]) + "/s_band_downlink_power_gain.pdf")
    plt.show()






