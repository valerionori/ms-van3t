# Valerio Nori, Università di Modena e Reggio Emilia (valerio.nori@hotmail.com)

import os
import xml.etree.ElementTree as ET
import random
from random import randrange


# This function generates a FIXED .rou.xml with a specified number of UEs
def fixedFlow(vehicleDensity, maps_path):

    numberOfUEs = int(vehicleDensity * 0.75)

    routes = ET.Element('routes')

    edges = ['ne_to_nw nw_to_sw sw_to_se se_to_ne', 'se_to_sw sw_to_nw nw_to_ne ne_to_se',
             'nw_to_sw sw_to_se se_to_ne ne_to_nw', 'sw_to_nw nw_to_ne ne_to_se se_to_sw',
             'sw_to_se se_to_ne ne_to_nw nw_to_sw', 'nw_to_ne ne_to_se se_to_sw sw_to_nw',
             'ne_to_se se_to_sw sw_to_nw nw_to_ne', 'se_to_ne ne_to_nw nw_to_sw sw_to_se']

    maxSpeed = ['30.83', '25.55', '26.94', '28.33', '29.72', '31.11', '32.5', '33.89', '35.27', '36.66']

    depart_time = 0.0
    vehicle_type = 0
    vehicle_route = 0

    for i in range(8):
        route = ET.SubElement(routes, 'route')
        route.set('id', str(i))
        route.set('edges', edges[i])

    for i in range(10):
        vType = ET.SubElement(routes, 'vType')
        vType.set('accel', '4')
        vType.set('decel', '7.5')
        vType.set('minGap', '1.0')
        vType.set('id', 'Car' + str(i))
        vType.set('maxSpeed', maxSpeed[i])

    for i in range(numberOfUEs):
        vehicle = ET.SubElement(routes, 'vehicle')
        vehicle.set('id', 'veh' + str(i + 1))

        if vehicle_type == 10:
            vehicle_type = 0
        vehicle.set('type', 'Car' + str(vehicle_type))
        vehicle_type = vehicle_type + 1

        depart_time += 0.0001
        vehicle.set('depart', str(depart_time))

        if vehicle_route == 8:
            vehicle_route = 0
        vehicle.set('route', str(vehicle_route))
        vehicle_route = vehicle_route + 1

        vehicle.set('departPos', 'random_free')
        vehicle.set('departSpeed', 'random')
        vehicle.set('departLane', 'free')

    tree = ET.ElementTree(routes)
    os.chdir(maps_path)
    ET.indent(tree, '  ')
    tree.write('cars.rou.xml')


# This function generates a RANDOM .rou.xml with a specified number of UEs (intended for final simulations)
def randomFlow(numberOfUEs, maps_path):

    routes = ET.Element('routes')

    edges = ['ne_to_nw nw_to_sw sw_to_se se_to_ne', 'se_to_sw sw_to_nw nw_to_ne ne_to_se',
             'nw_to_sw sw_to_se se_to_ne ne_to_nw', 'sw_to_nw nw_to_ne ne_to_se se_to_sw',
             'sw_to_se se_to_ne ne_to_nw nw_to_sw', 'nw_to_ne ne_to_se se_to_sw sw_to_nw',
             'ne_to_se se_to_sw sw_to_nw nw_to_ne', 'se_to_ne ne_to_nw nw_to_sw sw_to_se']

    depart_time = 0.0

    for i in range(8):
        route = ET.SubElement(routes, 'route')
        route.set('id', str(i))
        route.set('edges', edges[i])

    for i in range(10):
        vType = ET.SubElement(routes, 'vType')
        vType.set('accel', str(random.uniform(3, 6)))
        vType.set('decel', str(random.uniform(6, 9)))
        vType.set('minGap', '2.0')
        vType.set('id', 'Car' + str(i))
        vType.set('maxSpeed', str(random.uniform(20, 37)))

    for i in range(numberOfUEs):
        vehicle = ET.SubElement(routes, 'vehicle')
        vehicle.set('id', 'veh' + str(i + 1))
        vehicle.set('type', 'Car' + str(randrange(10)))
        depart_time += 0.0001
        vehicle.set('depart', str(depart_time))
        vehicle.set('route', str(randrange(8)))
        vehicle.set('departPos', 'random_free')
        vehicle.set('departSpeed', 'random')
        vehicle.set('departLane', 'free')

    tree = ET.ElementTree(routes)
    os.chdir(maps_path)
    ET.indent(tree, '  ')
    tree.write('cars.rou.xml')
