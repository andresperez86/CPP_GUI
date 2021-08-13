#!/usr/bin python3
from math import cos, radians, sin
import numpy as np
from shapely.geometry import polygon
from shapely.geometry.polygon import Polygon
from CoveragePathPlanning.cpp_algorithms.common_helpers import get_all_area_maps, get_end_coords, get_random_coords
import csv
from CoveragePathPlanning.classes.builGrid import Grid
from CoveragePathPlanning.cpp_algorithms.bcd import bcd
from CoveragePathPlanning.cpp_algorithms.wavefront import wavefront
from CoveragePathPlanning.cpp_algorithms.stc import stc
from CoveragePathPlanning.cpp_algorithms.metrics import coverage_metrics, printer

class calculateFromCoords():
    def __init__(self, route = 'Boustrophedon_CPP', velocidad = 5, altura = 5, anchoLineas = 5):
        self.gpsCoordinates = []
        self.file ='coords.csv' 
        self.typeOfRoute = route
        self.coveragePathRedundancy = 0.0
        self.coveragePathPercentage = 0.0
        self.distanceOfFly = 0.0
        self.timeOfFly = 0.0
        self.anchoLineas = anchoLineas
        self.gps0 = (10.444601, -75.391864)
        self.gps1 = (10.444648, -75.390153)
        self.gps2 = (10.443347, -75.391815)
        self.gps3 = (10.443399, -75.390110)
        self.takeoffPoint = (10.445997, -75.391791)
        self.coverage_path = []
        self.original_path = []
        self.pathRotated = []
        self.velocidad = velocidad
        self.altura = altura
        self.cantidadCurvas = 0
        
        

    def readArchiveAndCalculateRoute(self, file):
        with open(file, 'r') as csvfile:
            for data in csv.reader(csvfile, delimiter = ','):
                # coords = (0, 0)
                coords = data
                self.gpsCoordinates.append(coords)
                #self.gpsCoordinates.append(data[0], data[1])
        del data
        
        self.assignCoords()
        self.calculateGrid()



    def assignCoords(self):
        self.gps0 = (float(self.gpsCoordinates[0][0]), float(self.gpsCoordinates[0][1]))
        self.gps1 = (float(self.gpsCoordinates[1][0]), float(self.gpsCoordinates[1][1]))
        self.gps2 = (float(self.gpsCoordinates[2][0]), float(self.gpsCoordinates[2][1]))
        self.gps3 = (float(self.gpsCoordinates[3][0]), float(self.gpsCoordinates[3][1]))
        self.takeoffPoint = (float(self.gpsCoordinates[4][0]), float(self.gpsCoordinates[4][1]))
    

    def calculateGrid(self):
        self.grid = Grid(self.velocidad, 
                    self.altura,
                    self.gps0, 
                    self.gps1, 
                    self.gps2, 
                    self.gps3, 
                    self.takeoffPoint, 
                    self.anchoLineas)
        
        self.grid.validatePixelstoMeters()
        
        # ---- Get The Map ----
        area_maps = get_all_area_maps('./CoveragePathPlanning/test_maps/')
        #area_maps = get_all_area_maps("./Cove/test_maps/")   # all area maps in the folder
        area_map = area_maps[0]
        # self.deleteIsolated(area_map)
        print(area_map.shape)
        print(area_map[0][0])

        self.grid.assignCoords(area_map)
        

        # ---- Calculate Coverage Path ----
        start_point = get_random_coords(area_map, 1)[0]  # returns a random coord not on an obstacle
        print(start_point)
        end_point = get_end_coords(area_map, 1)[0]
        if self.typeOfRoute == 'Boustrophedon_ECD':
            self.coverage_path = bcd(area_map, start_point)
        elif self.typeOfRoute == 'Wavefront_Grid':
            self.coverage_path = wavefront(area_map, start_point)
            self.recalculatePath()
        else:
            self.coverage_path = stc(area_map, start_point)

        self.coveragePathPercentage, self.coveragePathRedundancy = coverage_metrics(area_map, self.coverage_path)


        
        # print((f"La ruta tiene {len(self.coverage_path)} puntos"))
        self.rotatePolygonTo(-90, abs(area_map.shape[0] - area_map.shape[1]))
        #self.pathRotated,self.original_path = grid.buildGrid(self.pathRotated, self.coverage_path)

        self.grid.assignDataGrid(self.coverage_path, 'Route')
        self.grid.calculateStatistics()
        self.distanceOfFly = self.grid.distanceOfFly
        self.timeOfFly = self.grid.timeOfFly
        self.cantidadCurvas = len(self.grid.curvePoints)


    def recalculatePath(self):
        cp = np.array(self.coverage_path)
        px, py = cp.T
        coverage_path_wavefront = []
        for i in range(len(px)):
            coverage_path_wavefront.append((px[i], py[i]))
        self.coverage_path = coverage_path_wavefront
    
    def rotatePolygonFrom(self):
        polygon = Polygon(np.array(self.original_path))
        polygon_points = np.array(polygon.exterior)
        rt = RotationTransform(90)
        tf_points = rotate_from(polygon_points, rt)

    def rotatePolygonTo(self, angle, yCoord):
        polygon = Polygon(np.array(self.coverage_path))
        polygon_points = np.array(polygon.exterior)

        rt = RotationTransform(angle)
        tf_points = rotate_to(polygon_points, rt)
        print(tf_points.min())
        # add = abs(tf_points.min())
        # path = []
        for i in range(len(tf_points)):
            x = int(tf_points[i][0])
            y = int(tf_points[i][1])
            self.pathRotated.append((x, y))
        print(tf_points[0][0], tf_points[0][1])
        tf_polygon = ndarray2polygon(tf_points)

        #ax, _ = make_axis()
        #plot_polygon(polygon, ax, color='blue')
        plot_polygon(tf_polygon, color='red')
        #ax.axvline(x=0, color='black')
        #ax.axhline(y=0, color='black')
        #zoom_extents(ax, [polygon, tf_polygon])
        #import pylab

        #pylab.show()

    ##### Rotation Transformations #####
class RotationTransform:
    """Represents a rotational transform"""

    def __init__(self, angle):
        self.angle = angle
        self.w = radians(self.angle)
        self.irm = np.mat([[cos(self.w), -sin(self.w), 0.0],
                           [sin(self.w), cos(self.w), 0.0],
                           [0.0, 0.0, 1.0]])


def rotate_to(points, rotation_transform):
    """Rotates an ndarray of given points(x,y) to a given rotation"""
    if type(points) != np.ndarray:
        raise TypeError("rotate_to: takes an numpy.ndarray")
    new_points = []
    for point in points:
        point_mat = np.mat([[point[0]], [point[1]], [0]], dtype='float64')
        new_point = rotation_transform.irm * point_mat
        new_points.append(np.array(new_point[:-1].T, dtype='float64'))
    return np.squeeze(np.array(new_points, dtype='float64'))

def rotate_from(points, rotation_transform):
    """Rotate an ndarray of given points(x,y) from a given rotation"""
    if type(points) != np.ndarray:
        raise TypeError("rotate_from: takes an numpy.ndarray")
    new_points = []
    for point in points:
        point_mat = np.mat([[point[0]], [point[1]], [0]], dtype='float64')
        new_point = rotation_transform.irm.I * point_mat
        new_points.append(np.array(new_point[:-1].T, dtype='float64'))
    return np.squeeze(np.array(new_points, dtype='float64'))


def ndarray2polygon(points):
    if type(points) != np.ndarray:
        raise TypeError("ndarray2polygon: takes an numpy.ndarray")
    new_tuples = []
    for point in points:
        new_tuples.append((float(point[0]), float(point[1])))
    return Polygon(new_tuples)

def make_axis():
    from pylab import plot, figure
    return figure(1, dpi=90).add_subplot(111), True



def plot_polygon(polygon, ax=None, color='#999999', alpha=1.0):
    show = False
    if ax == None:
        ax, show = make_axis()
    x, y = polygon.exterior.xy
    ax.plot(x, y, color=color, alpha=alpha)
    ax.set_xlim(min(x) - 1, max(x) + 1)
    ax.set_ylim(min(y) - 1, max(y) + 1)
    ax.set_aspect(1)
    if show:
        import pylab
        pylab.show()

def zoom_extents(ax, polygons, buff=10.0):
    """Sets the axis to view all polygons given"""
    min_x = None
    max_x = None
    min_y = None
    max_y = None
    for polygon in polygons:
        bounds = polygon.bounds
        if min_x == None or bounds[0] < min_x:
            min_x = bounds[0]
        if min_y == None or bounds[1] < min_y:
            min_y = bounds[1]
        if max_x == None or bounds[2] > max_x:
            max_x = bounds[2]
        if max_y == None or bounds[3] > max_y:
            max_y = bounds[3]
    ax.set_xlim(min_x - buff, max_x + buff)
    ax.set_ylim(min_y - buff, max_y + buff)
    ax.set_aspect(1)



def main():
    coords = calculateFromCoords()
    coords.readArchiveAndCalculateRoute('coords.csv')
    # coords.assignCoords()
    print('Métricas Calculadas Ruta')
    print('Longitud Ruta :', coords.distanceOfFly, 'mts')
    print('Tiempo de Vuelo :', coords.timeOfFly, 'seconds')
    print('Porcentaje Cobertura:', coords.coveragePathPercentage*100, '%')
    print('Redundancia :', coords.coveragePathRedundancy*100, '%')



if __name__ == "__main__":
    main()

