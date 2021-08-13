
import numpy as np
import matplotlib.pyplot as plt
from .cpp_algorithms.bcd import bcd
from .cpp_algorithms.bcd import bcd
from .cpp_algorithms.wavefront import wavefront
from .cpp_algorithms.stc import stc
from .cpp_algorithms.common_helpers import get_all_area_maps, get_random_coords, get_end_coords
from .cpp_algorithms.metrics import coverage_metrics, printer
# from cpp_algorithms.common_helpers import plot, imshow, imshow_scatter
from .classes.builGrid import Grid
# from shapely.geometry import Polygon, Point
# from PIL import Image
from shapely.geometry import Polygon, LineString
from matplotlib.collections import LineCollection
from math import cos, sin, sqrt, atan, degrees, radians



do_animation = True


def visualize_path(grid_map, start, goal, path):  # pragma: no cover
    ox, oy = start
    gx, gy = goal
    cp = np.array(path)
    px, py = cp.T
   # px, py = np.transpose(np.flipud(np.fliplr(path)))
    if not do_animation:
        plt.imshow(grid_map, cmap='viridis')
        plt.plot(ox, oy, "-xy")
        plt.plot(px, py, "-r")
        plt.plot(gx, gy, "-pg")
        plt.show()
    else:
        for ipx, ipy in zip(py, px):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.imshow(grid_map, cmap='viridis')
            plt.plot(oy, ox, "-xb")
            plt.plot(py, px, "-r")
            """print(ipx, ipy)"""
            plt.plot(gy, gx, "-pg")
            plt.plot(ipx, ipy, "or")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(1e-16)


def recalculatePath(coveragePath):
    cp = np.array(coveragePath)
    px, py = cp.T
    coverage_path_wavefront = []
    for i in range(len(px)):
        coverage_path_wavefront.append((px[i], py[i]))
    return coverage_path_wavefront


##### Numpy-Shapely Functions #####
def ndarray2polygon(points):
    if type(points) != np.ndarray:
        raise TypeError("ndarray2polygon: takes an numpy.ndarray")
    new_tuples = []
    for point in points:
        new_tuples.append((float(point[0]), float(point[1])))
    return Polygon(new_tuples)


##### Rotation Transformations #####
class RotationTransform:
    """Represents a rotational transform"""

    def __init__(self, angle):
        self.angle = angle
        self.w = radians(self.angle)
        self.irm = np.mat([[cos(self.w), -sin(self.w), 0.0],
                           [sin(self.w), cos(self.w), 0.0],
                           [0.0, 0.0, 1.0]])


def rotation_tf_from_longest_edge(polygon):
    """Returns a rotation tf for the longest edge of the given polygon"""
    max_distance = None
    max_points = [(None, None), (None, None)]
    # Find the longest edge and the points that make it
    xs, ys = polygon.exterior.coords.xy
    points =[]
    for i in range(len(coverage_path_bcd)+1):
        points.append((xs[i], ys[i]))
    # points = zip(xs, ys)
    for i in range(len(points)):
        if i == len(points) - 1:
            pair = (points[i], points[0])
        else:
            pair = (points[i], points[i + 1])
        distance = sqrt((pair[1][0] - pair[0][0]) ** 2 + (pair[1][1] - pair[0][1]) ** 2)
        if max_distance == None or max_distance < distance:
            max_distance = distance
            max_points = pair
    # Calculate the angle and return the rotation tf
    dy = float(max_points[0][1] - max_points[1][1])
    dx = float(max_points[0][0] - max_points[1][0])
    return RotationTransform((degrees(atan(dy / dx))))


def rotate_polygon_to(polygon, rotation_transform):
    """Takes a polygon and a rotation, returns a rotated polygon"""
    points = np.array(polygon.exterior)
    tf_points = rotate_to(points, rotation_transform)
    return ndarray2polygon(tf_points)


def rotate_polygon_from(polygon, rotation_transform):
    """Takes a polygon and a rotation, returns an inverse rotated polygon"""
    points = np.array(polygon.exterior)
    tf_points = rotate_from(points, rotation_transform)
    return ndarray2polygon(tf_points)


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


##### Plotting Tools #####

def zoom_extents(ax, polygons, buff=1.0):
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


def make_axis():
    from pylab import plot, figure
    return figure(1, dpi=90).add_subplot(111), True


"""def plot_lines(lines, ax=None, color='#6699cc', alpha=1.0):
    import matplotlib.pyplot as pyplot
    show = False
    if ax == None:
        ax, show = make_axis()
    # x, y = line.xy
    t = np.linspace(0, 10, len(x))
    lc = LineCollection(lines, cmap=pyplot.get_cmap('hot'),
                        norm=pyplot.Normalize(0, 20))
    ax.add_collection(lc)
    lc.set_array(t)
    lc.set_linewidth(2)
    if show:
        import pylab
        pylab.show()"""


def plot_line(line, ax=None, color='#6699cc', alpha=1.0):
    import matplotlib.pyplot as pyplot
    show = False
    if ax == None:
        ax, show = make_axis()
    x, y = line.xy
    lines = []
    for p in range(0, len(x), 2):
        lines.append(LineString([(x[p], y[p]), (x[p + 1], y[p + 1])]))
    t = np.linspace(12, 20, len(lines))
    lc = LineCollection(lines, cmap=pyplot.get_cmap('jet'),
                        norm=pyplot.Normalize(0, 20))
    ax.add_collection(lc)
    lc.set_array(t)
    lc.set_linewidth(2)
    if show:
        import pylab
        pylab.show()


def plot_coords(coords, ax=None, color='#999999', alpha=1.0):
    show = False
    if ax == None:
        ax, show = make_axis()
    x, y = coords.xy
    ax.plot(x, y, 'o', color=color, alpha=alpha, zorder=1)
    if show:
        import pylab
        pylab.show()


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






# get the gps coordinates

gps0 = (36.029996029123815, -80.30474417324561)
gps1 = (36.02998673448093, -80.30319208665827) 
gps3 = (36.02879993669135, -80.30319273878997)
gps2 = (36.0286220439705, -80.30474630745738)
takeoffPoint = (36.02934201840477,-80.30398849277026)

step = 2
name = str(step)
rotate = -90

grid = Grid(gps0, gps1, gps2, gps3, takeoffPoint, step)
grid.validatePixelstoMeters()
# ---- Get The Map ----
area_maps = get_all_area_maps("./test_maps/")   # all area maps in the folder
area_map = area_maps[0]

grid.buildGrid(area_map.shape[0], area_map.shape[1])

# ---- Calculate Coverage Path ----
start_point = get_random_coords(area_map, 1)[0]  # returns a random coord not on an obstacle
end_point = get_end_coords(area_map, 1)[0]
print(start_point)
# start_point = (area_map(0),1)
coverage_path_bcd = bcd(area_map, start_point)      # calculate coverage path using bcd
# coverage_path_bcd = recalculatePath(coverage_path)
cm = coverage_metrics(area_map, coverage_path_bcd)
print('Métricas Calculadas BCD')
# printer(cm)
"""polygon = Polygon(np.array(coverage_path_bcd))
polygon_points = np.array(polygon.exterior)

rt = RotationTransform(rotate)
tf_points = rotate_to(polygon_points, rt)
print(tf_points.min())
add = abs(tf_points.min())
path = []
for i in range(len(tf_points)-1):
    x = int(tf_points[i][0])
    y = int(tf_points[i][1]+add)
    path.append((x, y))
print(tf_points[0][0], tf_points[0][1]+31)
tf_polygon = ndarray2polygon(tf_points)

ax, _ = make_axis()
plot_polygon(polygon, ax, color='blue')
plot_polygon(tf_polygon, ax, color='red')
ax.axvline(x=0, color='black')
ax.axhline(y=0, color='black')
zoom_extents(ax, [polygon, tf_polygon])
import pylab

pylab.show()"""


# grid.assignDataGrid(coverage_path_bcd, 'BCD_Route'+str(step))
grid.assignDataGrid(coverage_path_bcd, 'BCD_Route')
grid.calculateStatistics()
print('Longitud Ruta :', grid.distanceOfFly, 'mts')
print('Tiempo de Vuelo :', grid.timeOfFly, 'seconds')

# grid.generatePathGPS()
# grid.generateRoute('BCD_Route.csv')
# end_point = coverage_path_bcd[-1]
# print(end_point)
# print(coverage_path_bcd)





# ---- Display The Stuff ----

#imshow(area_map, figsize=(200, 200), cmap="Blues_r")                # shows the area_map
# plot(coverage_path_bcd, alpha=0.8, color="green")
# imshow_scatter([start_point], color="black")    # show the start_point   (green)
# imshow_scatter([end_point], color="red")        # show the end_point     (red)
# print(end_point)
# cm = coverage_metrics(area_map, coverage_path)  # calculate coverage metrics
# print(coverage_path_bcd)
# visualize_path(area_map, start_point, end_point, coverage_path_bcd)

# ---- Calculate Coverage Path ----
# start_point = get_random_coords(area_map, 1)[0]  # returns a random coord not on an obstacle
# print(start_point)
# start_point = (area_map(0),1)
coverage_path = wavefront(area_map, start_point, end_point)      # calculate coverage path using wavefront
coverage_path_wavefront = recalculatePath(coverage_path)


# cm = coverage_metrics(area_map, coverage_path)
grid.assignDataGrid(coverage_path_wavefront, 'WAVE_Route'+str(step))
grid.calculateStatistics()
print('Métricas Calculadas Wavefront')
print('Longitud Ruta :', grid.distanceOfFly, 'mts')
print('Tiempo de Vuelo :', grid.timeOfFly, 'seconds')
# grid.generatePathGPS()
# grid.generateRoute('Wavefront_Route.csv')

# print('Métricas Calculadas Wavefront')
# printer(cm)
# visualize_path(area_map, start_point, end_point, coverage_path_wavefront)
# ---- Display The Stuff ----
# imshow(area_map, figsize=(200, 200), cmap="Blues_r")                # shows the area_map
# plot(coverage_path_wavefront, alpha=1.8, color="green")
# imshow_scatter([start_point], color="black")    # show the start_point   (green)
# imshow_scatter([end_point], color="red")        # show the end_point     (red)
# print(end_point)

#    stc CPP

coverage_path_stc = stc(area_map, start_point)

# cm = coverage_metrics(area_map, coverage_path_stc)
grid.assignDataGrid(coverage_path_stc, 'STC_Route'+str(step))
grid.calculateStatistics()
print('Métricas Calculadas STC')
print('Longitud Ruta :', grid.distanceOfFly, 'mts')
print('Tiempo de Vuelo :', grid.timeOfFly, 'seconds')
# grid.generatePathGPS()
# grid.generateRoute('STC_Route.csv')
# print('Métricas Calculadas STC')
# printer(cm)
# visualize_path(area_map, start_point, end_point, coverage_path_stc)
# print(end_point)
