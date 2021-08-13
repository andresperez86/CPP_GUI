import numpy as np
import math

def coverage_metrics(area_map, path):
    """
    Returns a dict of metrics pertaining to area coverage.
    PARAMETERS
    ---
    area_map : -1 (obstacle), 0 valued 2 dim ndarray
    path : [(x1,y1),..., (xm,ym)]
    """
    path = np.array(path).T
    assert path.shape[0] == 2
    x, y = path

    """
    calculates the distance to travel by the drone
    """
    traveled = 0
    angle = 0
    curves = 0

    for m in range(len(x)-1):
            traveled += math.sqrt(math.pow(x[m]-x[m+1], 2) + math.pow(y[m]-y[m+1], 2))

    """
    for n in range(len(x)-2):
            m1 = (y[n+1]-y[n])/(x[n+1]-x[n])
            m2 = (y[n+2]-y[n+1])/(x[n+2]-x[n+1])
            res = m1*m2
            a = np.array([x[n], y[n]])
            b = np.array([x[n+1], y[n+1]])
            c = np.array([x[n + 2], y[n + 2]])
            if res != 0:
                curves +=1
    """



    p_len = path.shape[1]
    obs_map = area_map == -1
    obs_points = obs_map.sum()
    total_points_nobs = (~obs_map).sum()

    vis_map = obs_map.copy()
    vis_map[x, y] = True
    vis_points = vis_map.sum() - obs_points

    coverage = vis_points / total_points_nobs
    redundancy = p_len / vis_points - 1

    """return {
        "points_to_visit": total_points_nobs,
        "obstacle_points": obs_points,
        "points_visited": vis_points,
        "coverage_path_len": p_len,
        "coverage": coverage,
        "redundancy": redundancy,
        "area_shape": area_map.shape,
        "distance to travel": traveled,
        "total curves": curves
    }"""
    return coverage, redundancy


def fuel_metrics(fuel_paths, fuel_capacity, full_path, area_map):
    """
    TODO : Need to complete this function, add more metrics.
    """
    full_path_metrics = coverage_metrics(area_map, full_path)
    fp_len = np.sum([len(fp) for fp in fuel_paths]) * 2
    return {
        "refuels": len(fuel_paths),
        "fuel_path_len": fp_len,
        "fuel_capacity": fuel_capacity,
        "total_redundancy": full_path_metrics["redundancy"],
    }

def printer(metrics):
    for m in metrics:
        print(f"{m.rjust(20)} : {metrics[m]}")