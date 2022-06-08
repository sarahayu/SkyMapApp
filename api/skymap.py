from skyfield.api import Star, load, N, W, wgs84
from skyfield.data import hipparcos, stellarium
import numpy as np
import quaternion as quat
import math
import datetime as dt
import time

# https://stackoverflow.com/a/68141297
def unstack(a, axis=0):
    return np.moveaxis(a, axis, 0)

# https://stackoverflow.com/a/21032099
def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)

# https://stackoverflow.com/a/44729925
def rotate_about_axis_onecoord(vec, axis, rad):
    vector = np.array([0.] + vec)
    rot_axis = np.array([0.] + axis)
    axis_angle = (rad*0.5) * rot_axis/np.linalg.norm(rot_axis)
    vecq = quat.quaternion(*vec)
    qlog = quat.quaternion(*axis_angle)
    q = np.exp(qlog)
    return (q * vecq * np.conjugate(q)).imag

def rotate_about_axis(x, y, z, axis, rad):
    x2 = np.zeros(np.size(x))
    y2 = np.zeros(np.size(y))
    z2 = np.zeros(np.size(z))
    vecs = np.stack((x, y, z), axis=-1)

    offset = 0
    for vec in vecs:
        v = rotate_about_axis_onecoord(vec, axis, rad)
        [x2[offset], y2[offset], z2[offset]] = v
        offset += 1
    return x2, y2, z2

def normalize_carts(x, y, z):
    vecs = np.stack((x, y, z), axis=-1)
    # vecs_return = np.zeros_like(vecs)

    vecs_return = normalized(vecs, axis=1)
    return unstack(vecs_return, axis=-1)

def rotate_cartesian(x, y, z, right, up, roll):
    # due to some trig, when right (azimuth) is 0, it defaults East when it should be North. So add 90deg
    x1, y1, z1 = rotate_about_axis(x, y, z, [0, 0, 1], right + np.pi / 2)
    x2, y2, z2 = rotate_about_axis(x1, y1, z1, [1, 0, 0], -up)
    x3, y3, z3 = rotate_about_axis(x2, y2, z2, [0, 1, 0], roll)
    return x3, y3, z3

# fov in radians
def project(x, y, z, fov):
    fraction_of_view = math.tan(fov / 2)
    return x / (1 + y) / fraction_of_view, z / (1 + y) / fraction_of_view

class Starmap:
    def __init__(self, min_mag):
        with load.open(hipparcos.URL) as f:
            self.df = hipparcos.load_dataframe(f)

        self.df = self.df[self.df['magnitude'] <= min_mag]

        # self.bright_stars = Star.from_dataframe(self.df)

        self.planets = load('de421.bsp')
        self.earth = self.planets['earth']


        url = ('https://raw.githubusercontent.com/Stellarium/stellarium/master'
            '/skycultures/western_SnT/constellationship.fab')
        with load.open(url) as f:
            constellations = stellarium.parse_constellations(f)

        edges = [edge for name, edges in constellations for edge in edges]
        self.edges_star1 = [star1 for star1, star2 in edges]
        self.edges_star2 = [star2 for star1, star2 in edges]

    def update_minmag(self, min_mag_new):
        # TODO dynamically change this?
        pass


# az alt are in DEGREES! UTC in seconds
def get_stars(map_instance, fov, mag, utc, lon, lat, az, alt, roll, compact):
    # utc = 1653532306526 // 1000
    df = map_instance.df[map_instance.df['magnitude'] <= mag]

    bright_stars = Star.from_dataframe(df)
    viewer_loc = wgs84.latlon(lat * N, lon * W)
    origin = map_instance.earth + viewer_loc
    date_now = dt.datetime.utcfromtimestamp(utc)
    ts = load.timescale()
    t = ts.utc(date_now.year, date_now.month, date_now.day, date_now.hour, date_now.minute)
    astrometric = origin.at(t).observe(bright_stars).apparent()
    
    x, y, z = astrometric.frame_xyz(viewer_loc).au
    cx, cy, cz = normalize_carts(x, y, z)

    # flip y axis for ease of use, i think apparent differs from actual?
    cy = -cy

    rx, ry, rz = rotate_cartesian(cx, cy, cz, np.radians(az), np.radians(alt), np.radians(roll))
    RX, RY = project(rx, ry, rz, fov * np.pi / 180)

    in_frame = ((RX <= 1) & (RX >= -1) & (RY <= 1) & (RY >= -1))

    start = time.time()
    star1s = []
    star2s = []
    for i in range(len(map_instance.edges_star1)):
        if map_instance.edges_star1[i] in df.index \
            and map_instance.edges_star2[i] in df.index:
            star1s.append(df.index.get_loc(map_instance.edges_star1[i]))
            star2s.append(df.index.get_loc(map_instance.edges_star2[i]))

    star1xs = RX[star1s]
    star1ys = RY[star1s]
    star2xs = RX[star2s]
    star2ys = RY[star2s]

    in_frame2 = (((star1xs <= 1) & (star1xs >= -1) & (star1ys <= 1) & (star1ys >= -1))
        | ((star2xs <= 1) & (star2xs >= -1) & (star2ys <= 1) & (star2ys >= -1)))

    star1xs = np.round((star1xs[in_frame2] + 1) / 2, 3)
    star1ys = np.round((star1ys[in_frame2] + 1) / 2, 3)
    star2xs = np.round((star2xs[in_frame2] + 1) / 2, 3)
    star2ys = np.round((star2ys[in_frame2] + 1) / 2, 3)
        
    # xy1 = np.stack([star1xs, star1ys], axis=1)
    # xy2 = np.stack([star2xs, star2ys], axis=1)

    # lines_xy = np.rollaxis(np.array([xy1, xy2]), 1)

    # return (df['magnitude'][in_frame]) * 100 // 8, (RX[in_frame] + 1) / 2, (RY[in_frame] + 1) / 2, lines_xy

    stars = np.stack(((RX[in_frame] + 1) / 2, (RY[in_frame] + 1) / 2, (df['magnitude'][in_frame]) * 100 // 8), axis=1)

    # if compact, return comma separated string
    if compact:
        stars_str = ""
        first = True

        for star in stars:
            if not first:
                stars_str += ","
            first = False
            stars_str += f"{ int(star[2]) },{ round(star[0], 3) },{ round(star[1], 3) }"

        stars_str += ";"
        first = True

        for i in range(len(star1xs)):

            if math.isnan(star1xs[i]) \
                or math.isnan(star1ys[i]) \
                or math.isnan(star2xs[i]) \
                or math.isnan(star2ys[i]):
                continue
            
            if not first:
                stars_str += ","
            first = False
            
            stars_str += f"{ star1xs[i] },{ star1ys[i] },{ star2xs[i] },{ star2ys[i] }"

        return stars_str
    
    # if not compact, return normal object representation
    star_objs = []
    line_objs = []

    for star in stars:
        star_objs.append({
            "x": round(star[0], 3),
            "y": round(star[1], 3),
            "mag": int(star[2])
        })

    for i in range(len(star1xs)):
        if math.isnan(star1xs[i]) \
            or math.isnan(star1ys[i]) \
            or math.isnan(star2xs[i]) \
            or math.isnan(star2ys[i]):
            continue

        line_objs.append({
            "x1": star1xs[i],
            "y1": star1ys[i],
            "x2": star2xs[i],
            "y2": star2ys[i],
        })

    return { "stars": star_objs, "lines": line_objs }

# starmap = Starmap(5)

# start = time.time()
# print(get_stars(starmap, 3, 1653532306526 // 1000, 121.7405, 38.5449, 25, 20))
# end = time.time()
# print(f"First took { end - start } seconds")
# start = time.time()
# print(get_stars(starmap, 4, 1653532306526 // 1000, 121.7405, 38.5449, 25, 20))
# end = time.time()
# print(f"Second took { end - start } seconds")
# start = time.time()
# print(get_stars(starmap, 5, 1653532306726 // 1000, 121.7405, 38.5449, 25, 20))
# end = time.time()
# print(f"Third took { end - start } seconds")
# start = time.time()
# print(get_stars(starmap, 5, 1653532306926 // 1000, 121.7405, 38.5449, 25, 20))
# end = time.time()
# print(f"Fourth took { end - start } seconds")