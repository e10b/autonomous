from geographiclib.geodesic import Geodesic
from geographiclib.localcoordinates import LocalCartesian
from geographiclib import utm


class GPS2NED:
    def __init__(self, olat, olon, oalt=0.0):
        self.olat = olat
        self.olon = olon
        self.oalt = oalt

        # Local ENU frame centered at origin
        self.local_cart = LocalCartesian(olat, olon, oalt)

    def gps_to_xy(self, lat, lon, alt=0.0):
        x, y, z = self.local_cart.Forward(lat, lon, alt)
        return x, y

    def xy_to_gps(self, x, y, z=0.0):
        lat, lon, alt = self.local_cart.Reverse(x, y, z)
        return lat, lon

    def gps_to_utm(self, lat, lon):
        utm_x, utm_y, zone_number, zone_letter = utm.from_latlon(lat, lon)
        return utm_y, utm_x, f"{zone_number}{zone_letter}"