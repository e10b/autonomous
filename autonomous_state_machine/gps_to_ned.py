#!/usr/bin/env python3

# import sys
# print("Python version", sys.version)
# print("Python executable", sys.executable)

from pyproj import Transformer
import utm

class GPS_2_NED:
    def __init__(self, olat, olon, oalt=0.0):
        self.olat = olat
        self.olon = olon
        self.oalt = oalt

        # Create ENU transformer centered at origin
        self.transformer_to_local = Transformer.from_crs(
            "epsg:4979",  # WGS84 lat/lon/alt
            f"+proj=tmerc +lat_0={olat} +lon_0={olon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84",
            always_xy=True,
        )

        self.transformer_to_gps = Transformer.from_crs(
            f"+proj=tmerc +lat_0={olat} +lon_0={olon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84",
            "epsg:4979",
            always_xy=True,
        )

    def gps_to_xy(self, lat, lon, alt=0.0):
        x, y = self.transformer_to_local.transform(lon, lat)
        return x, y

    def xy_to_gps(self, x, y):
        lon, lat = self.transformer_to_gps.transform(x, y)
        return lat, lon

    def gps_to_utm(self, lat, lon):
        utm_x, utm_y, zone_number, zone_letter = utm.from_latlon(lat, lon)
        return utm_y, utm_x, f"{zone_number}{zone_letter}"