# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc
# reload(gc)
# Import AlvinXY transformation module
import alvinxy.alvinxy as axy
# reload(axy)

class GPS_2_NED():
  def __init__(self, olat, olon):
    self.olat = olat
    self.olon = olon

  def gps_to_xy(self, lat, lon):
    xg2, yg2 = gc.ll2xy(lat, lon, self.olat, self.olon)
    return xg2, yg2

  def gps_to_utm(self, lat, lon):
    utmy, utmx, utmzone = gc.LLtoUTM(lat, lon)
    return utmy, ytmx, utmzone

  def gpt_to_axy(self, lat, lon):
    xa, ya = axy.ll2xy(lat, lon, self.olat, self.olon)
    return xa, ya

  def xy_to_gps(self, x, y):
    lat, lon = gc.xy2ll(x, y, olat, olan)
    return lat, lon
