import math

# Brandon did this, I am just copying this to a new file
# centerX--- X origin of the spiral.
# centerY--- Y origin of the spiral.
# radius---- Distance from origin to outer arm.
# coils----- Number of coils or full rotations.
# rotation-- Overall rotation of the spiral. ('0'=no rotation, '1'=360 degrees, '180/360'=180 degrees)
# step------ Distance between points in spiral.
# direction- Direction of the spiral. ('1'=clockwise, '-1'=counter-clockwise)
# ref: https://stackoverflow.com/questions/13894715/draw-equidistant-points-on-a-spiral
def get_spiral(centerX, centerY, radius, coils, step, rotation=1, direction=1):
    pts = []
    # value of theta corresponding to end of last coil
    thetaMax = coils * 2 * math.pi

    rotation *= 2 * math.pi

    # How far to step away from center for each side.
    outStep = radius / thetaMax

    # TODO: Go to the origin
    # goto(centerX, centerY)
    pts.append((centerX, centerY))

    # For every side, step around and away from center.
    # start at the angle corresponding to a distance of chord
    # away from centre.
    theta = step / outStep
    while theta <= thetaMax:
        # How far away from center
        out = outStep * theta
        # How far around the center.
        around = theta + rotation

        # Convert 'around' and 'away' to X and Y.
        x = centerX + (math.cos(around) * out) * direction
        y = centerY + math.sin(around) * out

        # TODO: Go to the next point
        pts.append((x, y))

        # to a first approximation, the points are on a circle
        # so the angle between them is chord/radius

        # More precise version:
        # delta = ( -2 * away + math.sqrt ( 4 * away * away + 8 * awayStep * chord ) ) / ( 2 * awayStep )

        # General version:
        delta = step / out
        theta += delta
    return pts