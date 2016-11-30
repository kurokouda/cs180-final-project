
def two_circles_overlapped(center_a, radius_a, center_b, radius_b):
    distance_between_centers = ((center_a.x - center_b.x)**2 +
            (center_a.y - center_b.y)**2)**0.5

    return (distance_between_centers < radius_a + radius_b or
            distance_between_centers < abs(radius_a - radius_b))
