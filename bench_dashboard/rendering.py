"""Low-level pygame drawing helpers shared by the dashboard panels."""

import math

import pygame

from . import config

# Distance heatmap colormap stops (mm -> RGB), near = hot, far = cold
TOF_COLOR_STOPS = [
    (0.0, (255, 64, 48)),
    (500.0, (255, 190, 40)),
    (1200.0, (96, 220, 96)),
    (2500.0, (64, 150, 255)),
    (4000.0, (150, 100, 240)),
]


def distance_color(distance_mm):
    """Map a distance in mm to a heatmap color; 0 or negative means invalid."""
    if distance_mm <= 0:
        return (44, 48, 58)
    value = min(float(distance_mm), config.TOF_MAX_MM)
    for (v0, c0), (v1, c1) in zip(TOF_COLOR_STOPS, TOF_COLOR_STOPS[1:]):
        if value <= v1:
            fraction = (value - v0) / (v1 - v0)
            return tuple(int(a + (b - a) * fraction) for a, b in zip(c0, c1))
    return TOF_COLOR_STOPS[-1][1]


def panel(surface, rect, title, fonts):
    pygame.draw.rect(surface, config.PANEL, rect, border_radius=8)
    pygame.draw.rect(surface, config.PANEL_BORDER, rect, 1, border_radius=8)
    if title:
        surface.blit(fonts["title"].render(title, True, config.TEXT_DIM), (rect.x + 12, rect.y + 7))


def fit_text(text, font, max_w):
    """Truncate text with an ellipsis so it never renders wider than max_w."""
    if font.size(text)[0] <= max_w:
        return text
    approx_char_w = max(1, font.size("n")[0])
    text = text[: max(0, int(max_w / approx_char_w))]
    while text and font.size(text + "...")[0] > max_w:
        text = text[:-1]
    return text + "..." if text else ""


def blit_text(surface, text, font, color, center=None, topleft=None, topright=None):
    surf = font.render(text, True, color)
    if center is not None:
        rect = surf.get_rect(center=center)
    elif topright is not None:
        rect = surf.get_rect(topright=topright)
    else:
        rect = surf.get_rect(topleft=topleft)
    surface.blit(surf, rect)
    return rect


def center_bar(surface, rect, value, limit, color):
    """Horizontal bar growing from the center, signed value within +/-limit."""
    pygame.draw.rect(surface, (38, 42, 52), rect, border_radius=4)
    half = rect.w / 2.0
    width = min(1.0, abs(value) / limit) * half
    x = rect.centerx if value >= 0 else rect.centerx - width
    if width >= 1:
        pygame.draw.rect(surface, color, (x, rect.y + 1, width, rect.h - 2), border_radius=3)
    pygame.draw.line(surface, config.PANEL_BORDER, (rect.centerx, rect.y), (rect.centerx, rect.bottom), 1)
    pygame.draw.rect(surface, config.PANEL_BORDER, rect, 1, border_radius=4)


def euler_to_matrix(yaw_deg, pitch_deg, roll_deg):
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll); degrees in, 3x3 row-major out."""
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rotate(matrix, vector):
    x, y, z = vector
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


CUBE_VERTS = [
    (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
    (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
]
CUBE_FACES = [(0, 1, 2, 3), (5, 4, 7, 6), (4, 0, 3, 7), (1, 5, 6, 2), (4, 5, 1, 0), (3, 2, 6, 7)]
CUBE_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
AXES = [((1, 0, 0), "X", (235, 90, 80)), ((0, 1, 0), "Y", (110, 220, 130)), ((0, 0, 1), "Z", (90, 160, 255))]


def draw_cube(surface, area, yaw, pitch, roll, fonts):
    """Shaded 3D cube rotated by yaw/pitch/roll with a body-frame axis triad."""
    cx, cy = area.centerx, area.centery
    scale = min(area.w, area.h) * 0.27
    view_dist = 4.2
    matrix = euler_to_matrix(yaw, pitch, roll)

    def project(vertex):
        x, y, z = rotate(matrix, vertex)
        factor = scale * view_dist / (z + view_dist)
        return (cx + x * factor, cy - y * factor, z)

    points = [project(vertex) for vertex in CUBE_VERTS]
    light = (-0.45, -0.6, -0.66)

    # Painter sort back-to-front, cull backfaces via the rotated outward normal.
    face_order = sorted(range(len(CUBE_FACES)), key=lambda i: sum(points[v][2] for v in CUBE_FACES[i]))
    for face_index in face_order:
        a, b, c, d = CUBE_FACES[face_index]
        corners = (a, b, c, d)
        centroid = rotate(matrix, tuple(sum(CUBE_VERTS[v][k] for v in corners) / 4.0 for k in range(3)))
        edge1 = [points[b][k] - points[a][k] for k in range(3)]
        edge2 = [points[c][k] - points[a][k] for k in range(3)]
        normal = [
            edge1[1] * edge2[2] - edge1[2] * edge2[1],
            edge1[2] * edge2[0] - edge1[0] * edge2[2],
            edge1[0] * edge2[1] - edge1[1] * edge2[0],
        ]
        length = math.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2) or 1.0
        normal = [n / length for n in normal]
        if sum(n * c for n, c in zip(normal, centroid)) < 0:
            normal = [-n for n in normal]
        if normal[2] >= 0.0:  # view looks along +z; backface
            continue
        lambert = abs(sum(n * lum for n, lum in zip(normal, light)))
        shade = int(60 + 140 * lambert)
        color = (int(shade * 0.55), int(shade * 0.75), shade)
        pygame.draw.polygon(surface, color, [points[v][:2] for v in corners])

    for a, b in CUBE_EDGES:  # wireframe keeps the silhouette readable
        pygame.draw.line(surface, (18, 20, 26), points[a][:2], points[b][:2], 1)

    origin = project((0, 0, 0))
    for axis_vector, label, color in AXES:
        tip = project(axis_vector)
        pygame.draw.line(surface, color, origin[:2], tip[:2], 2)
        label_pos = (origin[0] + (tip[0] - origin[0]) * 1.12, origin[1] + (tip[1] - origin[1]) * 1.12)
        blit_text(surface, label, fonts["small"], color, center=label_pos)
