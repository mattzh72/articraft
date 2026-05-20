from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _door_outline(
    length: float,
    height: float,
    corner_radius: float,
    notch_depth: float,
    notch_height: float,
    *,
    samples: int = 8,
) -> list[tuple[float, float]]:
    """Rounded camcorder LCD door profile with a shallow concave front notch."""
    half_l = length * 0.5
    half_h = height * 0.5
    r = corner_radius
    pts: list[tuple[float, float]] = []

    def arc(cx: float, cy: float, a0: float, a1: float, count: int = samples) -> None:
        for i in range(count + 1):
            t = a0 + (a1 - a0) * i / count
            pts.append((cx + r * cos(t), cy + r * sin(t)))

    pts.append((half_l - r, half_h))
    pts.append((-half_l + r, half_h))
    arc(-half_l + r, half_h - r, pi / 2.0, pi)
    pts.append((-half_l, -half_h + r))
    arc(-half_l + r, -half_h + r, pi, 3.0 * pi / 2.0)
    pts.append((half_l - r, -half_h))
    arc(half_l - r, -half_h + r, 3.0 * pi / 2.0, 2.0 * pi)
    pts.append((half_l, -notch_height * 0.5))
    for i in range(1, samples + 1):
        t = -pi / 2.0 + pi * i / samples
        pts.append((half_l - notch_depth * cos(t), (notch_height * 0.5) * sin(t)))
    pts.append((half_l, half_h - r))
    arc(half_l - r, half_h - r, 0.0, pi / 2.0)
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_camcorder")

    warm_graphite = model.material("warm_graphite", rgba=(0.18, 0.18, 0.17, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.037, 0.040, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.016, 0.017, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.05, 0.12, 0.18, 0.62))
    screen_glass = model.material("screen_glass", rgba=(0.02, 0.025, 0.03, 1.0))
    screen_blue = model.material("screen_blue", rgba=(0.02, 0.10, 0.20, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.43, 0.44, 0.43, 1.0))
    light_mark = model.material("light_mark", rgba=(0.86, 0.86, 0.80, 1.0))

    housing = model.part("housing")

    # Consumer hand-held camcorder scale: roughly 16 cm long, 8.5 cm tall.
    body_shell = superellipse_side_loft(
        [
            (-0.039, 0.006, 0.079, 0.132),
            (-0.034, 0.001, 0.084, 0.154),
            (-0.018, 0.000, 0.086, 0.162),
            (0.018, 0.000, 0.086, 0.162),
            (0.034, 0.001, 0.084, 0.154),
            (0.039, 0.006, 0.079, 0.132),
        ],
        exponents=3.0,
        segments=64,
    )
    housing.visual(_mesh("soft_body_shell", body_shell), material=warm_graphite, name="soft_body_shell")

    # Objective barrel and glass, embedded slightly into the rounded front.
    housing.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.078, -0.001, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="objective_rear_ring",
    )
    housing.visual(
        Cylinder(radius=0.024, length=0.044),
        origin=Origin(xyz=(0.100, -0.001, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_rubber,
        name="objective_barrel",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.122, -0.001, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="front_bezel",
    )
    housing.visual(
        Cylinder(radius=0.019, length=0.003),
        origin=Origin(xyz=(0.1265, -0.001, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )

    # Small rear eyepiece and a top socket for the power/mode dial.
    housing.visual(
        Box((0.020, 0.030, 0.024)),
        origin=Origin(xyz=(-0.079, -0.001, 0.055)),
        material=satin_black,
        name="rear_eyepiece",
    )
    housing.visual(
        Cylinder(radius=0.016, length=0.003),
        origin=Origin(xyz=(-0.052, -0.018, 0.0865), rpy=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="dial_socket",
    )
    housing.visual(
        Box((0.026, 0.024, 0.010)),
        origin=Origin(xyz=(-0.052, -0.018, 0.0820)),
        material=satin_black,
        name="dial_pedestal",
    )

    # Left-side LCD hinge carrier fixed to the body.
    hinge_x = -0.064
    hinge_y = 0.042
    hinge_z = 0.045
    housing.visual(
        Box((0.014, 0.009, 0.070)),
        origin=Origin(xyz=(hinge_x, 0.0365, hinge_z)),
        material=satin_black,
        name="side_hinge_carrier",
    )
    housing.visual(
        Cylinder(radius=0.0036, length=0.018),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.022), rpy=(0.0, 0.0, 0.0)),
        material=hinge_metal,
        name="lower_hinge_knuckle",
    )
    housing.visual(
        Cylinder(radius=0.0036, length=0.018),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.068), rpy=(0.0, 0.0, 0.0)),
        material=hinge_metal,
        name="upper_hinge_knuckle",
    )

    # Flip-out left side screen door, its frame at the vertical hinge line.
    screen_door = model.part("screen_door")
    door_length = 0.114
    door_height = 0.066
    door_thick = 0.006
    door_mesh = ExtrudeGeometry(
        _door_outline(door_length, door_height, 0.006, 0.007, 0.023),
        door_thick,
        center=True,
    ).rotate_x(-pi / 2.0)
    screen_door.visual(
        _mesh("screen_door_panel", door_mesh),
        origin=Origin(xyz=(door_length * 0.5, door_thick * 0.5, 0.0)),
        material=satin_black,
        name="notched_door_panel",
    )
    screen_door.visual(
        Box((0.083, 0.0016, 0.049)),
        origin=Origin(xyz=(0.059, -0.0008, 0.0)),
        material=warm_graphite,
        name="inner_bezel",
    )
    screen_door.visual(
        Cylinder(radius=0.0034, length=0.018),
        origin=Origin(xyz=(0.0, door_thick * 0.5, 0.0), rpy=(0.0, 0.0, 0.0)),
        material=hinge_metal,
        name="middle_hinge_knuckle",
    )

    # Internal screen panel on a second swivel pivot mounted in the door.
    screen_panel = model.part("screen_panel")
    screen_panel.visual(
        Box((0.082, 0.0030, 0.047)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="screen_back",
    )
    screen_panel.visual(
        Box((0.070, 0.0010, 0.037)),
        origin=Origin(xyz=(0.0, -0.0020, 0.0)),
        material=screen_glass,
        name="black_display",
    )
    screen_panel.visual(
        Box((0.054, 0.0007, 0.026)),
        origin=Origin(xyz=(0.0, -0.0027, 0.0)),
        material=screen_blue,
        name="dim_lcd_area",
    )
    screen_panel.visual(
        Cylinder(radius=0.0065, length=0.0020),
        origin=Origin(xyz=(-0.034, -0.0022, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="swivel_cap",
    )

    power_dial = model.part("power_dial")
    dial_geom = KnobGeometry(
        0.030,
        0.010,
        body_style="faceted",
        grip=KnobGrip(style="ribbed", count=18, depth=0.0009, width=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        edge_radius=0.0007,
        center=False,
    )
    power_dial.visual(
        _mesh("power_mode_dial", dial_geom),
        origin=Origin(),
        material=satin_black,
        name="ridged_dial",
    )
    power_dial.visual(
        Box((0.012, 0.0020, 0.0010)),
        origin=Origin(xyz=(0.005, 0.0, 0.0106)),
        material=light_mark,
        name="indicator_line",
    )

    model.articulation(
        "screen_door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=screen_door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "screen_swivel",
        ArticulationType.REVOLUTE,
        parent=screen_door,
        child=screen_panel,
        origin=Origin(xyz=(0.061, -0.0015, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.0, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "power_dial_rotate",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=power_dial,
        origin=Origin(xyz=(-0.052, -0.018, 0.088)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    screen_door = object_model.get_part("screen_door")
    screen_panel = object_model.get_part("screen_panel")
    power_dial = object_model.get_part("power_dial")
    door_hinge = object_model.get_articulation("screen_door_hinge")
    screen_swivel = object_model.get_articulation("screen_swivel")
    power_joint = object_model.get_articulation("power_dial_rotate")

    ctx.expect_gap(
        screen_door,
        housing,
        axis="y",
        positive_elem="notched_door_panel",
        negative_elem="soft_body_shell",
        min_gap=0.0,
        max_gap=0.008,
        name="closed screen door sits just outside the left body side",
    )
    ctx.expect_contact(
        screen_panel,
        screen_door,
        elem_a="screen_back",
        elem_b="inner_bezel",
        contact_tol=0.0008,
        name="internal LCD panel seats on the inside of the door",
    )

    closed_door_aabb = ctx.part_world_aabb(screen_door)
    with ctx.pose({door_hinge: 1.45}):
        open_door_aabb = ctx.part_world_aabb(screen_door)
    ctx.check(
        "screen door swings outward on a vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.055,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({door_hinge: 1.35, screen_swivel: 0.0}):
        flat_screen_aabb = ctx.part_world_aabb(screen_panel)
    with ctx.pose({door_hinge: 1.35, screen_swivel: 1.0}):
        swivel_screen_aabb = ctx.part_world_aabb(screen_panel)
    ctx.check(
        "internal screen panel visibly swivels for viewing",
        flat_screen_aabb is not None
        and swivel_screen_aabb is not None
        and abs((swivel_screen_aabb[1][0] - swivel_screen_aabb[0][0]) - (flat_screen_aabb[1][0] - flat_screen_aabb[0][0])) > 0.015,
        details=f"flat={flat_screen_aabb}, swivel={swivel_screen_aabb}",
    )

    line_closed = ctx.part_element_world_aabb(power_dial, elem="indicator_line")
    with ctx.pose({power_joint: 0.9}):
        line_rotated = ctx.part_element_world_aabb(power_dial, elem="indicator_line")
    ctx.check(
        "top rear power mode dial rotates about its vertical axis",
        line_closed is not None
        and line_rotated is not None
        and abs(((line_rotated[0][1] + line_rotated[1][1]) * 0.5) - ((line_closed[0][1] + line_closed[1][1]) * 0.5)) > 0.002,
        details=f"closed={line_closed}, rotated={line_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
