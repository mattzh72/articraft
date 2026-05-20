from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HANDLE_LENGTH = 0.150
HANDLE_WIDTH = 0.026
HANDLE_HEIGHT = 0.034
CARRIAGE_LOWER = -0.018
CARRIAGE_UPPER = 0.026
DOOR_UPPER = 1.35


def _xz_prism(points: list[tuple[float, float]], thickness: float) -> MeshGeometry:
    """Build a thin prism from a closed x/z polygon, extruded along y."""
    half = thickness / 2.0
    verts = [(x, -half, z) for x, z in points] + [(x, half, z) for x, z in points]
    n = len(points)
    faces: list[tuple[int, int, int]] = []

    # Back and front faces use opposite windings.
    for i in range(1, n - 1):
        faces.append((0, i + 1, i))
        faces.append((n, n + i, n + i + 1))

    # Side walls.
    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j))
        faces.append((i, n + j, n + i))

    return MeshGeometry(vertices=verts, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_retractable_box_cutter")

    safety_yellow = Material("safety_yellow", color=(1.0, 0.66, 0.08, 1.0))
    dark_plastic = Material("dark_plastic", color=(0.035, 0.035, 0.035, 1.0))
    black_rubber = Material("black_rubber", color=(0.01, 0.01, 0.012, 1.0))
    satin_steel = Material("satin_steel", color=(0.62, 0.64, 0.66, 1.0))
    blade_steel = Material("blade_steel", color=(0.86, 0.88, 0.88, 1.0))
    warning_red = Material("red_release_dot", color=(0.9, 0.05, 0.02, 1.0))

    handle = model.part("handle")

    # The handle is a short plastic sleeve made from connected wall members.
    # The centerline cavity is left open for the sliding carriage and blade.
    handle.visual(
        Box((HANDLE_LENGTH, HANDLE_WIDTH, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=safety_yellow,
        name="bottom_shell",
    )
    for suffix, y in (("0", -0.00875), ("1", 0.00875)):
        handle.visual(
            Box((HANDLE_LENGTH, 0.0085, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.014)),
            material=safety_yellow,
            name=f"top_rail_{suffix}",
        )
    for suffix, y in (("0", -0.011), ("1", 0.011)):
        handle.visual(
            Box((HANDLE_LENGTH, 0.004, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=safety_yellow,
            name=f"side_wall_{suffix}",
        )
    handle.visual(
        Box((0.010, HANDLE_WIDTH, HANDLE_HEIGHT)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0)),
        material=safety_yellow,
        name="rear_cap",
    )

    # Nose hardware: two cheek plates and small jaws around the blade slot.
    handle.visual(
        Box((0.042, 0.002, 0.026)),
        origin=Origin(xyz=(0.054, -0.014, 0.0)),
        material=satin_steel,
        name="nose_plate_0",
    )
    handle.visual(
        Box((0.016, 0.002, 0.026)),
        origin=Origin(xyz=(0.067, 0.014, 0.0)),
        material=satin_steel,
        name="nose_plate_1",
    )
    handle.visual(
        Box((0.012, 0.018, 0.006)),
        origin=Origin(xyz=(0.074, 0.0, 0.0095)),
        material=satin_steel,
        name="upper_jaw",
    )
    handle.visual(
        Box((0.012, 0.018, 0.006)),
        origin=Origin(xyz=(0.074, 0.0, -0.0095)),
        material=satin_steel,
        name="lower_jaw",
    )

    # Rubber pads give the compact body a believable handheld scale and grip.
    for suffix, y in (("0", -0.0138), ("1", 0.0138)):
        handle.visual(
            Box((0.066, 0.0016, 0.017)),
            origin=Origin(xyz=(-0.031, y, 0.0)),
            material=black_rubber,
            name=f"grip_pad_{suffix}",
        )
    for i, x in enumerate((-0.052, -0.039, -0.026, -0.013)):
        handle.visual(
            Box((0.0025, 0.0018, 0.018)),
            origin=Origin(xyz=(x, -0.0149, 0.0)),
            material=dark_plastic,
            name=f"grip_groove_{i}",
        )

    # The fixed middle knuckle of the small side hinge is carried by the handle.
    hinge_x = 0.058
    hinge_y = 0.01525
    hinge_radius = 0.0022
    handle.visual(
        Cylinder(radius=hinge_radius, length=0.006),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        material=satin_steel,
        name="body_hinge_barrel",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.078, 0.010, 0.022)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        material=dark_plastic,
        name="carriage_rail",
    )
    blade_profile = [
        (0.030, -0.0048),
        (0.070, -0.0048),
        (0.081, -0.0012),
        (0.068, 0.0048),
        (0.034, 0.0048),
    ]
    carriage.visual(
        mesh_from_geometry(_xz_prism(blade_profile, 0.0014), "trapezoid_blade"),
        material=blade_steel,
        name="blade",
    )
    carriage.visual(
        Box((0.014, 0.005, 0.015)),
        origin=Origin(xyz=(-0.013, 0.0, 0.011)),
        material=dark_plastic,
        name="slider_stem",
    )
    carriage.visual(
        Box((0.028, 0.014, 0.006)),
        origin=Origin(xyz=(-0.013, 0.0, 0.0215)),
        material=dark_plastic,
        name="thumb_slider",
    )
    for i, x in enumerate((-0.023, -0.016, -0.009, -0.002)):
        carriage.visual(
            Box((0.002, 0.012, 0.0015)),
            origin=Origin(xyz=(x, 0.0, 0.0244)),
            material=black_rubber,
            name=f"slider_rib_{i}",
        )

    door = model.part("service_door")
    door.visual(
        Box((0.036, 0.0022, 0.022)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=safety_yellow,
        name="service_panel",
    )
    door.visual(
        Box((0.032, 0.0008, 0.017)),
        origin=Origin(xyz=(-0.020, 0.0015, 0.0)),
        material=dark_plastic,
        name="panel_recess",
    )
    for suffix, z in (("0", -0.008), ("1", 0.008)):
        door.visual(
            Cylinder(radius=hinge_radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=satin_steel,
            name=f"door_hinge_barrel_{suffix}",
        )
    door.visual(
        Cylinder(radius=0.003, length=0.001),
        origin=Origin(xyz=(-0.023, 0.0021, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="door_screw",
    )
    door.visual(
        Cylinder(radius=0.0015, length=0.0012),
        origin=Origin(xyz=(-0.010, 0.0021, -0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warning_red,
        name="release_dot",
    )

    model.articulation(
        "handle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=CARRIAGE_LOWER,
            upper=CARRIAGE_UPPER,
        ),
    )
    model.articulation(
        "handle_to_service_door",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        # The closed door extends along local -X on the +Y side; -Z makes
        # positive motion swing it outward from the handle.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=DOOR_UPPER),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carriage = object_model.get_part("carriage")
    door = object_model.get_part("service_door")
    slide = object_model.get_articulation("handle_to_carriage")
    hinge = object_model.get_articulation("handle_to_service_door")

    ctx.expect_within(
        carriage,
        handle,
        axes="yz",
        inner_elem="carriage_rail",
        margin=0.002,
        name="blade carriage rides inside the handle sleeve",
    )
    ctx.expect_overlap(
        carriage,
        handle,
        axes="x",
        elem_a="carriage_rail",
        elem_b="bottom_shell",
        min_overlap=0.060,
        name="carriage is retained inside the short handle",
    )
    ctx.expect_gap(
        door,
        handle,
        axis="y",
        positive_elem="service_panel",
        negative_elem="side_wall_1",
        min_gap=0.0004,
        max_gap=0.0025,
        name="service door is seated just proud of the side wall",
    )

    with ctx.pose({slide: CARRIAGE_LOWER}):
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a="carriage_rail",
            elem_b="bottom_shell",
            min_overlap=0.060,
            name="retracted carriage remains captured",
        )
        retracted_aabb = ctx.part_element_world_aabb(carriage, elem="blade")

    with ctx.pose({slide: CARRIAGE_UPPER}):
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a="carriage_rail",
            elem_b="bottom_shell",
            min_overlap=0.045,
            name="extended carriage still has retained insertion",
        )
        extended_aabb = ctx.part_element_world_aabb(carriage, elem="blade")

    ctx.check(
        "blade tip advances forward",
        retracted_aabb is not None
        and extended_aabb is not None
        and extended_aabb[1][0] > retracted_aabb[1][0] + 0.035,
        details=f"retracted={retracted_aabb}, extended={extended_aabb}",
    )

    closed_panel = ctx.part_element_world_aabb(door, elem="service_panel")
    with ctx.pose({hinge: DOOR_UPPER}):
        opened_panel = ctx.part_element_world_aabb(door, elem="service_panel")

    closed_y = None if closed_panel is None else (closed_panel[0][1] + closed_panel[1][1]) / 2.0
    opened_y = None if opened_panel is None else (opened_panel[0][1] + opened_panel[1][1]) / 2.0
    ctx.check(
        "service door swings outward on side hinge",
        closed_y is not None and opened_y is not None and opened_y > closed_y + 0.012,
        details=f"closed_y={closed_y}, opened_y={opened_y}",
    )

    return ctx.report()


object_model = build_object_model()
