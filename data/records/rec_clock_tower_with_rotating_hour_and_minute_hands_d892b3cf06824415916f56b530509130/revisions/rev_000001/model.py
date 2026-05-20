from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _pyramid_mesh(base: float, height: float) -> MeshGeometry:
    """Square pyramid centered on XY with its base on z=0."""
    half = base / 2.0
    geom = MeshGeometry()
    v0 = geom.add_vertex(-half, -half, 0.0)
    v1 = geom.add_vertex(half, -half, 0.0)
    v2 = geom.add_vertex(half, half, 0.0)
    v3 = geom.add_vertex(-half, half, 0.0)
    apex = geom.add_vertex(0.0, 0.0, height)

    # Four slate roof planes.
    geom.add_face(v0, v1, apex)
    geom.add_face(v1, v2, apex)
    geom.add_face(v2, v3, apex)
    geom.add_face(v3, v0, apex)
    # Closed underside sitting on the cornice cap.
    geom.add_face(v0, v2, v1)
    geom.add_face(v0, v3, v2)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="victorian_station_clock_tower")

    brick = model.material("warm_red_brick", rgba=(0.55, 0.18, 0.10, 1.0))
    brick_dark = model.material("dark_brick_shadow", rgba=(0.30, 0.10, 0.07, 1.0))
    mortar = model.material("pale_lime_mortar", rgba=(0.72, 0.66, 0.56, 1.0))
    stone = model.material("weathered_stone", rgba=(0.60, 0.56, 0.48, 1.0))
    slate = model.material("blue_grey_slate", rgba=(0.12, 0.16, 0.19, 1.0))
    clock_white = model.material("enameled_clock_face", rgba=(0.92, 0.89, 0.80, 1.0))
    clock_black = model.material("blackened_iron", rgba=(0.01, 0.01, 0.01, 1.0))
    brass = model.material("aged_brass", rgba=(0.70, 0.50, 0.19, 1.0))

    tower = model.part("tower")

    # A broad square Victorian brick shaft on a stone plinth.
    tower.visual(
        Box((2.25, 2.25, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=stone,
        name="stone_plinth",
    )
    tower.visual(
        Box((2.00, 2.00, 3.62)),
        origin=Origin(xyz=(0.0, 0.0, 2.07)),
        material=brick,
        name="brick_shaft",
    )
    # Slightly darker corner quoins give the square shaft Victorian massing.
    for ix, x in enumerate((-1.025, 1.025)):
        for iy, y in enumerate((-1.025, 1.025)):
            tower.visual(
                Box((0.13, 0.13, 3.62)),
                origin=Origin(xyz=(x, y, 2.07)),
                material=brick_dark,
                name=f"corner_quoin_{ix}_{iy}",
            )

    # Regular horizontal mortar courses on the visible front and side faces.
    row_z = 0.36
    row_index = 0
    while row_z < 3.82:
        tower.visual(
            Box((1.90, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -1.000, row_z)),
            material=mortar,
            name=f"front_mortar_{row_index}",
        )
        tower.visual(
            Box((0.018, 1.90, 0.018)),
            origin=Origin(xyz=(-1.000, 0.0, row_z)),
            material=mortar,
            name=f"side_mortar_{row_index}",
        )
        tower.visual(
            Box((0.018, 1.90, 0.018)),
            origin=Origin(xyz=(1.000, 0.0, row_z)),
            material=mortar,
            name=f"far_side_mortar_{row_index}",
        )
        row_z += 0.20
        row_index += 1

    # A few staggered vertical brick joints around the clock face region.
    for row in range(7):
        z_mid = 2.23 + row * 0.22
        offset = 0.14 if row % 2 else 0.0
        for col, x in enumerate((-0.72 + offset, -0.36 + offset, 0.0 + offset, 0.36 + offset, 0.72 + offset)):
            if abs(x) < 0.52 and 2.52 < z_mid < 3.43:
                # Leave the clock circle clean.
                continue
            tower.visual(
                Box((0.018, 0.020, 0.105)),
                origin=Origin(xyz=(x, -1.000, z_mid)),
                material=mortar,
                name=f"front_vertical_joint_{row}_{col}",
            )

    # Corbelled cornice tier: stacked projecting courses plus individual dentils.
    cornice_courses = [
        (2.08, 2.08, 0.12, 3.94),
        (2.22, 2.22, 0.12, 4.055),
        (2.36, 2.36, 0.14, 4.180),
        (2.28, 2.28, 0.16, 4.330),
    ]
    for idx, (sx, sy, sz, cz) in enumerate(cornice_courses):
        tower.visual(
            Box((sx, sy, sz)),
            origin=Origin(xyz=(0.0, 0.0, cz)),
            material=brick if idx < 2 else stone,
            name=f"cornice_course_{idx}",
        )
    for i, x in enumerate((-0.84, -0.56, -0.28, 0.0, 0.28, 0.56, 0.84)):
        tower.visual(
            Box((0.16, 0.20, 0.20)),
            origin=Origin(xyz=(x, -1.11, 4.03)),
            material=brick_dark,
            name=f"front_corbel_{i}",
        )
        tower.visual(
            Box((0.16, 0.20, 0.20)),
            origin=Origin(xyz=(x, 1.11, 4.03)),
            material=brick_dark,
            name=f"rear_corbel_{i}",
        )
        tower.visual(
            Box((0.20, 0.16, 0.20)),
            origin=Origin(xyz=(-1.11, x, 4.03)),
            material=brick_dark,
            name=f"side_corbel_{i}",
        )
        tower.visual(
            Box((0.20, 0.16, 0.20)),
            origin=Origin(xyz=(1.11, x, 4.03)),
            material=brick_dark,
            name=f"far_side_corbel_{i}",
        )

    # Pitched slate pyramid roof with finial.
    tower.visual(
        mesh_from_geometry(_pyramid_mesh(2.42, 1.02), "pyramid_slate_roof"),
        origin=Origin(xyz=(0.0, 0.0, 4.410)),
        material=slate,
        name="pyramid_roof",
    )
    # Subtle slate course lines on the front roof plane.
    for i, z in enumerate((4.66, 4.87, 5.08)):
        scale = 1.0 - (z - 4.410) / 1.02
        tower.visual(
            Box((2.02 * scale, 0.050, 0.018)),
            origin=Origin(xyz=(0.0, -1.21 * scale, z)),
            material=clock_black,
            name=f"front_slate_course_{i}",
        )
    tower.visual(
        Cylinder(radius=0.025, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 5.60)),
        material=brass,
        name="roof_finial_spike",
    )
    tower.visual(
        Sphere(radius=0.065),
        origin=Origin(xyz=(0.0, 0.0, 5.43)),
        material=brass,
        name="roof_finial_ball",
    )

    # Round clock face mounted proud on the front shaft face.
    face_z = 3.12
    tower.visual(
        Cylinder(radius=0.48, length=0.035),
        origin=Origin(xyz=(0.0, -1.015, face_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_white,
        name="clock_face",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=0.49, tube=0.035), "clock_rim"),
        origin=Origin(xyz=(0.0, -1.030, face_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="clock_rim",
    )
    tower.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, -1.035, face_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_bushing",
    )
    # Hour ticks, radial with heavier marks at the quarters.
    for hour in range(12):
        theta = hour * math.tau / 12.0
        r = 0.38
        long_mark = hour % 3 == 0
        mark_w = 0.040 if long_mark else 0.026
        mark_l = 0.120 if long_mark else 0.070
        tower.visual(
            Box((mark_w, 0.012, mark_l)),
            origin=Origin(
                xyz=(r * math.sin(theta), -1.035, face_z + r * math.cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material=clock_black,
            name=f"hour_tick_{hour}",
        )

    # Small Roman XII on the upper part of the dial, built from raised black bars.
    for idx, x in enumerate((-0.100, -0.070, 0.000, 0.070, 0.100)):
        if idx in (1, 3):
            continue
        tower.visual(
            Box((0.018, 0.012, 0.115)),
            origin=Origin(xyz=(x, -1.036, face_z + 0.245)),
            material=clock_black,
            name=f"roman_xii_bar_{idx}",
        )
    for idx, (x, ang) in enumerate(((-0.070, 0.45), (-0.070, -0.45), (0.070, 0.45), (0.070, -0.45))):
        tower.visual(
            Box((0.018, 0.012, 0.110)),
            origin=Origin(xyz=(x, -1.037, face_z + 0.245), rpy=(0.0, ang, 0.0)),
            material=clock_black,
            name=f"roman_xii_diag_{idx}",
        )

    # Clock hands are separate articulated links, concentric at the face hub.
    hour_hand = model.part("hour_hand")
    hour_angle = math.radians(-50.0)
    hour_hand.visual(
        Box((0.075, 0.016, 0.310)),
        origin=Origin(
            xyz=(math.sin(hour_angle) * 0.155, 0.0, math.cos(hour_angle) * 0.155),
            rpy=(0.0, hour_angle, 0.0),
        ),
        material=clock_black,
        name="hour_pointer",
    )
    hour_hand.visual(
        Box((0.045, 0.016, 0.125)),
        origin=Origin(
            xyz=(-math.sin(hour_angle) * 0.0625, 0.0, -math.cos(hour_angle) * 0.0625),
            rpy=(0.0, hour_angle, 0.0),
        ),
        material=clock_black,
        name="hour_counterweight",
    )
    hour_hand.visual(
        Cylinder(radius=0.070, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hour_hub",
    )

    minute_hand = model.part("minute_hand")
    minute_angle = math.radians(58.0)
    minute_hand.visual(
        Box((0.040, 0.014, 0.410)),
        origin=Origin(
            xyz=(math.sin(minute_angle) * 0.205, -0.020, math.cos(minute_angle) * 0.205),
            rpy=(0.0, minute_angle, 0.0),
        ),
        material=clock_black,
        name="minute_pointer",
    )
    minute_hand.visual(
        Box((0.030, 0.014, 0.115)),
        origin=Origin(
            xyz=(-math.sin(minute_angle) * 0.0575, -0.020, -math.cos(minute_angle) * 0.0575),
            rpy=(0.0, minute_angle, 0.0),
        ),
        material=clock_black,
        name="minute_counterweight",
    )
    minute_hand.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="minute_hub",
    )

    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hour_hand,
        origin=Origin(xyz=(0.0, -1.060, face_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=0.20, lower=-math.tau, upper=math.tau),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=minute_hand,
        origin=Origin(xyz=(0.0, -1.060, face_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=1.20, lower=-math.tau, upper=math.tau),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    ctx.check(
        "clock hands use revolute joints",
        hour_joint.articulation_type == ArticulationType.REVOLUTE
        and minute_joint.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "hand pivots are concentric at the dial hub",
        hour_joint.origin.xyz == minute_joint.origin.xyz
        and hour_joint.axis == minute_joint.axis
        and abs(hour_joint.origin.xyz[1] + 1.060) < 1e-9,
        details=f"hour_origin={hour_joint.origin.xyz}, minute_origin={minute_joint.origin.xyz}",
    )
    ctx.expect_overlap(
        hour_hand,
        tower,
        axes="xz",
        elem_a="hour_hub",
        elem_b="clock_face",
        min_overlap=0.08,
        name="hour hub sits over clock face center",
    )
    ctx.expect_overlap(
        minute_hand,
        tower,
        axes="xz",
        elem_a="minute_hub",
        elem_b="clock_face",
        min_overlap=0.08,
        name="minute hub sits over clock face center",
    )
    ctx.expect_gap(
        tower,
        hour_hand,
        axis="y",
        positive_elem="pivot_bushing",
        negative_elem="hour_hub",
        min_gap=0.0,
        max_gap=0.012,
        name="hour hand hub is just proud of the fixed bushing",
    )

    before = ctx.part_element_world_aabb(minute_hand, elem="minute_pointer")
    with ctx.pose({minute_joint: math.pi / 2.0}):
        after = ctx.part_element_world_aabb(minute_hand, elem="minute_pointer")
    before_center = None if before is None else tuple((before[0][i] + before[1][i]) / 2.0 for i in range(3))
    after_center = None if after is None else tuple((after[0][i] + after[1][i]) / 2.0 for i in range(3))
    ctx.check(
        "minute hand visibly rotates about the face hub",
        before is not None
        and after is not None
        and before_center is not None
        and after_center is not None
        and abs(after_center[0] - before_center[0]) > 0.08
        and abs(after_center[2] - before_center[2]) > 0.05,
        details=f"before={before}, after={after}",
    )

    return ctx.report()


object_model = build_object_model()
