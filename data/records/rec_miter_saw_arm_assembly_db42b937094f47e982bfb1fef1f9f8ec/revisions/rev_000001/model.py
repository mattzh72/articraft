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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_sector_xz(
    *,
    center_x: float,
    center_z: float,
    inner_radius: float,
    outer_radius: float,
    thickness_y: float,
    start_angle: float,
    end_angle: float,
    segments: int = 48,
) -> MeshGeometry:
    """Closed annular sector extruded along local Y for the blade guard."""
    geom = MeshGeometry()
    angles = [
        start_angle + (end_angle - start_angle) * i / segments
        for i in range(segments + 1)
    ]
    y0 = -thickness_y / 2.0
    y1 = thickness_y / 2.0

    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []
    for angle in angles:
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer0.append(geom.add_vertex(center_x + outer_radius * ca, y0, center_z + outer_radius * sa))
        outer1.append(geom.add_vertex(center_x + outer_radius * ca, y1, center_z + outer_radius * sa))
        inner0.append(geom.add_vertex(center_x + inner_radius * ca, y0, center_z + inner_radius * sa))
        inner1.append(geom.add_vertex(center_x + inner_radius * ca, y1, center_z + inner_radius * sa))

    for i in range(segments):
        # Visible side faces at +/-Y.
        geom.add_face(outer1[i], outer1[i + 1], inner1[i + 1])
        geom.add_face(outer1[i], inner1[i + 1], inner1[i])
        geom.add_face(outer0[i + 1], outer0[i], inner0[i])
        geom.add_face(outer0[i + 1], inner0[i], inner0[i + 1])
        # Outer and inner curved walls.
        geom.add_face(outer0[i], outer0[i + 1], outer1[i + 1])
        geom.add_face(outer0[i], outer1[i + 1], outer1[i])
        geom.add_face(inner0[i + 1], inner0[i], inner1[i])
        geom.add_face(inner0[i + 1], inner1[i], inner1[i + 1])

    # End caps close the sector.
    for i in (0, segments):
        geom.add_face(outer0[i], outer1[i], inner1[i])
        geom.add_face(outer0[i], inner1[i], inner0[i])

    return geom


def _toothed_blade_xz(
    *,
    center_x: float,
    center_z: float,
    radius: float,
    tooth_depth: float,
    thickness_y: float,
    teeth: int = 48,
) -> MeshGeometry:
    """Thin saw blade disk with alternating radial teeth, extruded along Y."""
    geom = MeshGeometry()
    count = teeth * 2
    y0 = -thickness_y / 2.0
    y1 = thickness_y / 2.0
    center0 = geom.add_vertex(center_x, y0, center_z)
    center1 = geom.add_vertex(center_x, y1, center_z)
    ring0: list[int] = []
    ring1: list[int] = []
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        r = radius if i % 2 == 0 else radius - tooth_depth
        x = center_x + r * math.cos(angle)
        z = center_z + r * math.sin(angle)
        ring0.append(geom.add_vertex(x, y0, z))
        ring1.append(geom.add_vertex(x, y1, z))

    for i in range(count):
        j = (i + 1) % count
        geom.add_face(center1, ring1[i], ring1[j])
        geom.add_face(center0, ring0[j], ring0[i])
        geom.add_face(ring0[i], ring0[j], ring1[j])
        geom.add_face(ring0[i], ring1[j], ring1[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_compound_miter_saw")

    cast_grey = model.material("cast_grey", rgba=(0.42, 0.43, 0.43, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.12, 0.13, 0.13, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))
    rail_steel = model.material("polished_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    blade_steel = model.material("brushed_blade", rgba=(0.86, 0.84, 0.78, 1.0))
    saw_yellow = model.material("saw_yellow", rgba=(0.95, 0.68, 0.08, 1.0))
    warning_red = model.material("trigger_red", rgba=(0.82, 0.05, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.90, 0.62, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_grey,
        name="base_bed",
    )
    base.visual(
        Box((0.035, 0.66, 0.130)),
        origin=Origin(xyz=(-0.385, 0.0, 0.125)),
        material=dark_grey,
        name="fence_face",
    )
    base.visual(
        Box((0.055, 0.25, 0.030)),
        origin=Origin(xyz=(0.365, -0.225, 0.015)),
        material=rubber_black,
        name="foot_0",
    )
    base.visual(
        Box((0.055, 0.25, 0.030)),
        origin=Origin(xyz=(0.365, 0.225, 0.015)),
        material=rubber_black,
        name="foot_1",
    )
    base.visual(
        Box((0.055, 0.25, 0.030)),
        origin=Origin(xyz=(-0.365, -0.225, 0.015)),
        material=rubber_black,
        name="foot_2",
    )
    base.visual(
        Box((0.055, 0.25, 0.030)),
        origin=Origin(xyz=(-0.365, 0.225, 0.015)),
        material=rubber_black,
        name="foot_3",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.215, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_grey,
        name="round_table",
    )
    turntable.visual(
        Box((0.340, 0.022, 0.006)),
        origin=Origin(xyz=(0.030, 0.0, 0.0370)),
        material=rubber_black,
        name="blade_slot",
    )
    turntable.visual(
        Box((0.195, 0.070, 0.026)),
        origin=Origin(xyz=(0.285, 0.0, 0.019)),
        material=dark_grey,
        name="miter_handle",
    )
    turntable.visual(
        Box((0.070, 0.210, 0.310)),
        origin=Origin(xyz=(-0.205, 0.0, 0.190)),
        material=dark_grey,
        name="rear_tower",
    )
    turntable.visual(
        Box((0.095, 0.190, 0.074)),
        origin=Origin(xyz=(-0.235, 0.0, 0.372)),
        material=dark_grey,
        name="rail_clamp",
    )
    for idx, y in enumerate((-0.055, 0.055)):
        turntable.visual(
            Cylinder(radius=0.014, length=0.590),
            origin=Origin(xyz=(-0.055, y, 0.405), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rail_steel,
            name=f"rail_{idx}",
        )
    turntable.visual(
        Box((0.040, 0.160, 0.050)),
        origin=Origin(xyz=(-0.345, 0.0, 0.405)),
        material=dark_grey,
        name="rear_rail_bridge",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.110, 0.180, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=dark_grey,
        name="upper_pad",
    )
    carriage.visual(
        Box((0.110, 0.180, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=dark_grey,
        name="lower_pad",
    )
    for idx, y in enumerate((-0.095, 0.095)):
        carriage.visual(
            Box((0.125, 0.020, 0.144)),
            origin=Origin(xyz=(0.015, y, -0.040)),
            material=dark_grey,
            name=f"side_post_{idx}",
        )
        carriage.visual(
            Box((0.055, 0.030, 0.082)),
            origin=Origin(xyz=(0.060, y * 0.87, -0.080)),
            material=dark_grey,
            name=f"pivot_lug_{idx}",
        )

    head = model.part("saw_head")
    head.visual(
        Cylinder(radius=0.033, length=0.1353),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="pivot_hub",
    )
    head.visual(
        Box((0.235, 0.050, 0.044)),
        origin=Origin(xyz=(0.112, 0.0, 0.000)),
        material=saw_yellow,
        name="blade_arm",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.190),
        origin=Origin(xyz=(0.092, 0.0, 0.155), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=saw_yellow,
        name="motor_body",
    )
    head.visual(
        Box((0.090, 0.040, 0.140)),
        origin=Origin(xyz=(0.092, 0.0, 0.070)),
        material=saw_yellow,
        name="motor_neck",
    )
    blade_mesh = _toothed_blade_xz(
        center_x=0.215,
        center_z=-0.055,
        radius=0.135,
        tooth_depth=0.009,
        thickness_y=0.012,
        teeth=54,
    )
    head.visual(
        mesh_from_geometry(blade_mesh, "toothed_blade"),
        material=blade_steel,
        name="blade_disk",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(xyz=(0.215, 0.0, -0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="arbor_cap",
    )
    guard_mesh = _annular_sector_xz(
        center_x=0.215,
        center_z=-0.055,
        inner_radius=0.146,
        outer_radius=0.174,
        thickness_y=0.044,
        start_angle=math.radians(-8.0),
        end_angle=math.radians(207.0),
        segments=56,
    )
    head.visual(
        mesh_from_geometry(guard_mesh, "upper_blade_guard"),
        material=saw_yellow,
        name="upper_guard",
    )
    head.visual(
        Box((0.080, 0.058, 0.045)),
        origin=Origin(xyz=(0.150, 0.0, 0.070)),
        material=saw_yellow,
        name="guard_mount",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.175),
        origin=Origin(xyz=(0.020, 0.0, 0.178), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="handle_grip",
    )
    head.visual(
        Box((0.030, 0.034, 0.105)),
        origin=Origin(xyz=(-0.050, 0.0, 0.178)),
        material=rubber_black,
        name="handle_stem_0",
    )
    head.visual(
        Box((0.030, 0.034, 0.105)),
        origin=Origin(xyz=(0.083, 0.0, 0.178)),
        material=rubber_black,
        name="handle_stem_1",
    )
    trigger = model.part("trigger")
    trigger.visual(
        Box((0.018, 0.022, 0.030)),
        origin=Origin(xyz=(0.009, 0.0, -0.015)),
        material=warning_red,
        name="trigger_lever",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
            effort=40.0,
            velocity=1.0,
        ),
    )
    model.articulation(
        "turntable_to_carriage",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=carriage,
        origin=Origin(xyz=(-0.095, 0.0, 0.405)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.200, effort=120.0, velocity=0.45),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.060, 0.0, -0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(50.0),
            effort=55.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(-0.035, 0.0, 0.151)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.18, effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    turntable = object_model.get_part("turntable")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("saw_head")
    yaw = object_model.get_articulation("base_to_turntable")
    slide = object_model.get_articulation("turntable_to_carriage")
    pitch = object_model.get_articulation("carriage_to_head")
    trigger_pull = object_model.get_articulation("head_to_trigger")

    ctx.check(
        "turntable yaw range is about +/-45 degrees",
        yaw.motion_limits is not None
        and yaw.motion_limits.lower <= -math.radians(44.0)
        and yaw.motion_limits.upper >= math.radians(44.0),
    )
    ctx.check(
        "carriage slide travel is about 200 mm",
        slide.motion_limits is not None
        and 0.195 <= (slide.motion_limits.upper - slide.motion_limits.lower) <= 0.205,
    )
    ctx.check(
        "head pitch range is about 50 degrees",
        pitch.motion_limits is not None
        and pitch.motion_limits.upper >= math.radians(49.0)
        and pitch.motion_limits.lower == 0.0,
    )
    ctx.check(
        "trigger has a short pull arc",
        trigger_pull.motion_limits is not None
        and 0.12 <= trigger_pull.motion_limits.upper <= 0.25,
    )

    with ctx.pose({slide: 0.0, pitch: 0.0, yaw: 0.0}):
        rest_carriage = ctx.part_world_position(carriage)
        rest_blade = ctx.part_element_world_aabb(head, elem="blade_disk")
        ctx.expect_contact(
            carriage,
            turntable,
            elem_a="upper_pad",
            elem_b="rail_0",
            contact_tol=0.002,
            name="carriage upper pad rides on a rail",
        )

    with ctx.pose({slide: 0.200, pitch: 0.0, yaw: 0.0}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            turntable,
            elem_a="upper_pad",
            elem_b="rail_0",
            contact_tol=0.002,
            name="extended carriage remains on the rail",
        )

    ctx.check(
        "carriage translates fore-aft along the twin rails",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.190,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_blade_low = rest_blade[0][2] if rest_blade is not None else None
    with ctx.pose({pitch: math.radians(50.0), slide: 0.0, yaw: 0.0}):
        down_blade = ctx.part_element_world_aabb(head, elem="blade_disk")
    down_blade_low = down_blade[0][2] if down_blade is not None else None
    ctx.check(
        "pitching the head lowers the blade",
        rest_blade_low is not None
        and down_blade_low is not None
        and down_blade_low < rest_blade_low - 0.080,
        details=f"rest_zmin={rest_blade_low}, down_zmin={down_blade_low}",
    )

    with ctx.pose({yaw: math.radians(45.0), slide: 0.0, pitch: 0.0}):
        yawed_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "turntable yaw swings the rail assembly about the vertical axis",
        rest_carriage is not None
        and yawed_carriage is not None
        and abs(yawed_carriage[1] - rest_carriage[1]) > 0.060,
        details=f"rest={rest_carriage}, yawed={yawed_carriage}",
    )

    return ctx.report()


object_model = build_object_model()
