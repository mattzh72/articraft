from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _fork_arm_geometry():
    """Side cheek plate with a real through-bore on the pitch axis."""
    arm = cq.Workplane("XY").box(0.050, 0.180, 0.240)
    bore = (
        cq.Workplane("YZ")
        .center(0.0, 0.050)
        .circle(0.023)
        .extrude(0.080, both=True)
    )
    return arm.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_fork_yaw_pitch_head")

    cast_black = Material("satin_black_cast_metal", rgba=(0.025, 0.028, 0.030, 1.0))
    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.08, 0.095, 0.11, 1.0))
    warm_steel = Material("brushed_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    bearing_blue = Material("blue_bearing_ring", rgba=(0.03, 0.14, 0.34, 1.0))
    rubber = Material("matte_rubber_foot", rgba=(0.008, 0.008, 0.008, 1.0))

    # Fixed, grounded lower stage: a broad plinth and stacked stator bearing.
    base = model.part("base")
    base.visual(
        Box((0.45, 0.36, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_black,
        name="ground_plinth",
    )
    base.visual(
        Cylinder(radius=0.170, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_anodized,
        name="stator_drum",
    )
    base.visual(
        Cylinder(radius=0.120, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=warm_steel,
        name="bearing_cap",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.155, tube=0.0045, radial_segments=20, tubular_segments=48),
            "yaw_index_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=bearing_blue,
        name="yaw_index_ring",
    )
    for i, (x, y) in enumerate(
        (
            (-0.185, -0.140),
            (0.185, -0.140),
            (-0.185, 0.140),
            (0.185, 0.140),
        )
    ):
        base.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"foot_{i}",
        )

    # The yawing stage carries the fork.  Its local frame is the vertical yaw axis
    # at the top of the fixed bearing cap, so all fork geometry is authored above
    # z=0 in this part frame.
    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_anodized,
        name="turntable_platter",
    )
    fork.visual(
        Cylinder(radius=0.055, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=dark_anodized,
        name="center_pedestal",
    )
    fork.visual(
        Box((0.300, 0.180, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
        material=dark_anodized,
        name="fork_saddle",
    )
    fork.visual(
        mesh_from_cadquery(_fork_arm_geometry(), "fork_arm_0"),
        origin=Origin(xyz=(-0.125, 0.0, 0.225)),
        material=dark_anodized,
        name="fork_arm_0",
    )
    fork.visual(
        mesh_from_cadquery(_fork_arm_geometry(), "fork_arm_1"),
        origin=Origin(xyz=(0.125, 0.0, 0.225)),
        material=dark_anodized,
        name="fork_arm_1",
    )
    fork.visual(
        Box((0.300, 0.020, 0.170)),
        origin=Origin(xyz=(0.0, 0.097, 0.245)),
        material=dark_anodized,
        name="rear_bridge",
    )
    fork.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.029, tube=0.007, radial_segments=18, tubular_segments=42),
            "bearing_ring_0",
        ),
        origin=Origin(xyz=(-0.154, 0.0, 0.275), rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_blue,
        name="bearing_ring_0",
    )
    fork.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.029, tube=0.007, radial_segments=18, tubular_segments=42),
            "bearing_ring_1",
        ),
        origin=Origin(xyz=(0.154, 0.0, 0.275), rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_blue,
        name="bearing_ring_1",
    )

    # Compact pitch cradle.  The child frame is the trunnion axis; the body hangs
    # slightly below that axis like a small sensor or fixture cradle.
    cradle = model.part("cradle")
    cradle_body = ExtrudeGeometry(
        rounded_rect_profile(0.145, 0.120, 0.014, corner_segments=8),
        0.085,
        center=True,
    )
    cradle.visual(
        mesh_from_geometry(cradle_body, "cradle_body"),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=cast_black,
        name="cradle_body",
    )
    cradle.visual(
        Box((0.125, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.069, -0.018)),
        material=warm_steel,
        name="front_clamp_bar",
    )
    cradle.visual(
        Box((0.125, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.069, -0.018)),
        material=warm_steel,
        name="rear_clamp_bar",
    )
    cradle.visual(
        Cylinder(radius=0.018, length=0.318),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_steel,
        name="trunnion_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(-0.088, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_steel,
        name="hub_0",
    )
    cradle.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.088, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_steel,
        name="hub_1",
    )

    model.articulation(
        "base_to_fork",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-pi, upper=pi),
    )
    model.articulation(
        "fork_to_cradle",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.80, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fork = object_model.get_part("fork")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("base_to_fork")
    pitch = object_model.get_articulation("fork_to_cradle")

    ctx.check(
        "two revolute axes",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE
        and yaw.axis == (0.0, 0.0, 1.0)
        and pitch.axis == (1.0, 0.0, 0.0),
        details=f"yaw={yaw.articulation_type} axis={yaw.axis}; pitch={pitch.articulation_type} axis={pitch.axis}",
    )
    ctx.expect_gap(
        fork,
        base,
        axis="z",
        positive_elem="turntable_platter",
        negative_elem="bearing_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="yawing turntable is seated on the fixed bearing",
    )
    ctx.expect_overlap(
        cradle,
        fork,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="fork_arm_0",
        min_overlap=0.040,
        name="trunnion shaft passes through first bored arm",
    )
    ctx.expect_overlap(
        cradle,
        fork,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="fork_arm_1",
        min_overlap=0.040,
        name="trunnion shaft passes through second bored arm",
    )
    ctx.expect_gap(
        cradle,
        fork,
        axis="x",
        positive_elem="hub_0",
        negative_elem="fork_arm_0",
        max_gap=0.001,
        max_penetration=0.001,
        name="first trunnion hub bears on fork arm",
    )
    ctx.expect_gap(
        fork,
        cradle,
        axis="x",
        positive_elem="fork_arm_1",
        negative_elem="hub_1",
        max_gap=0.001,
        max_penetration=0.001,
        name="second trunnion hub bears on fork arm",
    )
    ctx.expect_within(
        cradle,
        fork,
        axes="y",
        inner_elem="cradle_body",
        outer_elem="fork_arm_0",
        margin=0.003,
        name="cradle body fits between fork cheeks in depth",
    )

    rest_aabb = ctx.part_element_world_aabb(cradle, elem="cradle_body")
    with ctx.pose({pitch: 0.65}):
        ctx.expect_within(
            cradle,
            fork,
            axes="y",
            inner_elem="cradle_body",
            outer_elem="fork_arm_0",
            margin=0.003,
            name="pitched cradle clears the fork depth",
        )
        raised_aabb = ctx.part_element_world_aabb(cradle, elem="cradle_body")
    with ctx.pose({pitch: -0.65}):
        ctx.expect_within(
            cradle,
            fork,
            axes="y",
            inner_elem="cradle_body",
            outer_elem="fork_arm_0",
            margin=0.003,
            name="reverse-pitched cradle clears the fork depth",
        )
        lowered_aabb = ctx.part_element_world_aabb(cradle, elem="cradle_body")

    ctx.check(
        "pitch cradle rotates about fixed trunnions",
        rest_aabb is not None
        and raised_aabb is not None
        and lowered_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.015
        and lowered_aabb[1][2] > rest_aabb[1][2] + 0.015,
        details=f"rest={rest_aabb}, pitched_plus={raised_aabb}, pitched_minus={lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
