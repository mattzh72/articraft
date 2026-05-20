from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    ExtrudeGeometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pit_orchestra_stand")

    black = Material("satin_black", rgba=(0.015, 0.015, 0.014, 1.0))
    dark = Material("dark_graphite", rgba=(0.08, 0.085, 0.085, 1.0))
    metal = Material("brushed_steel", rgba=(0.55, 0.56, 0.53, 1.0))
    edge = Material("worn_black_edge", rgba=(0.025, 0.023, 0.021, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.18, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=black,
        name="weighted_puck",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark,
        name="top_weight_plate",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=black,
        name="lower_collar",
    )

    sleeve_geom = LatheGeometry.from_shell_profiles(
        [(0.024, 0.0), (0.024, 0.300)],
        [(0.017, 0.0), (0.017, 0.300)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(sleeve_geom, "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=metal,
        name="outer_sleeve",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.338)),
        material=black,
        name="upper_collar",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.012, length=0.610),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=metal,
        name="inner_tube",
    )
    mast.visual(
        Box((0.120, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, -0.0225, 0.360)),
        material=black,
        name="head_arm",
    )
    mast.visual(
        Box((0.026, 0.025, 0.020)),
        origin=Origin(xyz=(-0.047, -0.054, 0.360)),
        material=black,
        name="yoke_ear_0",
    )
    mast.visual(
        Box((0.026, 0.025, 0.020)),
        origin=Origin(xyz=(0.047, -0.054, 0.360)),
        material=black,
        name="yoke_ear_1",
    )
    mast.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(-0.047, -0.065, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hinge_knuckle_0",
    )
    mast.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.047, -0.065, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hinge_knuckle_1",
    )
    mast.visual(
        Cylinder(radius=0.004, length=0.120),
        origin=Origin(xyz=(0.0, -0.065, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.16),
    )

    desk = model.part("desk")
    desk.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="center_knuckle",
    )
    desk.visual(
        Box((0.050, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.012, 0.016)),
        material=black,
        name="hinge_leaf",
    )

    panel_geom = ExtrudeGeometry(
        rounded_rect_profile(0.580, 0.320, 0.024, corner_segments=8),
        0.010,
        cap=True,
        center=True,
    ).rotate_x(math.pi / 2.0)
    desk.visual(
        mesh_from_geometry(panel_geom, "desk_panel"),
        origin=Origin(xyz=(0.0, -0.018, 0.190)),
        material=black,
        name="panel",
    )
    desk.visual(
        Box((0.560, 0.060, 0.028)),
        origin=Origin(xyz=(0.0, -0.045, 0.027)),
        material=black,
        name="lower_tray",
    )
    desk.visual(
        Box((0.560, 0.010, 0.055)),
        origin=Origin(xyz=(0.0, -0.077, 0.045)),
        material=edge,
        name="front_lip",
    )
    desk.visual(
        Box((0.018, 0.018, 0.265)),
        origin=Origin(xyz=(-0.292, -0.021, 0.185)),
        material=edge,
        name="side_rail_0",
    )
    desk.visual(
        Box((0.018, 0.018, 0.265)),
        origin=Origin(xyz=(0.292, -0.021, 0.185)),
        material=edge,
        name="side_rail_1",
    )

    model.articulation(
        "desk_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, -0.065, 0.360)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.30, upper=0.70),
    )

    for index, x in enumerate((-0.185, 0.185)):
        retainer = model.part(f"retainer_{index}")
        retainer.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="pivot_pin",
        )
        arm_geom = tube_from_spline_points(
            [
                (0.0, 0.0, 0.006),
                (0.0, -0.004, 0.035),
                (0.0, -0.004, 0.100),
                (0.0, -0.002, 0.118),
            ],
            radius=0.0022,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        )
        retainer.visual(
            mesh_from_geometry(arm_geom, f"retainer_arm_{index}"),
            material=metal,
            name="wire_arm",
        )
        retainer.visual(
            Box((0.034, 0.005, 0.012)),
            origin=Origin(xyz=(0.0, -0.004, 0.116)),
            material=dark,
            name="page_pad",
        )
        model.articulation(
            f"retainer_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=desk,
            child=retainer,
            origin=Origin(xyz=(x, -0.079, 0.075)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-1.05, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    mast_slide = object_model.get_articulation("mast_slide")
    desk_tilt = object_model.get_articulation("desk_tilt")

    ctx.allow_overlap(
        base,
        mast,
        elem_a="upper_collar",
        elem_b="inner_tube",
        reason="A compact guide bushing is represented as a snug sleeve around the sliding mast.",
    )
    ctx.expect_overlap(
        base,
        mast,
        axes="z",
        elem_a="upper_collar",
        elem_b="inner_tube",
        min_overlap=0.020,
        name="mast passes through top guide bushing",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="inner mast is centered in outer sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.25,
        name="collapsed mast remains deeply inserted",
    )
    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.16}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.08,
            name="extended mast retains sleeve insertion",
        )
        raised_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast slide raises head",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.12,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    rest_desk_aabb = ctx.part_world_aabb(desk)
    ctx.allow_overlap(
        mast,
        desk,
        elem_a="hinge_pin",
        elem_b="center_knuckle",
        reason="The tilt hinge pin is intentionally captured inside the desk hinge barrel.",
    )
    ctx.expect_within(
        mast,
        desk,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="center_knuckle",
        margin=0.001,
        name="tilt hinge pin is concentric with desk barrel",
    )
    ctx.expect_overlap(
        mast,
        desk,
        axes="x",
        elem_a="hinge_pin",
        elem_b="center_knuckle",
        min_overlap=0.055,
        name="tilt hinge pin spans desk barrel",
    )
    with ctx.pose({desk_tilt: 0.55}):
        tilted_desk_aabb = ctx.part_world_aabb(desk)
    ctx.check(
        "desk tilt hinge changes tray pitch",
        rest_desk_aabb is not None
        and tilted_desk_aabb is not None
        and tilted_desk_aabb[0][1] < rest_desk_aabb[0][1] - 0.04,
        details=f"rest={rest_desk_aabb}, tilted={tilted_desk_aabb}",
    )

    for index in (0, 1):
        retainer = object_model.get_part(f"retainer_{index}")
        pivot = object_model.get_articulation(f"retainer_pivot_{index}")
        ctx.allow_overlap(
            desk,
            retainer,
            elem_a="front_lip",
            elem_b="pivot_pin",
            reason="Each spring page retainer has a short pin passing through the desk lip.",
        )
        ctx.expect_overlap(
            retainer,
            desk,
            axes="yz",
            elem_a="pivot_pin",
            elem_b="front_lip",
            min_overlap=0.003,
            name=f"retainer {index} pin is captured in lip",
        )
        rest_aabb = ctx.part_world_aabb(retainer)
        swing = 0.75 if index == 0 else -0.75
        with ctx.pose({pivot: swing}):
            swung_aabb = ctx.part_world_aabb(retainer)
        ctx.check(
            f"retainer {index} pivots across page",
            rest_aabb is not None
            and swung_aabb is not None
            and max(
                abs(swung_aabb[0][0] - rest_aabb[0][0]),
                abs(swung_aabb[1][0] - rest_aabb[1][0]),
            )
            > 0.04,
            details=f"rest={rest_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
