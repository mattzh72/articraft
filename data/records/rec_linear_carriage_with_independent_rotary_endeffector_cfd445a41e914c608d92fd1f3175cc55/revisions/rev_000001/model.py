from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    """A true hollow cylindrical sleeve, authored along local +Z."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
            [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_slide_rotary_module")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.22, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.08, 0.20, 0.38, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.72, 0.48, 0.24, 1.0))
    spindle_metal = model.material("spindle_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    screw_black = model.material("screw_black", rgba=(0.01, 0.012, 0.014, 1.0))

    # Two precise hollow sleeve meshes: one for the linear bearings around the
    # fixed rails, and a smaller one for the rotary spindle bearing.
    linear_bearing_mesh = _tube_mesh(
        "linear_bearing_sleeve",
        outer_radius=0.026,
        inner_radius=0.014,
        length=0.115,
    )
    spindle_bearing_mesh = _tube_mesh(
        "spindle_bearing_sleeve",
        outer_radius=0.047,
        inner_radius=0.014,
        length=0.056,
    )

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.86, 0.20, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=painted_steel,
        name="top_plate",
    )
    top_support.visual(
        Box((0.042, 0.20, 0.145)),
        origin=Origin(xyz=(-0.41, 0.0, 0.435)),
        material=painted_steel,
        name="end_plate_0",
    )
    top_support.visual(
        Box((0.042, 0.20, 0.145)),
        origin=Origin(xyz=(0.41, 0.0, 0.435)),
        material=painted_steel,
        name="end_plate_1",
    )
    top_support.visual(
        Cylinder(radius=0.014, length=0.82),
        origin=Origin(xyz=(0.0, -0.052, 0.405), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="guide_rod_front",
    )
    top_support.visual(
        Cylinder(radius=0.014, length=0.82),
        origin=Origin(xyz=(0.0, 0.052, 0.405), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="guide_rod_rear",
    )
    top_support.visual(
        Box((0.80, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.095, 0.365)),
        material=dark_steel,
        name="front_keeper_bar",
    )
    top_support.visual(
        Box((0.80, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.095, 0.365)),
        material=dark_steel,
        name="rear_keeper_bar",
    )
    top_support.visual(
        Box((0.040, 0.145, 0.028)),
        origin=Origin(xyz=(-0.41, 0.0, 0.358)),
        material=dark_steel,
        name="end_keeper_0",
    )
    top_support.visual(
        Box((0.040, 0.145, 0.028)),
        origin=Origin(xyz=(0.41, 0.0, 0.358)),
        material=dark_steel,
        name="end_keeper_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        linear_bearing_mesh,
        origin=Origin(xyz=(0.0, -0.052, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_bronze,
        name="linear_bearing_front",
    )
    carriage.visual(
        linear_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_bronze,
        name="linear_bearing_rear",
    )
    carriage.visual(
        Box((0.124, 0.150, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=carriage_blue,
        name="bearing_bridge",
    )
    carriage.visual(
        Box((0.040, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, -0.052, -0.031)),
        material=carriage_blue,
        name="front_hanger_tab",
    )
    carriage.visual(
        Box((0.040, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, 0.052, -0.031)),
        material=carriage_blue,
        name="rear_hanger_tab",
    )
    carriage.visual(
        Box((0.110, 0.062, 0.142)),
        origin=Origin(xyz=(0.0, 0.0, -0.124)),
        material=carriage_blue,
        name="drop_web",
    )
    carriage.visual(
        spindle_bearing_mesh,
        origin=Origin(xyz=(0.0, -0.078, -0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_bronze,
        name="spindle_bearing",
    )
    carriage.visual(
        Box((0.026, 0.056, 0.056)),
        origin=Origin(xyz=(-0.065, -0.078, -0.190)),
        material=carriage_blue,
        name="bearing_clamp_0",
    )
    carriage.visual(
        Box((0.026, 0.056, 0.056)),
        origin=Origin(xyz=(0.065, -0.078, -0.190)),
        material=carriage_blue,
        name="bearing_clamp_1",
    )
    carriage.visual(
        Box((0.040, 0.040, 0.080)),
        origin=Origin(xyz=(-0.040, -0.045, -0.153)),
        material=carriage_blue,
        name="left_gusset",
    )
    carriage.visual(
        Box((0.040, 0.040, 0.080)),
        origin=Origin(xyz=(0.040, -0.045, -0.153)),
        material=carriage_blue,
        name="right_gusset",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.014, length=0.160),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_metal,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.066, length=0.018),
        origin=Origin(xyz=(0.0, -0.128, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_metal,
        name="faceplate_disk",
    )
    spindle.visual(
        Cylinder(radius=0.030, length=0.025),
        origin=Origin(xyz=(0.0, -0.108, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_hub",
    )
    spindle.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, -0.139, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="center_boss",
    )
    spindle.visual(
        Box((0.082, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.139, 0.0)),
        material=screw_black,
        name="face_slot",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        spindle.visual(
            Cylinder(radius=0.0048, length=0.004),
            origin=Origin(
                xyz=(0.041 * math.cos(angle), -0.139, 0.041 * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=screw_black,
            name=f"face_bolt_{index}",
        )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=carriage,
        origin=Origin(xyz=(-0.18, 0.0, 0.405)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.45, lower=0.0, upper=0.36),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, -0.078, -0.190)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("support_to_carriage")
    rotary = object_model.get_articulation("carriage_to_spindle")

    ctx.allow_overlap(
        carriage,
        top_support,
        elem_a="linear_bearing_front",
        elem_b="guide_rod_front",
        reason=(
            "The front rail is intentionally captured through the linear bearing sleeve; "
            "the visual sleeve is treated as a solid proxy by overlap QC."
        ),
    )
    ctx.allow_overlap(
        carriage,
        top_support,
        elem_a="linear_bearing_rear",
        elem_b="guide_rod_rear",
        reason=(
            "The rear rail is intentionally captured through the linear bearing sleeve; "
            "the visual sleeve is treated as a solid proxy by overlap QC."
        ),
    )
    ctx.allow_overlap(
        carriage,
        spindle,
        elem_a="spindle_bearing",
        elem_b="spindle_shaft",
        reason=(
            "The rotating spindle shaft is intentionally retained inside the bearing sleeve; "
            "the sleeve mesh acts as a simplified bearing proxy."
        ),
    )

    ctx.expect_within(
        top_support,
        carriage,
        axes="yz",
        inner_elem="guide_rod_front",
        outer_elem="linear_bearing_front",
        margin=0.003,
        name="front guide rod is captured by front bearing sleeve",
    )
    ctx.expect_within(
        top_support,
        carriage,
        axes="yz",
        inner_elem="guide_rod_rear",
        outer_elem="linear_bearing_rear",
        margin=0.003,
        name="rear guide rod is captured by rear bearing sleeve",
    )
    ctx.expect_overlap(
        carriage,
        top_support,
        axes="x",
        elem_a="linear_bearing_front",
        elem_b="guide_rod_front",
        min_overlap=0.11,
        name="front linear bearing remains engaged at rest",
    )
    ctx.expect_within(
        spindle,
        carriage,
        axes="xz",
        inner_elem="spindle_shaft",
        outer_elem="spindle_bearing",
        margin=0.004,
        name="spindle shaft is centered in bearing bore",
    )
    ctx.expect_overlap(
        spindle,
        carriage,
        axes="y",
        elem_a="spindle_shaft",
        elem_b="spindle_bearing",
        min_overlap=0.050,
        name="spindle shaft passes through bearing support",
    )
    ctx.expect_gap(
        carriage,
        spindle,
        axis="y",
        positive_elem="spindle_bearing",
        negative_elem="faceplate_disk",
        min_gap=0.004,
        name="faceplate clears bearing housing",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.36}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            top_support,
            axes="x",
            elem_a="linear_bearing_front",
            elem_b="guide_rod_front",
            min_overlap=0.11,
            name="front linear bearing remains engaged at full travel",
        )
    ctx.check(
        "prismatic carriage translates along rail",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    spindle_rest = ctx.part_world_position(spindle)
    with ctx.pose({rotary: math.pi / 2.0}):
        spindle_rotated = ctx.part_world_position(spindle)
    ctx.check(
        "spindle rotation is independent of slide position",
        spindle_rest is not None
        and spindle_rotated is not None
        and abs(spindle_rest[0] - spindle_rotated[0]) < 1e-6
        and abs(spindle_rest[1] - spindle_rotated[1]) < 1e-6
        and abs(spindle_rest[2] - spindle_rotated[2]) < 1e-6,
        details=f"rest={spindle_rest}, rotated={spindle_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
