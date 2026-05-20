from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convenience_chest_freezer")

    white = Material("slightly_glossy_white_insulation", rgba=(0.92, 0.95, 0.96, 1.0))
    inner = Material("dark_recessed_liner", rgba=(0.06, 0.075, 0.085, 1.0))
    gasket = Material("black_rubber_gasket", rgba=(0.01, 0.012, 0.012, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.65, 0.68, 0.69, 1.0))
    glass = Material("pale_blue_safety_glass", rgba=(0.45, 0.80, 0.96, 0.38))
    nylon = Material("black_nylon_runner", rgba=(0.02, 0.02, 0.018, 1.0))
    brass = Material("brushed_brass_lock", rgba=(0.86, 0.65, 0.22, 1.0))

    cabinet = model.part("cabinet")

    length = 1.60
    depth = 0.72
    height = 0.86
    wall = 0.055

    # Deep insulated chest body: separate walls and bottom leave a readable
    # top cavity instead of a solid block.
    cabinet.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, height / 2.0)),
        material=white,
        name="front_wall",
    )
    cabinet.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-length / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="end_wall_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_wall",
    )
    cabinet.visual(
        Box((length, depth, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=white,
        name="insulated_floor",
    )
    cabinet.visual(
        Box((length - 2 * wall, depth - 2 * wall, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=inner,
        name="dark_inner_floor",
    )

    # Thick rim and rubber gasket around the open top.
    cabinet.visual(
        Box((length, 0.075, 0.055)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.0375, height + 0.020)),
        material=white,
        name="front_top_rim",
    )
    cabinet.visual(
        Box((length, 0.075, 0.055)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.0375, height + 0.020)),
        material=white,
        name="rear_top_rim",
    )
    cabinet.visual(
        Box((0.090, depth, 0.055)),
        origin=Origin(xyz=(-length / 2.0 + 0.045, 0.0, height + 0.020)),
        material=white,
        name="end_top_rim_0",
    )
    cabinet.visual(
        Box((0.090, depth, 0.055)),
        origin=Origin(xyz=(length / 2.0 - 0.045, 0.0, height + 0.020)),
        material=white,
        name="end_top_rim_1",
    )
    cabinet.visual(
        Box((length - 0.16, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.285, 0.884)),
        material=gasket,
        name="front_gasket",
    )
    cabinet.visual(
        Box((length - 0.16, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.285, 0.884)),
        material=gasket,
        name="rear_gasket",
    )

    # Two-height aluminum sliding tracks support the overlapping glass lids.
    cabinet.visual(
        Box((length - 0.15, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -0.275, 0.891)),
        material=aluminum,
        name="lower_track_front",
    )
    cabinet.visual(
        Box((length - 0.15, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.275, 0.891)),
        material=aluminum,
        name="lower_track_rear",
    )
    cabinet.visual(
        Box((length - 0.15, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -0.275, 0.945)),
        material=aluminum,
        name="upper_track_front",
    )
    cabinet.visual(
        Box((length - 0.15, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.275, 0.945)),
        material=aluminum,
        name="upper_track_rear",
    )
    cabinet.visual(
        Box((0.040, 0.026, 0.030)),
        origin=Origin(xyz=(0.730, -0.275, 0.922)),
        material=aluminum,
        name="upper_track_front_post",
    )
    cabinet.visual(
        Box((0.040, 0.026, 0.030)),
        origin=Origin(xyz=(0.730, 0.275, 0.922)),
        material=aluminum,
        name="upper_track_rear_post",
    )

    # Side lock hardware mounted on the narrow end wall.
    cabinet.visual(
        Box((0.020, 0.170, 0.220)),
        origin=Origin(xyz=(0.810, -0.180, 0.555)),
        material=white,
        name="side_lock_boss",
    )
    cabinet.visual(
        Box((0.020, 0.150, 0.022)),
        origin=Origin(xyz=(0.810, -0.180, 0.660)),
        material=aluminum,
        name="hinge_leaf",
    )
    cabinet.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.824, -0.180, 0.550), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="key_cylinder",
    )
    cabinet.visual(
        Box((0.003, 0.007, 0.026)),
        origin=Origin(xyz=(0.829, -0.180, 0.550)),
        material=inner,
        name="key_slot",
    )

    # Low-level slider: its runners touch the lower track at the closed pose.
    lower_lid = model.part("lower_lid")
    lid_len = 0.84
    lid_depth = 0.58
    lower_lid.visual(
        Box((0.780, 0.500, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=glass,
        name="glass_pane",
    )
    lower_lid.visual(
        Box((lid_len, 0.035, 0.026)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0 + 0.0175, 0.0)),
        material=aluminum,
        name="front_frame",
    )
    lower_lid.visual(
        Box((lid_len, 0.035, 0.026)),
        origin=Origin(xyz=(0.0, lid_depth / 2.0 - 0.0175, 0.0)),
        material=aluminum,
        name="rear_frame",
    )
    lower_lid.visual(
        Box((0.035, lid_depth, 0.026)),
        origin=Origin(xyz=(-lid_len / 2.0 + 0.0175, 0.0, 0.0)),
        material=aluminum,
        name="end_frame_0",
    )
    lower_lid.visual(
        Box((0.035, lid_depth, 0.026)),
        origin=Origin(xyz=(lid_len / 2.0 - 0.0175, 0.0, 0.0)),
        material=aluminum,
        name="end_frame_1",
    )
    lower_lid.visual(
        Box((lid_len - 0.16, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.275, -0.019)),
        material=nylon,
        name="front_shoe",
    )
    lower_lid.visual(
        Box((lid_len - 0.16, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.275, -0.019)),
        material=nylon,
        name="rear_shoe",
    )
    lower_lid.visual(
        Box((0.090, 0.035, 0.028)),
        origin=Origin(xyz=(-0.210, -0.160, 0.020)),
        material=aluminum,
        name="pull_handle",
    )

    upper_lid = model.part("upper_lid")
    upper_lid.visual(
        Box((0.780, 0.500, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=glass,
        name="glass_pane",
    )
    upper_lid.visual(
        Box((lid_len, 0.035, 0.026)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0 + 0.0175, 0.0)),
        material=aluminum,
        name="front_frame",
    )
    upper_lid.visual(
        Box((lid_len, 0.035, 0.026)),
        origin=Origin(xyz=(0.0, lid_depth / 2.0 - 0.0175, 0.0)),
        material=aluminum,
        name="rear_frame",
    )
    upper_lid.visual(
        Box((0.035, lid_depth, 0.026)),
        origin=Origin(xyz=(-lid_len / 2.0 + 0.0175, 0.0, 0.0)),
        material=aluminum,
        name="end_frame_0",
    )
    upper_lid.visual(
        Box((0.035, lid_depth, 0.026)),
        origin=Origin(xyz=(lid_len / 2.0 - 0.0175, 0.0, 0.0)),
        material=aluminum,
        name="end_frame_1",
    )
    upper_lid.visual(
        Box((lid_len - 0.16, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.275, -0.019)),
        material=nylon,
        name="front_shoe",
    )
    upper_lid.visual(
        Box((lid_len - 0.16, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.275, -0.019)),
        material=nylon,
        name="rear_shoe",
    )
    upper_lid.visual(
        Box((0.090, 0.035, 0.028)),
        origin=Origin(xyz=(0.210, 0.160, 0.020)),
        material=aluminum,
        name="pull_handle",
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_barrel",
    )
    lock_flap.visual(
        Box((0.014, 0.130, 0.170)),
        origin=Origin(xyz=(0.014, 0.0, -0.085)),
        material=white,
        name="flap_panel",
    )
    lock_flap.visual(
        Box((0.010, 0.080, 0.018)),
        origin=Origin(xyz=(0.025, 0.0, -0.163)),
        material=aluminum,
        name="finger_lip",
    )

    model.articulation(
        "cabinet_to_lower_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_lid,
        origin=Origin(xyz=(-0.350, 0.0, 0.925)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.50),
    )
    model.articulation(
        "cabinet_to_upper_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_lid,
        origin=Origin(xyz=(0.350, 0.0, 0.979)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.50),
    )
    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_flap,
        origin=Origin(xyz=(0.832, -0.180, 0.660)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lower_lid = object_model.get_part("lower_lid")
    upper_lid = object_model.get_part("upper_lid")
    lock_flap = object_model.get_part("lock_flap")
    lower_slide = object_model.get_articulation("cabinet_to_lower_lid")
    upper_slide = object_model.get_articulation("cabinet_to_upper_lid")
    flap_hinge = object_model.get_articulation("cabinet_to_lock_flap")

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    if cabinet_aabb is not None:
        mins, maxs = cabinet_aabb
        dims = tuple(maxs[i] - mins[i] for i in range(3))
    else:
        dims = (0.0, 0.0, 0.0)
    ctx.check(
        "store-scale deep insulated cabinet",
        dims[0] > 1.45 and dims[1] > 0.65 and dims[2] > 0.85,
        details=f"cabinet dims={dims}",
    )

    ctx.expect_gap(
        lower_lid,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="front_shoe",
        negative_elem="lower_track_front",
        name="lower glass slider sits on lower front track",
    )
    ctx.expect_gap(
        upper_lid,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="front_shoe",
        negative_elem="upper_track_front",
        name="upper glass slider sits on upper front track",
    )
    ctx.expect_overlap(
        lower_lid,
        upper_lid,
        axes="xy",
        min_overlap=0.045,
        elem_a="glass_pane",
        elem_b="glass_pane",
        name="glass lids visibly overlap in plan like store freezer sliders",
    )

    ctx.expect_overlap(
        lock_flap,
        cabinet,
        axes="yz",
        min_overlap=0.045,
        elem_a="flap_panel",
        elem_b="key_cylinder",
        name="side lock flap covers the key cylinder",
    )
    ctx.expect_gap(
        lock_flap,
        cabinet,
        axis="x",
        min_gap=0.004,
        max_gap=0.025,
        positive_elem="flap_panel",
        negative_elem="key_cylinder",
        name="closed lock flap stands proud of key cylinder",
    )
    ctx.expect_contact(
        lock_flap,
        cabinet,
        contact_tol=0.001,
        elem_a="hinge_barrel",
        elem_b="hinge_leaf",
        name="lock flap hinge barrel is mounted to side hinge leaf",
    )

    lower_rest = ctx.part_world_position(lower_lid)
    with ctx.pose({lower_slide: 0.45}):
        lower_extended = ctx.part_world_position(lower_lid)
    ctx.check(
        "lower glass lid slides along its track",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[0] > lower_rest[0] + 0.40,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(upper_lid)
    with ctx.pose({upper_slide: 0.45}):
        upper_extended = ctx.part_world_position(upper_lid)
    ctx.check(
        "upper glass lid slides oppositely along its track",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[0] < upper_rest[0] - 0.40,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    flap_closed = ctx.part_world_aabb(lock_flap)
    with ctx.pose({flap_hinge: 1.20}):
        flap_open = ctx.part_world_aabb(lock_flap)
    closed_max_x = flap_closed[1][0] if flap_closed is not None else 0.0
    open_max_x = flap_open[1][0] if flap_open is not None else 0.0
    ctx.check(
        "side lock flap rotates outward on short hinge",
        open_max_x > closed_max_x + 0.05,
        details=f"closed_max_x={closed_max_x}, open_max_x={open_max_x}",
    )

    return ctx.report()


object_model = build_object_model()
