from __future__ import annotations

import math

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
    model = ArticulatedObject(name="single_louvered_storm_shutter")

    painted_wood = Material("painted_wood", rgba=(0.17, 0.34, 0.53, 1.0))
    trim_paint = Material("white_exterior_trim", rgba=(0.90, 0.88, 0.82, 1.0))
    dark_glass = Material("dark_recessed_glass", rgba=(0.04, 0.06, 0.08, 1.0))
    black_metal = Material("black_powder_coated_metal", rgba=(0.02, 0.02, 0.018, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.62, 0.61, 0.56, 1.0))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    frame = model.part("wall_frame")
    frame.visual(
        Box((0.86, 0.040, 1.26)),
        origin=Origin(xyz=(0.28, -0.030, 0.0)),
        material=trim_paint,
        name="wall_backer",
    )
    frame.visual(
        Box((0.50, 0.010, 0.92)),
        origin=Origin(xyz=(0.29, -0.006, 0.0)),
        material=dark_glass,
        name="dark_opening",
    )
    frame.visual(
        Box((0.060, 0.045, 1.10)),
        origin=Origin(xyz=(-0.030, 0.000, 0.0)),
        material=trim_paint,
        name="hinge_jamb",
    )
    frame.visual(
        Box((0.060, 0.045, 1.10)),
        origin=Origin(xyz=(0.590, 0.000, 0.0)),
        material=trim_paint,
        name="outer_jamb",
    )
    frame.visual(
        Box((0.680, 0.045, 0.060)),
        origin=Origin(xyz=(0.280, 0.000, 0.520)),
        material=trim_paint,
        name="head_trim",
    )
    frame.visual(
        Box((0.680, 0.045, 0.060)),
        origin=Origin(xyz=(0.280, 0.000, -0.520)),
        material=trim_paint,
        name="sill_trim",
    )

    hinge_centers = (-0.365, 0.0, 0.365)
    for i, zc in enumerate(hinge_centers):
        frame.visual(
            Box((0.040, 0.020, 0.160)),
            origin=Origin(xyz=(-0.020, 0.027, zc)),
            material=black_metal,
            name=f"frame_hinge_leaf_{i}",
        )
        for suffix, dz in (("lower", -0.055), ("upper", 0.055)):
            frame.visual(
                Cylinder(radius=0.012, length=0.050),
                origin=Origin(xyz=(0.0, 0.055, zc + dz)),
                material=black_metal,
                name=f"frame_hinge_knuckle_{i}_{suffix}",
            )
            frame.visual(
                Box((0.014, 0.014, 0.052)),
                origin=Origin(xyz=(-0.006, 0.040, zc + dz)),
                material=black_metal,
                name=f"frame_hinge_curl_{i}_{suffix}",
            )
        frame.visual(
            Cylinder(radius=0.004, length=0.170),
            origin=Origin(xyz=(0.0, 0.055, zc)),
            material=galvanized,
            name=f"hinge_pin_{i}",
        )
        frame.visual(
            Cylinder(radius=0.0045, length=0.042),
            origin=Origin(xyz=(-0.020, 0.052, zc + 0.040), rpy=(math.pi / 2, 0.0, 0.0)),
            material=galvanized,
            name=f"frame_hinge_screw_{i}_0",
        )
        frame.visual(
            Cylinder(radius=0.0045, length=0.042),
            origin=Origin(xyz=(-0.020, 0.052, zc - 0.040), rpy=(math.pi / 2, 0.0, 0.0)),
            material=galvanized,
            name=f"frame_hinge_screw_{i}_1",
        )

    frame.visual(
        Box((0.080, 0.018, 0.060)),
        origin=Origin(xyz=(0.365, -0.002, -0.435)),
        material=black_metal,
        name="wall_stay_socket",
    )

    panel = model.part("panel")
    panel_width = 0.560
    panel_height = 1.040
    stile_w = 0.060
    rail_h = 0.070
    panel_thick = 0.045
    panel.visual(
        Box((stile_w, panel_thick, panel_height)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=painted_wood,
        name="hinge_stile",
    )
    panel.visual(
        Box((stile_w, panel_thick, panel_height)),
        origin=Origin(xyz=(0.530, 0.0, 0.0)),
        material=painted_wood,
        name="outer_stile",
    )
    panel.visual(
        Box((panel_width, panel_thick, rail_h)),
        origin=Origin(xyz=(0.295, 0.0, 0.485)),
        material=painted_wood,
        name="top_rail",
    )
    panel.visual(
        Box((panel_width, panel_thick, rail_h)),
        origin=Origin(xyz=(0.295, 0.0, -0.485)),
        material=painted_wood,
        name="bottom_rail",
    )
    panel.visual(
        Box((0.430, 0.020, 0.018)),
        origin=Origin(xyz=(0.295, -0.001, 0.430)),
        material=painted_wood,
        name="upper_louver_stop",
    )
    panel.visual(
        Box((0.430, 0.020, 0.018)),
        origin=Origin(xyz=(0.295, -0.001, -0.430)),
        material=painted_wood,
        name="lower_louver_stop",
    )

    for i, zc in enumerate(hinge_centers):
        panel.visual(
            Box((0.060, 0.008, 0.160)),
            origin=Origin(xyz=(0.045, 0.0, zc)),
            material=black_metal,
            name=f"panel_hinge_leaf_{i}",
        )
        panel.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=black_metal,
            name=f"panel_hinge_knuckle_{i}",
        )
        panel.visual(
            Box((0.014, 0.018, 0.052)),
            origin=Origin(xyz=(0.010, 0.0, zc)),
            material=black_metal,
            name=f"panel_hinge_curl_{i}",
        )

    panel.visual(
        Box((0.065, 0.020, 0.070)),
        origin=Origin(xyz=(0.525, 0.030, -0.435)),
        material=black_metal,
        name="arm_pivot_plate",
    )

    panel_hinge = model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    panel_hinge.meta["description"] = "Vertical side hinge line; positive motion swings the shutter panel outward."

    louver_z = [-0.360, -0.270, -0.180, -0.090, 0.0, 0.090, 0.180, 0.270, 0.360]
    for i, zc in enumerate(louver_z):
        louver = model.part(f"louver_{i}")
        louver.visual(
            Box((0.410, 0.018, 0.076)),
            origin=Origin(rpy=(math.radians(-32.0), 0.0, 0.0)),
            material=painted_wood,
            name="slat",
        )
        louver.visual(
            Cylinder(radius=0.006, length=0.405),
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=black_metal,
            name="long_axis_pin",
        )
        model.articulation(
            f"panel_to_louver_{i}",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(xyz=(0.295, 0.0, zc)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.55, upper=0.55),
        )

    support_arm = model.part("support_arm")
    support_arm.visual(
        Box((0.220, 0.024, 0.018)),
        origin=Origin(xyz=(-0.110, 0.0, 0.0)),
        material=galvanized,
        name="flat_stay_bar",
    )
    support_arm.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(),
        material=galvanized,
        name="panel_pivot_boss",
    )
    support_arm.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(-0.220, 0.0, 0.0)),
        material=galvanized,
        name="tip_pivot_boss",
    )
    model.articulation(
        "panel_to_support_arm",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=support_arm,
        origin=Origin(xyz=(0.525, 0.052, -0.435)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=-0.85, upper=0.95),
    )

    arm_tip = model.part("arm_tip")
    arm_tip.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(),
        material=black_metal,
        name="end_pivot_eye",
    )
    arm_tip.visual(
        Box((0.048, 0.018, 0.030)),
        origin=Origin(xyz=(-0.039, 0.0, 0.0)),
        material=rubber,
        name="rubber_stop_pad",
    )
    model.articulation(
        "support_arm_to_tip",
        ArticulationType.REVOLUTE,
        parent=support_arm,
        child=arm_tip,
        origin=Origin(xyz=(-0.220, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.90, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("wall_frame")
    panel = object_model.get_part("panel")
    arm = object_model.get_part("support_arm")
    tip = object_model.get_part("arm_tip")
    panel_hinge = object_model.get_articulation("frame_to_panel")
    arm_pivot = object_model.get_articulation("panel_to_support_arm")
    tip_pivot = object_model.get_articulation("support_arm_to_tip")

    ctx.check(
        "one side-hung shutter panel",
        panel_hinge.motion_limits is not None
        and panel_hinge.motion_limits.lower == 0.0
        and panel_hinge.motion_limits.upper > 1.5,
        details="The main panel should swing from closed to a wide hold-open angle.",
    )
    ctx.check(
        "nine independently pivoted louvers",
        sum(1 for joint in object_model.articulations if joint.name.startswith("panel_to_louver_")) == 9,
        details="Expected a bank of long-axis louver pivots across the large panel.",
    )
    ctx.allow_overlap(
        panel,
        arm,
        elem_a="arm_pivot_plate",
        elem_b="panel_pivot_boss",
        reason="The stay-arm pivot boss is intentionally seated through the outer-stile mounting plate.",
    )
    ctx.expect_overlap(
        panel,
        arm,
        axes="xy",
        min_overlap=0.007,
        elem_a="arm_pivot_plate",
        elem_b="panel_pivot_boss",
        name="stay arm boss is captured in the panel bracket",
    )
    ctx.allow_overlap(
        arm,
        tip,
        elem_a="tip_pivot_boss",
        elem_b="end_pivot_eye",
        reason="The end eye rotates around the captured rivet at the free end of the stay arm.",
    )
    ctx.expect_overlap(
        arm,
        tip,
        axes="xy",
        min_overlap=0.012,
        elem_a="tip_pivot_boss",
        elem_b="end_pivot_eye",
        name="stay arm end eye is captured around its rivet",
    )
    ctx.allow_overlap(
        arm,
        tip,
        elem_a="flat_stay_bar",
        elem_b="end_pivot_eye",
        reason="The flat stay bar and its rotating eye share the same small riveted knuckle at the free end.",
    )
    ctx.expect_overlap(
        arm,
        tip,
        axes="xy",
        min_overlap=0.012,
        elem_a="flat_stay_bar",
        elem_b="end_pivot_eye",
        name="stay bar reaches the rotating end eye",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="xz",
        min_overlap=0.45,
        elem_b="dark_opening",
        name="closed shutter covers the framed opening",
    )
    for i in range(3):
        ctx.allow_overlap(
            frame,
            panel,
            elem_a=f"hinge_pin_{i}",
            elem_b=f"panel_hinge_knuckle_{i}",
            reason="The fixed hinge pin intentionally passes through the shutter-side knuckle.",
        )
        ctx.expect_within(
            frame,
            panel,
            axes="xy",
            inner_elem=f"hinge_pin_{i}",
            outer_elem=f"panel_hinge_knuckle_{i}",
            margin=0.001,
            name=f"hinge pin {i} sits inside panel knuckle",
        )
        ctx.expect_overlap(
            frame,
            panel,
            axes="z",
            min_overlap=0.045,
            elem_a=f"hinge_pin_{i}",
            elem_b=f"panel_hinge_knuckle_{i}",
            name=f"hinge pin {i} spans the panel knuckle",
        )

    closed_tip = ctx.part_world_position(tip)
    with ctx.pose({panel_hinge: 1.20}):
        open_panel = ctx.part_world_aabb(panel)
        ctx.check(
            "panel swings outward",
            open_panel is not None and open_panel[1][1] > 0.50,
            details=f"open panel aabb={open_panel}",
        )

    with ctx.pose({arm_pivot: -0.60}):
        arm_only_tip = ctx.part_world_position(tip)
        arm_only_pad = ctx.part_element_world_aabb(tip, elem="rubber_stop_pad")
    with ctx.pose({arm_pivot: -0.60, tip_pivot: 0.50}):
        moved_tip = ctx.part_world_position(tip)
        moved_pad = ctx.part_element_world_aabb(tip, elem="rubber_stop_pad")
    arm_only_pad_y = None if arm_only_pad is None else (arm_only_pad[0][1] + arm_only_pad[1][1]) / 2.0
    moved_pad_y = None if moved_pad is None else (moved_pad[0][1] + moved_pad[1][1]) / 2.0
    ctx.check(
        "hold-open arm pivots on both joints",
        closed_tip is not None
        and arm_only_tip is not None
        and moved_tip is not None
        and arm_only_pad_y is not None
        and moved_pad_y is not None
        and abs(arm_only_tip[1] - closed_tip[1]) > 0.08
        and abs(moved_pad_y - arm_only_pad_y) > 0.006,
        details=f"tip closed={closed_tip}, tip arm-only={arm_only_tip}, pad_y arm-only={arm_only_pad_y}, pad_y moved={moved_pad_y}",
    )

    first_louver = object_model.get_part("louver_0")
    louver_joint = object_model.get_articulation("panel_to_louver_0")
    closed_louver = ctx.part_element_world_aabb(first_louver, elem="slat")
    with ctx.pose({louver_joint: 0.50}):
        tilted_louver = ctx.part_element_world_aabb(first_louver, elem="slat")
    ctx.check(
        "louver rotates on its long horizontal pivot",
        closed_louver is not None
        and tilted_louver is not None
        and abs((tilted_louver[1][1] - tilted_louver[0][1]) - (closed_louver[1][1] - closed_louver[0][1])) > 0.01,
        details=f"closed aabb={closed_louver}, tilted aabb={tilted_louver}",
    )

    return ctx.report()


object_model = build_object_model()
