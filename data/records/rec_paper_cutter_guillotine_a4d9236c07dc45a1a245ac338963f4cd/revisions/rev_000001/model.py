from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paper_cutter")

    base_dark = model.material("base_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    anodized = model.material("anodized", rgba=(0.62, 0.65, 0.69, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.86, 0.88, 0.90, 1.0))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))
    guard_clear = model.material("guard_clear", rgba=(0.72, 0.86, 0.92, 0.33))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.46, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=base_dark,
        name="base_slab",
    )
    base.visual(
        Box((0.56, 0.40, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=steel,
        name="bed_plate",
    )
    base.visual(
        Box((0.012, 0.34, 0.022)),
        origin=Origin(xyz=(-0.215, 0.0, 0.045)),
        material=anodized,
        name="left_fence",
    )
    base.visual(
        Box((0.32, 0.028, 0.001)),
        origin=Origin(xyz=(-0.060, -0.206, 0.0345)),
        material=anodized,
        name="front_rule_strip",
    )
    base.visual(
        Box((0.010, 0.34, 0.004)),
        origin=Origin(xyz=(0.186, 0.0, 0.036)),
        material=blade_steel,
        name="cut_anvil",
    )
    for support_name, x_pos in (("rail_support_left", -0.155), ("rail_support_right", -0.015)):
        base.visual(
            Box((0.024, 0.030, 0.006)),
            origin=Origin(xyz=(x_pos, -0.165, 0.031)),
            material=base_dark,
            name=support_name,
        )
    for y_pos in (-0.160, 0.160):
        base.visual(
            Box((0.014, 0.020, 0.041)),
            origin=Origin(xyz=(0.154, y_pos, 0.0545)),
            material=anodized,
            name=f"guard_post_{'front' if y_pos < 0 else 'rear'}",
        )
        base.visual(
            Box((0.014, 0.020, 0.030)),
            origin=Origin(xyz=(0.154, y_pos, 0.090)),
            material=anodized,
            name=f"guard_block_{'front' if y_pos < 0 else 'rear'}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.62, 0.46, 0.12)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    gauge_rail = model.part("gauge_rail")
    gauge_rail.visual(
        Box((0.22, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=anodized,
        name="rail_bar",
    )
    gauge_rail.visual(
        Box((0.22, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.010, 0.017)),
        material=steel,
        name="rail_cap",
    )
    for x_pos in (-0.105, 0.105):
        gauge_rail.visual(
            Box((0.010, 0.028, 0.020)),
            origin=Origin(xyz=(x_pos, 0.0, 0.010)),
            material=base_dark,
            name=f"rail_stop_{'left' if x_pos < 0 else 'right'}",
        )
    gauge_rail.inertial = Inertial.from_geometry(
        Box((0.22, 0.03, 0.022)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    side_gauge = model.part("side_gauge")
    side_gauge.visual(
        Box((0.048, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_dark,
        name="gauge_carriage",
    )
    side_gauge.visual(
        Box((0.006, 0.300, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=anodized,
        name="gauge_fence",
    )
    side_gauge.visual(
        Box((0.020, 0.030, 0.016)),
        origin=Origin(xyz=(0.011, 0.0, 0.026)),
        material=base_dark,
        name="gauge_brace",
    )
    side_gauge.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.020, 0.0, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="thumb_knob",
    )
    side_gauge.inertial = Inertial.from_geometry(
        Box((0.05, 0.30, 0.05)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    pivot_bracket = model.part("pivot_bracket")
    pivot_bracket.visual(
        Box((0.022, 0.052, 0.008)),
        origin=Origin(xyz=(-0.020, 0.0, -0.022)),
        material=base_dark,
        name="left_mount_pad",
    )
    pivot_bracket.visual(
        Box((0.022, 0.052, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.022)),
        material=base_dark,
        name="right_mount_pad",
    )
    pivot_bracket.visual(
        Box((0.010, 0.028, 0.066)),
        origin=Origin(xyz=(-0.018, 0.0, 0.011)),
        material=anodized,
        name="left_cheek",
    )
    pivot_bracket.visual(
        Box((0.010, 0.028, 0.066)),
        origin=Origin(xyz=(0.018, 0.0, 0.011)),
        material=anodized,
        name="right_cheek",
    )
    pivot_bracket.visual(
        Box((0.052, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, 0.030)),
        material=anodized,
        name="top_bridge",
    )
    pivot_bracket.inertial = Inertial.from_geometry(
        Box((0.09, 0.06, 0.08)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="pivot_hub",
    )
    blade_arm.visual(
        Box((0.018, 0.048, 0.020)),
        origin=Origin(xyz=(0.0, -0.022, -0.004)),
        material=base_dark,
        name="arm_neck",
    )
    blade_arm.visual(
        Box((0.060, 0.090, 0.016)),
        origin=Origin(xyz=(-0.030, -0.095, -0.010)),
        material=base_dark,
        name="blade_root",
    )
    blade_arm.visual(
        Box((0.014, 0.330, 0.012)),
        origin=Origin(xyz=(-0.058, -0.205, -0.018)),
        material=base_dark,
        name="blade_backer",
    )
    blade_arm.visual(
        Box((0.022, 0.360, 0.016)),
        origin=Origin(xyz=(-0.012, -0.220, 0.004)),
        material=base_dark,
        name="handle_spine",
    )
    blade_arm.visual(
        Cylinder(radius=0.014, length=0.082),
        origin=Origin(xyz=(-0.012, -0.372, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="handle_grip",
    )
    blade_arm.visual(
        Box((0.004, 0.336, 0.006)),
        origin=Origin(xyz=(-0.066, -0.212, -0.024)),
        material=blade_steel,
        name="blade_edge",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.12, 0.46, 0.07)),
        mass=1.8,
        origin=Origin(xyz=(-0.035, -0.22, 0.0)),
    )

    finger_guard = model.part("finger_guard")
    finger_guard.visual(
        Cylinder(radius=0.008, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="hinge_barrel",
    )
    finger_guard.visual(
        Box((0.025, 0.300, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, -0.003)),
        material=anodized,
        name="guard_top_rail",
    )
    finger_guard.visual(
        Box((0.003, 0.300, 0.060)),
        origin=Origin(xyz=(0.022, 0.0, -0.030)),
        material=guard_clear,
        name="guard_panel",
    )
    finger_guard.visual(
        Box((0.010, 0.300, 0.008)),
        origin=Origin(xyz=(0.017, 0.0, -0.056)),
        material=anodized,
        name="guard_bottom_rail",
    )
    finger_guard.inertial = Inertial.from_geometry(
        Box((0.03, 0.30, 0.07)),
        mass=0.4,
        origin=Origin(xyz=(0.012, 0.0, -0.028)),
    )

    model.articulation(
        "base_to_gauge_rail",
        ArticulationType.FIXED,
        parent=base,
        child=gauge_rail,
        origin=Origin(xyz=(-0.080, -0.165, 0.034)),
    )
    model.articulation(
        "rail_to_side_gauge",
        ArticulationType.PRISMATIC,
        parent=gauge_rail,
        child=side_gauge,
        origin=Origin(xyz=(-0.085, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.20, lower=0.0, upper=0.160),
    )
    model.articulation(
        "base_to_pivot_bracket",
        ArticulationType.FIXED,
        parent=base,
        child=pivot_bracket,
        origin=Origin(xyz=(0.250, 0.184, 0.060)),
    )
    model.articulation(
        "pivot_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=pivot_bracket,
        child=blade_arm,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-1.20, upper=0.0),
    )
    model.articulation(
        "base_to_finger_guard",
        ArticulationType.REVOLUTE,
        parent=base,
        child=finger_guard,
        origin=Origin(xyz=(0.154, 0.0, 0.096)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-1.20, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    gauge_rail = object_model.get_part("gauge_rail")
    side_gauge = object_model.get_part("side_gauge")
    pivot_bracket = object_model.get_part("pivot_bracket")
    blade_arm = object_model.get_part("blade_arm")
    finger_guard = object_model.get_part("finger_guard")

    rail_slide = object_model.get_articulation("rail_to_side_gauge")
    blade_hinge = object_model.get_articulation("pivot_to_blade_arm")
    guard_hinge = object_model.get_articulation("base_to_finger_guard")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "side gauge axis is x",
        tuple(rail_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={rail_slide.axis}",
    )
    ctx.check(
        "blade hinge axis is x",
        tuple(blade_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={blade_hinge.axis}",
    )
    ctx.check(
        "finger guard axis is y",
        tuple(guard_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={guard_hinge.axis}",
    )

    ctx.expect_contact(gauge_rail, base, name="gauge rail mounted to base")
    ctx.expect_contact(side_gauge, gauge_rail, name="side gauge rides on rail")
    ctx.expect_contact(pivot_bracket, base, name="pivot bracket mounted to base")
    ctx.expect_contact(
        blade_arm,
        pivot_bracket,
        elem_a="pivot_hub",
        elem_b="left_cheek",
        name="blade hub touches bracket cheek",
    )
    ctx.expect_contact(
        blade_arm,
        base,
        elem_a="blade_edge",
        elem_b="cut_anvil",
        name="blade edge seats on cut anvil",
    )
    ctx.expect_contact(finger_guard, base, name="finger guard retained by base blocks")

    ctx.expect_gap(
        side_gauge,
        gauge_rail,
        axis="z",
        positive_elem="gauge_carriage",
        negative_elem="rail_cap",
        max_gap=0.001,
        max_penetration=1e-6,
        name="side gauge sits on top of rail",
    )
    ctx.expect_overlap(
        side_gauge,
        gauge_rail,
        axes="xy",
        min_overlap=0.020,
        name="side gauge carriage overlaps rail footprint",
    )
    ctx.expect_gap(
        finger_guard,
        base,
        axis="z",
        positive_elem="guard_panel",
        negative_elem="bed_plate",
        min_gap=0.0,
        max_gap=0.004,
        name="finger guard hangs just above base",
    )

    gauge_rest = ctx.part_world_position(side_gauge)
    assert gauge_rest is not None
    with ctx.pose({rail_slide: 0.140}):
        gauge_open = ctx.part_world_position(side_gauge)
        assert gauge_open is not None
        ctx.check(
            "side gauge translates along rail",
            gauge_open[0] > gauge_rest[0] + 0.12 and abs(gauge_open[1] - gauge_rest[1]) < 1e-6,
            details=f"rest={gauge_rest}, moved={gauge_open}",
        )
        ctx.expect_contact(side_gauge, gauge_rail, name="side gauge stays supported at extension")

    guard_rest = ctx.part_element_world_aabb(finger_guard, elem="guard_panel")
    assert guard_rest is not None
    with ctx.pose({guard_hinge: -1.0}):
        guard_open = ctx.part_element_world_aabb(finger_guard, elem="guard_panel")
        assert guard_open is not None
        ctx.check(
            "finger guard swings upward",
            guard_open[0][2] > guard_rest[0][2] + 0.03,
            details=f"rest_zmin={guard_rest[0][2]:.4f}, open_zmin={guard_open[0][2]:.4f}",
        )
        ctx.expect_contact(finger_guard, base, name="finger guard remains on hinge supports")

    arm_rest = ctx.part_world_aabb(blade_arm)
    assert arm_rest is not None
    with ctx.pose({blade_hinge: -1.0}):
        arm_open = ctx.part_world_aabb(blade_arm)
        assert arm_open is not None
        ctx.check(
            "blade arm opens upward",
            arm_open[1][2] > arm_rest[1][2] + 0.16,
            details=f"rest_zmax={arm_rest[1][2]:.4f}, open_zmax={arm_open[1][2]:.4f}",
        )
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem="blade_edge",
            negative_elem="cut_anvil",
            min_gap=0.04,
            name="opened blade arm clears base",
        )
        ctx.expect_contact(
            blade_arm,
            pivot_bracket,
            elem_a="pivot_hub",
            elem_b="right_cheek",
            name="blade arm remains retained at pivot",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
