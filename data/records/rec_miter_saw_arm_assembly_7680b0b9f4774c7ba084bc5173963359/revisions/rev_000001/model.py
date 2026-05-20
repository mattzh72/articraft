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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_miter_saw")

    cast_gray = model.material("cast_gray", color=(0.24, 0.25, 0.27))
    steel = model.material("steel", color=(0.76, 0.77, 0.79))
    aluminum = model.material("aluminum", color=(0.68, 0.69, 0.71))
    guard_orange = model.material("guard_orange", color=(0.90, 0.47, 0.10))
    black = model.material("black", color=(0.12, 0.12, 0.12))

    base_width = 0.62
    base_depth = 0.38
    base_height = 0.08
    table_seat_radius = 0.16
    table_seat_height = 0.016
    table_joint_z = base_height + table_seat_height

    def x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
        return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))

    base = model.part("base")
    base_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(base_width, base_depth, 0.035),
        base_height,
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_shell"),
        material=cast_gray,
        name="deck",
    )
    base.visual(
        Cylinder(radius=table_seat_radius, length=table_seat_height),
        origin=Origin(xyz=(0.0, 0.0, base_height + table_seat_height / 2.0)),
        material=steel,
        name="table_seat",
    )
    base.visual(
        Box((0.17, 0.035, 0.05)),
        origin=Origin(xyz=(0.0, -0.1725, 0.105)),
        material=cast_gray,
        name="front_pivot_block",
    )
    base.visual(
        Box((0.03, 0.03, 0.07)),
        origin=Origin(xyz=(-0.065, -0.155, 0.165)),
        material=steel,
        name="front_lug_left",
    )
    base.visual(
        Box((0.03, 0.03, 0.07)),
        origin=Origin(xyz=(0.065, -0.155, 0.165)),
        material=steel,
        name="front_lug_right",
    )
    base.visual(
        Box((0.22, 0.018, 0.09)),
        origin=Origin(xyz=(-0.16, 0.165, 0.125)),
        material=aluminum,
        name="fence_left",
    )
    base.visual(
        Box((0.22, 0.018, 0.09)),
        origin=Origin(xyz=(0.16, 0.165, 0.125)),
        material=aluminum,
        name="fence_right",
    )
    base.visual(
        Box((0.04, 0.06, 0.22)),
        origin=Origin(xyz=(-0.08, 0.205, 0.19)),
        material=cast_gray,
        name="rear_column_left",
    )
    base.visual(
        Box((0.04, 0.06, 0.22)),
        origin=Origin(xyz=(0.08, 0.205, 0.19)),
        material=cast_gray,
        name="rear_column_right",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.145, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=aluminum,
        name="table_disc",
    )
    table.visual(
        Cylinder(radius=0.05, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="table_hub",
    )

    saw_arm = model.part("saw_arm")
    saw_arm.visual(
        Cylinder(radius=0.022, length=0.12),
        origin=x_axis_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    saw_arm.visual(
        Box((0.05, 0.23, 0.045)),
        origin=Origin(xyz=(0.0, -0.10, 0.085), rpy=(-0.78, 0.0, 0.0)),
        material=cast_gray,
        name="main_arm",
    )
    saw_arm.visual(
        Box((0.03, 0.16, 0.03)),
        origin=Origin(xyz=(0.0, -0.075, 0.145), rpy=(-0.78, 0.0, 0.0)),
        material=cast_gray,
        name="upper_link",
    )
    saw_arm.visual(
        Box((0.05, 0.14, 0.10)),
        origin=Origin(xyz=(-0.03, -0.145, 0.17)),
        material=cast_gray,
        name="arm_knuckle",
    )
    saw_arm.visual(
        Box((0.08, 0.10, 0.06)),
        origin=Origin(xyz=(-0.07, -0.205, 0.225)),
        material=cast_gray,
        name="motor_mount",
    )
    saw_arm.visual(
        Cylinder(radius=0.05, length=0.055),
        origin=x_axis_origin((-0.055, -0.225, 0.175)),
        material=cast_gray,
        name="gearbox_side",
    )
    saw_arm.visual(
        Box((0.03, 0.17, 0.12)),
        origin=Origin(xyz=(-0.075, -0.235, 0.205)),
        material=guard_orange,
        name="upper_guard_side",
    )
    saw_arm.visual(
        Box((0.08, 0.08, 0.06)),
        origin=Origin(xyz=(-0.055, -0.19, 0.285)),
        material=guard_orange,
        name="upper_guard_cap",
    )
    saw_arm.visual(
        Box((0.07, 0.06, 0.08)),
        origin=Origin(xyz=(-0.045, -0.305, 0.205)),
        material=guard_orange,
        name="guard_nose",
    )
    saw_arm.visual(
        Box((0.08, 0.05, 0.04)),
        origin=Origin(xyz=(-0.025, -0.185, 0.215)),
        material=steel,
        name="lower_guard_lug",
    )
    saw_arm.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=x_axis_origin((-0.075, -0.285, 0.295)),
        material=black,
        name="motor_can",
    )
    saw_arm.visual(
        Box((0.02, 0.065, 0.022)),
        origin=Origin(xyz=(-0.055, -0.305, 0.245), rpy=(-0.55, 0.0, 0.0)),
        material=black,
        name="handle_stem",
    )
    saw_arm.visual(
        Cylinder(radius=0.014, length=0.09),
        origin=x_axis_origin((-0.055, -0.335, 0.255)),
        material=black,
        name="handle_grip",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.152, length=0.003),
        origin=x_axis_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="blade_disc",
    )
    blade.visual(
        Cylinder(radius=0.03, length=0.032),
        origin=x_axis_origin((0.0, 0.0, 0.0)),
        material=black,
        name="blade_hub",
    )

    lower_guard = model.part("lower_guard")
    lower_guard_arc = wire_from_points(
        [
            (0.034, 0.0, 0.0),
            (0.034, 0.055, -0.055),
            (0.034, 0.028, -0.135),
            (0.034, -0.06, -0.17),
            (0.034, -0.122, -0.10),
            (0.034, -0.112, -0.012),
        ],
        radius=0.012,
        corner_mode="fillet",
        corner_radius=0.03,
        radial_segments=18,
        cap_ends=True,
    )
    lower_guard.visual(
        mesh_from_geometry(lower_guard_arc, "lower_guard_arc"),
        material=guard_orange,
        name="guard_arc",
    )
    lower_guard.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=x_axis_origin((0.022, 0.0, 0.0)),
        material=steel,
        name="guard_hinge_barrel",
    )
    lower_guard.visual(
        Box((0.036, 0.09, 0.05)),
        origin=Origin(xyz=(0.034, -0.055, -0.12)),
        material=guard_orange,
        name="guard_panel",
    )

    clamp_arm = model.part("clamp_arm")
    clamp_body = wire_from_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.03, 0.03),
            (0.0, 0.12, 0.15),
            (0.0, 0.18, 0.15),
            (0.0, 0.18, 0.05),
        ],
        radius=0.011,
        corner_mode="fillet",
        corner_radius=0.025,
        radial_segments=16,
        cap_ends=True,
    )
    clamp_arm.visual(
        mesh_from_geometry(clamp_body, "clamp_arm_body"),
        material=black,
        name="arm_rod",
    )
    clamp_arm.visual(
        Cylinder(radius=0.015, length=0.10),
        origin=x_axis_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="clamp_barrel",
    )
    clamp_arm.visual(
        Box((0.02, 0.02, 0.054)),
        origin=Origin(xyz=(0.0, 0.18, 0.027)),
        material=black,
        name="shoe_post",
    )
    clamp_arm.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.18, -0.006)),
        material=steel,
        name="pressure_shoe",
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, table_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "base_to_saw_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=saw_arm,
        origin=Origin(xyz=(0.0, 0.205, 0.26)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=0.55,
        ),
    )
    model.articulation(
        "saw_arm_to_blade",
        ArticulationType.CONTINUOUS,
        parent=saw_arm,
        child=blade,
        origin=Origin(xyz=(0.0, -0.245, 0.17)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=40.0),
    )
    model.articulation(
        "saw_arm_to_lower_guard",
        ArticulationType.REVOLUTE,
        parent=saw_arm,
        child=lower_guard,
        origin=Origin(xyz=(0.015, -0.185, 0.215)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=-0.1,
            upper=1.15,
        ),
    )
    model.articulation(
        "base_to_clamp_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clamp_arm,
        origin=Origin(xyz=(0.0, -0.155, 0.165)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-1.1,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    table = object_model.get_part("table")
    saw_arm = object_model.get_part("saw_arm")
    blade = object_model.get_part("blade")
    lower_guard = object_model.get_part("lower_guard")
    clamp_arm = object_model.get_part("clamp_arm")

    table_joint = object_model.get_articulation("base_to_table")
    saw_joint = object_model.get_articulation("base_to_saw_arm")
    blade_joint = object_model.get_articulation("saw_arm_to_blade")
    guard_joint = object_model.get_articulation("saw_arm_to_lower_guard")
    clamp_joint = object_model.get_articulation("base_to_clamp_arm")

    def axis_matches(joint, expected: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-9 for a, b in zip(joint.axis, expected))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
        "table rotates about vertical axis",
        axis_matches(table_joint, (0.0, 0.0, 1.0)),
        f"axis={table_joint.axis}",
    )
    ctx.check(
        "saw head pivots on rear horizontal axis",
        axis_matches(saw_joint, (1.0, 0.0, 0.0)),
        f"axis={saw_joint.axis}",
    )
    ctx.check(
        "blade spins on arbor axis",
        axis_matches(blade_joint, (1.0, 0.0, 0.0)),
        f"axis={blade_joint.axis}",
    )
    ctx.check(
        "lower guard uses a horizontal hinge axis",
        axis_matches(guard_joint, (1.0, 0.0, 0.0)),
        f"axis={guard_joint.axis}",
    )
    ctx.check(
        "front clamp arm uses its own horizontal pivot",
        axis_matches(clamp_joint, (1.0, 0.0, 0.0)),
        f"axis={clamp_joint.axis}",
    )

    ctx.expect_contact(
        saw_arm,
        base,
        elem_a="pivot_barrel",
        elem_b="rear_column_left",
        name="rear pivot barrel is captured by base support",
    )
    ctx.expect_contact(
        clamp_arm,
        base,
        elem_a="clamp_barrel",
        elem_b="front_lug_left",
        name="front clamp barrel stays clipped into base lug",
    )
    ctx.expect_gap(
        table,
        base,
        axis="z",
        positive_elem="table_disc",
        negative_elem="table_seat",
        min_gap=0.0,
        max_gap=0.006,
        name="turntable sits on machined seat",
    )
    ctx.expect_overlap(
        table,
        base,
        axes="xy",
        elem_a="table_disc",
        elem_b="table_seat",
        min_overlap=0.25,
        name="turntable stays centered on seat",
    )
    ctx.expect_gap(
        lower_guard,
        blade,
        axis="x",
        positive_elem="guard_arc",
        negative_elem="blade_disc",
        min_gap=0.01,
        name="lower guard rides outside blade plane",
    )
    ctx.expect_gap(
        blade,
        saw_arm,
        axis="x",
        positive_elem="blade_disc",
        negative_elem="gearbox_side",
        min_gap=0.02,
        name="blade clears gearbox side housing",
    )
    ctx.expect_origin_gap(
        table,
        clamp_arm,
        axis="y",
        min_gap=0.12,
        name="clamp pivot is mounted at front of base",
    )
    ctx.expect_origin_gap(
        saw_arm,
        table,
        axis="y",
        min_gap=0.16,
        name="saw pivot is mounted behind table center",
    )

    with ctx.pose({saw_joint: 0.0}):
        ctx.expect_gap(
            blade,
            table,
            axis="z",
            positive_elem="blade_disc",
            negative_elem="table_disc",
            min_gap=0.12,
            name="raised blade starts well above table",
        )

    with ctx.pose({saw_joint: 0.55}):
        ctx.expect_gap(
            blade,
            table,
            axis="z",
            positive_elem="blade_disc",
            negative_elem="table_disc",
            min_gap=0.0,
            max_gap=0.015,
            name="saw head lowers blade near cut plane",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
