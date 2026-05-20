from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.92
BASE_WIDTH = 0.18
BASE_PLATE_HEIGHT = 0.032
BASE_RAIL_HEIGHT = 0.018
BASE_RAIL_WIDTH = 0.028
BASE_RAIL_OFFSET_Y = 0.05
BASE_TOP_Z = BASE_PLATE_HEIGHT + BASE_RAIL_HEIGHT

X_CARRIAGE_LENGTH = 0.22
X_CARRIAGE_WIDTH = 0.15
X_CARRIAGE_BRIDGE_HEIGHT = 0.026
X_CARRIAGE_TOP_HEIGHT = 0.024
X_CARRIAGE_HEIGHT = X_CARRIAGE_BRIDGE_HEIGHT + X_CARRIAGE_TOP_HEIGHT

COLUMN_FOOT_HEIGHT = 0.024
COLUMN_SHELL_HEIGHT = 0.72
COLUMN_TOTAL_HEIGHT = COLUMN_FOOT_HEIGHT + COLUMN_SHELL_HEIGHT
Z_JOINT_BASE_Z = 0.03
Z_CARRIAGE_Y_OFFSET = 0.0

Z_CARRIAGE_TOP_Z = 0.82


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    if radius > 0.0:
        body = body.edges("|Z").fillet(radius)
    return body


def _build_base_frame() -> cq.Workplane:
    plate = _rounded_box(BASE_LENGTH, BASE_WIDTH, BASE_PLATE_HEIGHT, 0.006)
    rail_length = BASE_LENGTH - 0.06
    left_rail = _rounded_box(rail_length, BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT, 0.003).translate(
        (0.0, BASE_RAIL_OFFSET_Y, BASE_PLATE_HEIGHT)
    )
    right_rail = _rounded_box(rail_length, BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT, 0.003).translate(
        (0.0, -BASE_RAIL_OFFSET_Y, BASE_PLATE_HEIGHT)
    )
    end_pads = (
        _rounded_box(0.10, 0.12, 0.012, 0.004).translate((0.33, 0.0, 0.0))
        .union(_rounded_box(0.10, 0.12, 0.012, 0.004).translate((-0.33, 0.0, 0.0)))
    )
    return plate.union(left_rail).union(right_rail).union(end_pads)


def _build_x_carriage() -> cq.Workplane:
    bridge = _rounded_box(
        X_CARRIAGE_LENGTH,
        X_CARRIAGE_WIDTH,
        X_CARRIAGE_BRIDGE_HEIGHT,
        0.004,
    )
    top_block = _rounded_box(0.16, 0.11, X_CARRIAGE_TOP_HEIGHT, 0.003).translate(
        (0.0, 0.0, X_CARRIAGE_BRIDGE_HEIGHT)
    )
    return bridge.union(top_block)


def _build_column_foot() -> cq.Workplane:
    return _rounded_box(0.18, 0.14, COLUMN_FOOT_HEIGHT, 0.005)


def _build_column_shell() -> cq.Workplane:
    left_rail = cq.Workplane("XY").box(0.018, 0.028, COLUMN_SHELL_HEIGHT, centered=(True, True, False)).translate(
        (0.049, -0.014, 0.0)
    )
    right_rail = cq.Workplane("XY").box(0.018, 0.028, COLUMN_SHELL_HEIGHT, centered=(True, True, False)).translate(
        (-0.049, -0.014, 0.0)
    )
    back_spine = cq.Workplane("XY").box(0.116, 0.010, COLUMN_SHELL_HEIGHT, centered=(True, True, False)).translate(
        (0.0, -0.023, 0.0)
    )
    lower_head = cq.Workplane("XY").box(0.102, 0.028, 0.07, centered=(True, True, False)).translate(
        (0.0, -0.014, 0.0)
    )
    upper_head = cq.Workplane("XY").box(0.094, 0.028, 0.055, centered=(True, True, False)).translate(
        (0.0, -0.014, COLUMN_SHELL_HEIGHT - 0.055)
    )
    mid_bridge = cq.Workplane("XY").box(0.05, 0.018, 0.18, centered=(True, True, False)).translate(
        (0.0, -0.019, 0.25)
    )
    frame = (
        left_rail
        .union(right_rail)
        .union(back_spine)
        .union(lower_head)
        .union(upper_head)
        .union(mid_bridge)
    )
    return frame.translate((0.0, 0.0, COLUMN_FOOT_HEIGHT))


def _build_z_carriage() -> cq.Workplane:
    rear_guide = cq.Workplane("XY").box(0.072, 0.016, 0.13, centered=(True, True, False)).translate(
        (0.0, 0.008, 0.0)
    )
    front_plate = cq.Workplane("XY").box(0.092, 0.016, 0.16, centered=(True, True, False)).translate(
        (0.0, 0.024, 0.0)
    )
    mast = cq.Workplane("XY").box(0.07, 0.036, 0.68, centered=(True, True, False)).translate(
        (0.0, 0.018, 0.12)
    )
    top_carrier = cq.Workplane("XY").box(0.09, 0.048, 0.07, centered=(True, True, False)).translate(
        (0.0, 0.024, 0.75)
    )
    return rear_guide.union(front_plate).union(mast).union(top_carrier)


def _build_head_pad_mount() -> cq.Workplane:
    stem = _rounded_box(0.04, 0.03, 0.09, 0.003)
    bracket = _rounded_box(0.075, 0.035, 0.012, 0.002).translate((0.0, 0.02, 0.07))
    return stem.union(bracket)


def _build_head_pad_cushion() -> cq.Workplane:
    cushion = _rounded_box(0.22, 0.10, 0.055, 0.012)
    return cushion.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -12.0).translate((0.0, 0.046, 0.08))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_positioning_axis")

    base_finish = model.material("base_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    carriage_finish = model.material("carriage_finish", rgba=(0.71, 0.73, 0.76, 1.0))
    column_finish = model.material("column_finish", rgba=(0.31, 0.34, 0.38, 1.0))
    lift_finish = model.material("lift_finish", rgba=(0.83, 0.84, 0.86, 1.0))
    mount_finish = model.material("mount_finish", rgba=(0.26, 0.27, 0.29, 1.0))
    pad_finish = model.material("pad_finish", rgba=(0.08, 0.08, 0.09, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_build_base_frame(), "base_frame"),
        material=base_finish,
        name="base_frame_body",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_build_x_carriage(), "x_carriage"),
        material=carriage_finish,
        name="x_carriage_body",
    )

    column_outer = model.part("column_outer")
    column_outer.visual(
        mesh_from_cadquery(_build_column_foot(), "column_foot"),
        material=column_finish,
        name="column_foot",
    )
    column_outer.visual(
        mesh_from_cadquery(_build_column_shell(), "column_shell"),
        material=column_finish,
        name="column_shell",
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        mesh_from_cadquery(_build_z_carriage(), "z_carriage"),
        material=lift_finish,
        name="z_carriage_body",
    )

    head_pad = model.part("head_pad")
    head_pad.visual(
        mesh_from_cadquery(_build_head_pad_mount(), "head_pad_mount"),
        material=mount_finish,
        name="head_pad_mount",
    )
    head_pad.visual(
        mesh_from_cadquery(_build_head_pad_cushion(), "head_pad_cushion"),
        material=pad_finish,
        name="head_pad_cushion",
    )

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.45,
            lower=-0.28,
            upper=0.28,
        ),
    )
    model.articulation(
        "x_carriage_to_column_outer",
        ArticulationType.FIXED,
        parent=x_carriage,
        child=column_outer,
        origin=Origin(xyz=(0.0, 0.0, X_CARRIAGE_HEIGHT)),
    )
    model.articulation(
        "column_outer_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=column_outer,
        child=z_carriage,
        origin=Origin(xyz=(0.0, Z_CARRIAGE_Y_OFFSET, Z_JOINT_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.28,
            lower=0.0,
            upper=0.24,
        ),
    )
    model.articulation(
        "z_carriage_to_head_pad",
        ArticulationType.FIXED,
        parent=z_carriage,
        child=head_pad,
        origin=Origin(xyz=(0.0, 0.0, Z_CARRIAGE_TOP_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    parts_by_name = {part.name: part for part in object_model.parts}
    joints_by_name = {joint.name: joint for joint in object_model.articulations}

    expected_parts = {
        "base_frame",
        "x_carriage",
        "column_outer",
        "z_carriage",
        "head_pad",
    }
    expected_joints = {
        "base_to_x_carriage",
        "x_carriage_to_column_outer",
        "column_outer_to_z_carriage",
        "z_carriage_to_head_pad",
    }

    for part_name in sorted(expected_parts):
        ctx.check(
            f"{part_name}_present",
            part_name in parts_by_name,
            f"Missing part: {part_name}",
        )
    for joint_name in sorted(expected_joints):
        ctx.check(
            f"{joint_name}_present",
            joint_name in joints_by_name,
            f"Missing articulation: {joint_name}",
        )

    if not expected_parts.issubset(parts_by_name) or not expected_joints.issubset(joints_by_name):
        return ctx.report()

    base_frame = parts_by_name["base_frame"]
    x_carriage = parts_by_name["x_carriage"]
    column_outer = parts_by_name["column_outer"]
    z_carriage = parts_by_name["z_carriage"]
    head_pad = parts_by_name["head_pad"]

    x_axis = joints_by_name["base_to_x_carriage"]
    z_axis = joints_by_name["column_outer_to_z_carriage"]

    ctx.expect_contact(
        x_carriage,
        base_frame,
        name="x_carriage_contacts_base_rails",
    )
    ctx.expect_overlap(
        x_carriage,
        base_frame,
        axes="xy",
        min_overlap=0.14,
        name="x_carriage_footprint_reads_as_low_base_slide",
    )
    ctx.expect_gap(
        column_outer,
        x_carriage,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="column_outer_sits_on_x_carriage",
    )
    ctx.expect_overlap(
        column_outer,
        x_carriage,
        axes="xy",
        min_overlap=0.12,
        name="column_outer_centered_on_x_carriage",
    )
    ctx.expect_overlap(
        z_carriage,
        column_outer,
        axes="x",
        min_overlap=0.07,
        name="z_carriage_aligned_under_column_guides",
    )
    ctx.expect_contact(
        head_pad,
        z_carriage,
        name="head_pad_mounted_to_z_carriage",
    )
    ctx.expect_gap(
        head_pad,
        column_outer,
        axis="z",
        min_gap=0.08,
        name="head_pad_clearly_above_column_shell",
    )

    x_limits = x_axis.motion_limits
    z_limits = z_axis.motion_limits
    ctx.check(
        "x_axis_configuration",
        x_axis.articulation_type == ArticulationType.PRISMATIC
        and x_axis.axis == (1.0, 0.0, 0.0)
        and x_limits is not None
        and x_limits.lower is not None
        and x_limits.upper is not None
        and x_limits.lower < 0.0 < x_limits.upper,
        f"Expected horizontal X prismatic axis, got axis={x_axis.axis}, limits={x_limits}",
    )
    ctx.check(
        "z_axis_configuration",
        z_axis.articulation_type == ArticulationType.PRISMATIC
        and z_axis.axis == (0.0, 0.0, 1.0)
        and z_limits is not None
        and z_limits.lower == 0.0
        and z_limits.upper is not None
        and z_limits.upper >= 0.20,
        f"Expected vertical Z prismatic axis, got axis={z_axis.axis}, limits={z_limits}",
    )

    column_rest = ctx.part_world_position(column_outer)
    z_carriage_rest = ctx.part_world_position(z_carriage)
    head_pad_rest = ctx.part_world_position(head_pad)

    with ctx.pose({x_axis: 0.18}):
        column_shifted = ctx.part_world_position(column_outer)
        ctx.expect_overlap(
            x_carriage,
            base_frame,
            axes="x",
            min_overlap=0.20,
            name="x_carriage_remains_engaged_at_shifted_pose",
        )

    with ctx.pose({z_axis: 0.22}):
        head_pad_lifted = ctx.part_world_position(head_pad)
        ctx.expect_gap(
            head_pad,
            column_outer,
            axis="z",
            min_gap=0.28,
            name="z_axis_creates_tall_lifted_stack",
        )

    if column_rest is not None and column_shifted is not None:
        ctx.check(
            "x_axis_translates_upper_stack_horizontally",
            abs((column_shifted[0] - column_rest[0]) - 0.18) <= 1e-5
            and abs(column_shifted[1] - column_rest[1]) <= 1e-5
            and abs(column_shifted[2] - column_rest[2]) <= 1e-5,
            f"Column rest={column_rest}, shifted={column_shifted}",
        )
    if column_rest is not None and z_carriage_rest is not None:
        ctx.check(
            "z_carriage_sits_forward_of_column_backbone",
            abs((z_carriage_rest[1] - column_rest[1]) - Z_CARRIAGE_Y_OFFSET) <= 1e-5,
            f"Column rest={column_rest}, z carriage rest={z_carriage_rest}",
        )
    if head_pad_rest is not None and head_pad_lifted is not None:
        ctx.check(
            "z_axis_lifts_head_pad_vertically",
            abs((head_pad_lifted[2] - head_pad_rest[2]) - 0.22) <= 1e-5
            and abs(head_pad_lifted[0] - head_pad_rest[0]) <= 1e-5
            and abs(head_pad_lifted[1] - head_pad_rest[1]) <= 1e-5,
            f"Head pad rest={head_pad_rest}, lifted={head_pad_lifted}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
