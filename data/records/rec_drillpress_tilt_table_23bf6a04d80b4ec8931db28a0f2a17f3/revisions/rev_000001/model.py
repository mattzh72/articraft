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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_cross_slide_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.27, 0.29, 0.31, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.10, 0.10, 0.11, 1.0))
    blackened_tool = model.material("blackened_tool", rgba=(0.18, 0.18, 0.20, 1.0))

    def shift_profile(
        profile: list[tuple[float, float]],
        dx: float = 0.0,
        dy: float = 0.0,
    ) -> list[tuple[float, float]]:
        return [(x + dx, y + dy) for x, y in profile]

    def xz_section(
        width: float,
        height: float,
        radius: float,
        y: float,
        z_offset: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_offset) for x, z in rounded_rect_profile(width, height, radius)]

    base = model.part("base")
    base_plate = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.34, 0.26, 0.028), 0.025),
        "base_plate",
    )
    base.visual(base_plate, origin=Origin(xyz=(0.0, 0.0, 0.0125)), material=cast_iron, name="base_plate")
    base.visual(
        Box((0.12, 0.11, 0.05)),
        origin=Origin(xyz=(0.0, -0.07, 0.05)),
        material=cast_iron,
        name="base_pedestal",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.03),
        origin=Origin(xyz=(0.0, -0.07, 0.09)),
        material=cast_iron,
        name="column_socket",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.26, 0.105)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.032, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=machined_steel,
        name="column_shaft",
    )
    column.visual(
        Box((0.012, 0.018, 0.48)),
        origin=Origin(xyz=(0.0, 0.023, 0.30)),
        material=machined_steel,
        name="column_rack",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.62),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, -0.07, 0.105)),
    )

    support = model.part("table_support")
    support.visual(
        Box((0.024, 0.05, 0.05)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material=cast_iron,
        name="support_collar",
    )
    support.visual(
        Box((0.024, 0.05, 0.05)),
        origin=Origin(xyz=(-0.044, 0.0, 0.0)),
        material=cast_iron,
        name="support_collar_left",
    )
    support.visual(
        Box((0.03, 0.14, 0.05)),
        origin=Origin(xyz=(0.045, 0.095, 0.0)),
        material=cast_iron,
        name="support_arm",
    )
    support.visual(
        Box((0.03, 0.14, 0.05)),
        origin=Origin(xyz=(-0.045, 0.095, 0.0)),
        material=cast_iron,
        name="support_arm_left",
    )
    support.visual(
        Box((0.24, 0.18, 0.018)),
        origin=Origin(xyz=(0.0, 0.17, 0.034)),
        material=machined_steel,
        name="support_deck",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.24, 0.23, 0.07)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.11, 0.02)),
    )

    model.articulation(
        "column_to_table_support",
        ArticulationType.FIXED,
        parent=column,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    head = model.part("head")
    head_body = mesh_from_geometry(
        section_loft(
            [
                xz_section(0.11, 0.11, 0.022, 0.03, z_offset=0.03),
                xz_section(0.15, 0.16, 0.030, 0.10, z_offset=0.03),
                xz_section(0.12, 0.13, 0.024, 0.13, z_offset=0.03),
            ]
        ),
        "head_body",
    )
    quill_sleeve = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.032, -0.07), (0.034, -0.01), (0.034, 0.06)],
            inner_profile=[(0.024, -0.07), (0.024, 0.06)],
            segments=48,
        ),
        "quill_sleeve",
    )
    head.visual(
        Box((0.024, 0.06, 0.12)),
        origin=Origin(xyz=(0.044, 0.0, 0.03)),
        material=cast_iron,
        name="head_collar",
    )
    head.visual(
        Box((0.024, 0.06, 0.12)),
        origin=Origin(xyz=(-0.044, 0.0, 0.03)),
        material=cast_iron,
        name="head_collar_left",
    )
    head.visual(
        Box((0.112, 0.032, 0.12)),
        origin=Origin(xyz=(0.0, -0.046, 0.03)),
        material=cast_iron,
        name="head_collar_bridge",
    )
    head.visual(head_body, material=cast_iron, name="head_body")
    head.visual(
        Box((0.12, 0.16, 0.07)),
        origin=Origin(xyz=(0.0, 0.09, 0.14)),
        material=cast_iron,
        name="belt_cover",
    )
    head.visual(
        Box((0.018, 0.10, 0.055)),
        origin=Origin(xyz=(0.04, 0.165, -0.02)),
        material=cast_iron,
        name="nose_bridge",
    )
    head.visual(
        Box((0.018, 0.10, 0.055)),
        origin=Origin(xyz=(-0.04, 0.165, -0.02)),
        material=cast_iron,
        name="nose_bridge_left",
    )
    head.visual(
        quill_sleeve,
        origin=Origin(xyz=(0.0, 0.165, -0.07)),
        material=machined_steel,
        name="quill_sleeve",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.16, 0.24, 0.26)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.09, 0.04)),
    )

    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
    )

    lower_table = model.part("lower_table")
    saddle_body = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.22, 0.18, 0.010), 0.036),
        "lower_saddle",
    )
    lower_table.visual(
        saddle_body,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=cast_iron,
        name="saddle_body",
    )
    lower_table.visual(
        Box((0.03, 0.012, 0.02)),
        origin=Origin(xyz=(0.0, 0.094, 0.018)),
        material=cast_iron,
        name="y_mount_block",
    )
    lower_table.visual(
        Cylinder(radius=0.008, length=0.05),
        origin=Origin(xyz=(0.0, 0.115, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="y_feed_shaft",
    )
    lower_table.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.143, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_handle,
        name="y_handwheel",
    )
    lower_table.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(xyz=(0.018, 0.143, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_handle,
        name="y_hand_grip",
    )
    lower_table.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.05)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "support_to_lower_table",
        ArticulationType.PRISMATIC,
        parent=support,
        child=lower_table,
        origin=Origin(xyz=(0.0, 0.17, 0.043)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=-0.05,
            upper=0.05,
        ),
    )

    upper_table = model.part("upper_table")
    table_outer = rounded_rect_profile(0.24, 0.16, 0.012)
    slot_profile = rounded_rect_profile(0.018, 0.11, 0.004)
    table_top = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            table_outer,
            [
                shift_profile(slot_profile, dx=-0.07),
                shift_profile(slot_profile, dx=0.0),
                shift_profile(slot_profile, dx=0.07),
            ],
            0.028,
        ),
        "upper_table_top",
    )
    upper_table.visual(
        table_top,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=machined_steel,
        name="table_top",
    )
    upper_table.visual(
        Box((0.012, 0.03, 0.02)),
        origin=Origin(xyz=(0.118, 0.0, 0.014)),
        material=machined_steel,
        name="x_mount_block",
    )
    upper_table.visual(
        Cylinder(radius=0.008, length=0.05),
        origin=Origin(xyz=(0.145, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="x_feed_shaft",
    )
    upper_table.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.173, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_handle,
        name="x_handwheel",
    )
    upper_table.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(xyz=(0.173, 0.022, 0.024)),
        material=dark_handle,
        name="x_hand_grip",
    )
    upper_table.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.04)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    model.articulation(
        "lower_to_upper_table",
        ArticulationType.PRISMATIC,
        parent=lower_table,
        child=upper_table,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=-0.07,
            upper=0.07,
        ),
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.023, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.0425)),
        material=machined_steel,
        name="quill_body",
    )
    quill.visual(
        Cylinder(radius=0.03, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=machined_steel,
        name="quill_stop_collar",
    )
    quill.visual(
        Cylinder(radius=0.017, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=blackened_tool,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.010, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=blackened_tool,
        name="chuck_nose",
    )
    quill.visual(
        Cylinder(radius=0.003, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=blackened_tool,
        name="drill_bit",
    )
    quill.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.12),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
    )

    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, 0.165, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.10,
            lower=0.0,
            upper=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    support = object_model.get_part("table_support")
    head = object_model.get_part("head")
    lower_table = object_model.get_part("lower_table")
    upper_table = object_model.get_part("upper_table")
    quill = object_model.get_part("quill")

    y_slide = object_model.get_articulation("support_to_lower_table")
    x_slide = object_model.get_articulation("lower_to_upper_table")
    quill_slide = object_model.get_articulation("head_to_quill")

    ctx.expect_contact(
        column,
        base,
        elem_a="column_shaft",
        elem_b="column_socket",
        name="column seats on the base socket",
    )
    ctx.expect_contact(
        head,
        column,
        elem_a="head_collar",
        elem_b="column_shaft",
        name="head collar clamps the column",
    )
    ctx.expect_contact(
        support,
        column,
        elem_a="support_collar",
        elem_b="column_shaft",
        name="table support collar clamps the column",
    )

    ctx.expect_gap(
        lower_table,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="saddle_body",
        negative_elem="support_deck",
        name="lower table sits on the support deck",
    )
    ctx.expect_gap(
        upper_table,
        lower_table,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="table_top",
        negative_elem="saddle_body",
        name="upper table rides on the lower saddle",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="xy",
        elem_a="quill_body",
        elem_b="quill_sleeve",
        min_overlap=0.040,
        name="quill stays centered in the head sleeve",
    )
    ctx.expect_contact(
        quill,
        head,
        elem_a="quill_stop_collar",
        elem_b="quill_sleeve",
        name="quill is retained by the head sleeve at the top stop collar",
    )
    ctx.expect_gap(
        quill,
        upper_table,
        axis="z",
        positive_elem="drill_bit",
        negative_elem="table_top",
        min_gap=0.05,
        name="drill bit clears the table at rest",
    )

    lower_rest = ctx.part_world_position(lower_table)
    with ctx.pose({y_slide: 0.05}):
        lower_shifted = ctx.part_world_position(lower_table)
        ctx.expect_within(
            lower_table,
            support,
            axes="x",
            inner_elem="saddle_body",
            outer_elem="support_deck",
            margin=0.0,
            name="lower table remains laterally supported",
        )
        ctx.expect_overlap(
            lower_table,
            support,
            axes="y",
            elem_a="saddle_body",
            elem_b="support_deck",
            min_overlap=0.12,
            name="lower table retains Y-axis engagement",
        )
    ctx.check(
        "lower table moves in positive Y",
        lower_rest is not None
        and lower_shifted is not None
        and lower_shifted[1] > lower_rest[1] + 0.045,
        details=f"rest={lower_rest}, shifted={lower_shifted}",
    )

    upper_rest = ctx.part_world_position(upper_table)
    with ctx.pose({x_slide: 0.07}):
        upper_shifted = ctx.part_world_position(upper_table)
        ctx.expect_within(
            upper_table,
            lower_table,
            axes="y",
            inner_elem="table_top",
            outer_elem="saddle_body",
            margin=0.0,
            name="upper table stays captured in Y on the saddle",
        )
        ctx.expect_overlap(
            upper_table,
            lower_table,
            axes="x",
            elem_a="table_top",
            elem_b="saddle_body",
            min_overlap=0.12,
            name="upper table retains X-axis engagement",
        )
    ctx.check(
        "upper table moves in positive X",
        upper_rest is not None
        and upper_shifted is not None
        and upper_shifted[0] > upper_rest[0] + 0.065,
        details=f"rest={upper_rest}, shifted={upper_shifted}",
    )

    quill_rest = ctx.part_world_position(quill)
    with ctx.pose({quill_slide: 0.08}):
        quill_dropped = ctx.part_world_position(quill)
        ctx.expect_gap(
            quill,
            upper_table,
            axis="z",
            positive_elem="drill_bit",
            negative_elem="table_top",
            min_gap=0.01,
            name="drill bit still clears the table at full quill travel",
        )
    ctx.check(
        "quill drops downward",
        quill_rest is not None
        and quill_dropped is not None
        and quill_dropped[2] < quill_rest[2] - 0.07,
        details=f"rest={quill_rest}, dropped={quill_dropped}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
