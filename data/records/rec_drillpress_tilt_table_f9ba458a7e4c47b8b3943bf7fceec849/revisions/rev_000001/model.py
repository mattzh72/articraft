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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sensitive_bench_drill")

    machine_green = model.material("machine_green", rgba=(0.24, 0.38, 0.30, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.24, 0.26, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.86, 0.88, 0.90, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.300, 0.190, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=machine_green,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=machine_green,
        name="column_pad",
    )
    base.visual(
        Box((0.060, 0.090, 0.020)),
        origin=Origin(xyz=(-0.015, 0.0, 0.038)),
        material=machine_green,
        name="rear_web",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.300, 0.190, 0.060)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.016, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=polished_steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_iron,
        name="column_foot",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.470),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.028, 0.012, 0.078)),
        origin=Origin(xyz=(-0.003, -0.024, 0.0)),
        material=machine_green,
        name="left_jaw",
    )
    table_carriage.visual(
        Box((0.028, 0.012, 0.078)),
        origin=Origin(xyz=(-0.003, 0.024, 0.0)),
        material=machine_green,
        name="right_jaw",
    )
    table_carriage.visual(
        Box((0.018, 0.072, 0.030)),
        origin=Origin(xyz=(-0.023, 0.0, 0.0)),
        material=machine_green,
        name="rear_bridge",
    )
    table_carriage.visual(
        Box((0.024, 0.012, 0.024)),
        origin=Origin(xyz=(0.009, 0.024, -0.010)),
        material=machine_green,
        name="side_neck",
    )
    table_carriage.visual(
        Box((0.080, 0.012, 0.016)),
        origin=Origin(xyz=(0.060, 0.024, -0.018)),
        material=machine_green,
        name="side_spine",
    )
    table_carriage.visual(
        Box((0.020, 0.052, 0.010)),
        origin=Origin(xyz=(0.100, 0.0, 0.007)),
        material=machine_green,
        name="upper_yoke_bridge",
    )
    table_carriage.visual(
        Box((0.012, 0.006, 0.032)),
        origin=Origin(xyz=(0.100, -0.014, -0.010)),
        material=machine_green,
        name="left_yoke_cheek",
    )
    table_carriage.visual(
        Box((0.012, 0.006, 0.032)),
        origin=Origin(xyz=(0.100, 0.014, -0.010)),
        material=machine_green,
        name="right_yoke_cheek",
    )
    table_carriage.visual(
        Box((0.020, 0.032, 0.010)),
        origin=Origin(xyz=(0.100, 0.0, -0.026)),
        material=machine_green,
        name="lower_yoke_bridge",
    )
    table_carriage.visual(
        Box((0.014, 0.008, 0.028)),
        origin=Origin(xyz=(0.095, 0.022, 0.004)),
        material=machine_green,
        name="side_gusset",
    )
    table_carriage.inertial = Inertial.from_geometry(
        Box((0.150, 0.080, 0.090)),
        mass=2.0,
        origin=Origin(xyz=(0.040, 0.0, -0.008)),
    )

    model.articulation(
        "column_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=0.0,
            upper=0.160,
        ),
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="trunnion",
    )
    table.visual(
        Box((0.092, 0.018, 0.012)),
        origin=Origin(xyz=(0.046, 0.0, 0.008)),
        material=dark_iron,
        name="table_web",
    )
    table.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.070, 0.0, 0.017)),
        material=dark_iron,
        name="table_pedestal",
    )
    table.visual(
        Cylinder(radius=0.055, length=0.012),
        origin=Origin(xyz=(0.092, 0.0, 0.033)),
        material=dark_iron,
        name="table_disk",
    )
    table.inertial = Inertial.from_geometry(
        Box((0.150, 0.110, 0.045)),
        mass=1.2,
        origin=Origin(xyz=(0.080, 0.0, 0.022)),
    )

    model.articulation(
        "table_carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.100, 0.0, -0.011)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )

    head = model.part("head")
    head.visual(
        Box((0.076, 0.012, 0.116)),
        origin=Origin(xyz=(0.018, -0.022, 0.058)),
        material=machine_green,
        name="left_saddle",
    )
    head.visual(
        Box((0.076, 0.012, 0.116)),
        origin=Origin(xyz=(0.018, 0.022, 0.058)),
        material=machine_green,
        name="right_saddle",
    )
    head.visual(
        Box((0.020, 0.060, 0.030)),
        origin=Origin(xyz=(-0.022, 0.0, 0.096)),
        material=machine_green,
        name="rear_bridge",
    )
    head.visual(
        Box((0.018, 0.044, 0.024)),
        origin=Origin(xyz=(-0.020, 0.0, 0.022)),
        material=machine_green,
        name="lower_bridge",
    )
    head.visual(
        Box((0.146, 0.094, 0.082)),
        origin=Origin(xyz=(0.097, 0.0, 0.058)),
        material=machine_green,
        name="head_body",
    )
    head.visual(
        Box((0.090, 0.072, 0.040)),
        origin=Origin(xyz=(0.082, 0.0, 0.118)),
        material=machine_green,
        name="top_cap",
    )
    head.visual(
        Box((0.042, 0.054, 0.050)),
        origin=Origin(xyz=(0.145, 0.0, 0.029)),
        material=machine_green,
        name="spindle_nose",
    )
    head.visual(
        Box((0.024, 0.016, 0.030)),
        origin=Origin(xyz=(0.072, 0.051, 0.056)),
        material=machine_green,
        name="feed_pad",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.040, -0.060, 0.064), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="return_spring_canister",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.190, 0.110, 0.140)),
        mass=7.0,
        origin=Origin(xyz=(0.070, 0.0, 0.070)),
    )

    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.332)),
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=polished_steel,
        name="quill_collar",
    )
    quill.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=polished_steel,
        name="quill_tube",
    )
    quill.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=dark_iron,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.007, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.121)),
        material=steel,
        name="spindle_nose",
    )
    quill.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.142),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.071)),
    )

    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.140, 0.0, 0.004)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.12,
            lower=0.0,
            upper=0.060,
        ),
    )

    feed_lever = model.part("feed_lever")
    feed_lever.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="lever_hub",
    )
    feed_lever.visual(
        Cylinder(radius=0.004, length=0.090),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lever_arm",
    )
    feed_lever.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, 0.092, 0.0)),
        material=handle_black,
        name="lever_knob",
    )
    feed_lever.inertial = Inertial.from_geometry(
        Box((0.020, 0.110, 0.020)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
    )

    model.articulation(
        "head_to_feed_lever",
        ArticulationType.REVOLUTE,
        parent=head,
        child=feed_lever,
        origin=Origin(xyz=(0.072, 0.067, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.0,
            upper=1.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")
    feed_lever = object_model.get_part("feed_lever")

    base_to_column = object_model.get_articulation("base_to_column")
    table_slide = object_model.get_articulation("column_to_table_carriage")
    table_tilt = object_model.get_articulation("table_carriage_to_table")
    head_to_quill = object_model.get_articulation("head_to_quill")
    lever_joint = object_model.get_articulation("head_to_feed_lever")

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

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}

    for part_name in ("base", "column", "table_carriage", "table", "head", "quill", "feed_lever"):
        ctx.check(f"part {part_name} exists", part_name in part_names, details=str(sorted(part_names)))

    for joint_name in (
        "base_to_column",
        "column_to_table_carriage",
        "table_carriage_to_table",
        "column_to_head",
        "head_to_quill",
        "head_to_feed_lever",
    ):
        ctx.check(f"joint {joint_name} exists", joint_name in joint_names, details=str(sorted(joint_names)))

    ctx.expect_contact(base, column, contact_tol=0.0005, name="column seats on base pad")
    ctx.expect_contact(head, column, contact_tol=0.0005, name="head saddle bears on column")
    ctx.expect_contact(table_carriage, column, contact_tol=0.0005, name="table carriage bears on column")
    ctx.expect_contact(table, table_carriage, contact_tol=0.0005, name="table trunnion is supported by tilt bracket")
    ctx.expect_contact(quill, head, contact_tol=0.0005, name="quill seats against head nose at rest")
    ctx.expect_contact(feed_lever, head, contact_tol=0.0005, name="feed lever hub seats on feed pad")

    ctx.expect_origin_distance(
        table_carriage,
        column,
        axes="xy",
        max_dist=0.0015,
        name="table carriage stays centered on the column",
    )

    table_slide_upper = table_slide.motion_limits.upper if table_slide.motion_limits is not None else None
    quill_feed_upper = head_to_quill.motion_limits.upper if head_to_quill.motion_limits is not None else None

    rest_table_carriage_pos = ctx.part_world_position(table_carriage)
    with ctx.pose({table_slide: table_slide_upper}):
        raised_table_carriage_pos = ctx.part_world_position(table_carriage)
        ctx.expect_origin_distance(
            table_carriage,
            column,
            axes="xy",
            max_dist=0.0015,
            name="raised table carriage remains centered on the column",
        )

    ctx.check(
        "table carriage height adjustment moves upward",
        rest_table_carriage_pos is not None
        and raised_table_carriage_pos is not None
        and raised_table_carriage_pos[2] > rest_table_carriage_pos[2] + 0.120,
        details=f"rest={rest_table_carriage_pos}, raised={raised_table_carriage_pos}",
    )

    rest_quill_pos = ctx.part_world_position(quill)
    with ctx.pose({head_to_quill: quill_feed_upper}):
        fed_quill_pos = ctx.part_world_position(quill)

    ctx.check(
        "quill feed advances downward",
        rest_quill_pos is not None and fed_quill_pos is not None and fed_quill_pos[2] < rest_quill_pos[2] - 0.040,
        details=f"rest={rest_quill_pos}, fed={fed_quill_pos}",
    )

    rest_table_aabb = ctx.part_world_aabb(table)
    with ctx.pose({table_tilt: math.radians(30.0)}):
        tilted_table_aabb = ctx.part_world_aabb(table)

    ctx.check(
        "table tilt changes the table envelope",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and (tilted_table_aabb[1][2] - tilted_table_aabb[0][2])
        > (rest_table_aabb[1][2] - rest_table_aabb[0][2]) + 0.012,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_lever_aabb = ctx.part_world_aabb(feed_lever)
    with ctx.pose({lever_joint: 0.9}):
        swung_lever_aabb = ctx.part_world_aabb(feed_lever)

    ctx.check(
        "feed lever swings through a visible arc",
        rest_lever_aabb is not None
        and swung_lever_aabb is not None
        and abs(swung_lever_aabb[1][2] - rest_lever_aabb[1][2]) > 0.030,
        details=f"rest={rest_lever_aabb}, swung={swung_lever_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
