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
    model = ArticulatedObject(name="gang_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.30, 0.32, 0.34, 1.0))
    machine_green = model.material("machine_green", rgba=(0.18, 0.34, 0.26, 1.0))
    table_grey = model.material("table_grey", rgba=(0.38, 0.40, 0.42, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.20, 0.22, 1.0))
    bakelite = model.material("bakelite", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.74, 0.50, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base_plinth",
    )
    frame.visual(
        Box((0.44, 0.30, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=cast_iron,
        name="pedestal",
    )
    frame.visual(
        Box((0.28, 0.22, 1.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
        material=machine_green,
        name="column",
    )
    frame.visual(
        Box((0.04, 0.014, 0.62)),
        origin=Origin(xyz=(0.0, 0.117, 1.05)),
        material=dark_steel,
        name="feed_rack",
    )
    frame.visual(
        Box((0.34, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.57)),
        material=cast_iron,
        name="column_cap",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.74, 0.50, 1.62)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
    )

    head = model.part("head")
    head.visual(
        Box((0.84, 0.284, 0.36)),
        origin=Origin(xyz=(0.0, 0.267, 0.18)),
        material=machine_green,
        name="front_carriage",
    )
    head.visual(
        Box((0.11, 0.36, 0.36)),
        origin=Origin(xyz=(-0.195, 0.15, 0.18)),
        material=machine_green,
        name="left_slide_cheek",
    )
    head.visual(
        Box((0.11, 0.36, 0.36)),
        origin=Origin(xyz=(0.195, 0.15, 0.18)),
        material=machine_green,
        name="right_slide_cheek",
    )
    head.visual(
        Box((0.84, 0.24, 0.14)),
        origin=Origin(xyz=(0.0, 0.23, 0.43)),
        material=machine_green,
        name="drive_cover",
    )
    head.visual(
        Cylinder(radius=0.045, length=0.036),
        origin=Origin(xyz=(0.438, 0.18, 0.23), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="feed_hub",
    )
    for handle_index, handle_angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        direction_y = math.sin(handle_angle)
        direction_z = math.cos(handle_angle)
        head.visual(
            Cylinder(radius=0.009, length=0.18),
            origin=Origin(
                xyz=(0.456, 0.18 + 0.09 * direction_y, 0.23 + 0.09 * direction_z),
                rpy=(-handle_angle, 0.0, 0.0),
            ),
            material=steel,
            name=f"feed_handle_{handle_index}",
        )
        head.visual(
            Sphere(radius=0.018),
            origin=Origin(
                xyz=(0.456, 0.18 + 0.18 * direction_y, 0.23 + 0.18 * direction_z),
            ),
            material=bakelite,
            name=f"feed_knob_{handle_index}",
        )
    head.inertial = Inertial.from_geometry(
        Box((0.90, 0.42, 0.52)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.16, 0.26)),
    )

    spindle_offsets = (-0.24, 0.0, 0.24)
    spindle_parts = []
    for spindle_name, spindle_x in zip(("left", "center", "right"), spindle_offsets):
        spindle = model.part(f"spindle_{spindle_name}")
        spindle.visual(
            Cylinder(radius=0.020, length=0.03),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=steel,
            name="spindle_collar",
        )
        spindle.visual(
            Cylinder(radius=0.014, length=0.11),
            origin=Origin(xyz=(0.0, 0.0, -0.085)),
            material=steel,
            name="spindle_shaft",
        )
        spindle.visual(
            Cylinder(radius=0.028, length=0.06),
            origin=Origin(xyz=(0.0, 0.0, -0.17)),
            material=dark_steel,
            name="chuck_body",
        )
        spindle.visual(
            Cylinder(radius=0.020, length=0.04),
            origin=Origin(xyz=(0.0, 0.0, -0.22)),
            material=dark_steel,
            name="chuck_nose",
        )
        spindle.visual(
            Cylinder(radius=0.0045, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, -0.2525)),
            material=dark_steel,
            name="tool_stub",
        )
        spindle.inertial = Inertial.from_geometry(
            Box((0.07, 0.07, 0.29)),
            mass=2.0,
            origin=Origin(xyz=(0.0, 0.0, -0.145)),
        )
        model.articulation(
            f"head_to_spindle_{spindle_name}",
            ArticulationType.CONTINUOUS,
            parent=head,
            child=spindle,
            origin=Origin(xyz=(spindle_x, 0.27, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=30.0),
        )
        spindle_parts.append(spindle)

    table_bracket = model.part("table_bracket")
    table_bracket.visual(
        Box((0.30, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, -0.18, -0.08)),
        material=cast_iron,
        name="support_arm",
    )
    table_bracket.visual(
        Box((0.10, 0.20, 0.22)),
        origin=Origin(xyz=(-0.19, -0.19, -0.03)),
        material=cast_iron,
        name="left_column_clamp",
    )
    table_bracket.visual(
        Box((0.10, 0.20, 0.22)),
        origin=Origin(xyz=(0.19, -0.19, -0.03)),
        material=cast_iron,
        name="right_column_clamp",
    )
    table_bracket.visual(
        Box((0.03, 0.14, 0.12)),
        origin=Origin(xyz=(-0.125, -0.07, 0.0)),
        material=cast_iron,
        name="left_ear",
    )
    table_bracket.visual(
        Box((0.03, 0.14, 0.12)),
        origin=Origin(xyz=(0.125, -0.07, 0.0)),
        material=cast_iron,
        name="right_ear",
    )
    table_bracket.visual(
        Cylinder(radius=0.028, length=0.29),
        origin=Origin(xyz=(0.0, -0.12, -0.04), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="rear_tie_bar",
    )
    table_bracket.inertial = Inertial.from_geometry(
        Box((0.42, 0.33, 0.24)),
        mass=34.0,
        origin=Origin(xyz=(0.0, -0.15, -0.03)),
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.022, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=table_grey,
        name="trunnion_barrel",
    )
    table.visual(
        Cylinder(radius=0.05, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=table_grey,
        name="support_post",
    )
    table.visual(
        Cylinder(radius=0.25, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=table_grey,
        name="table_top",
    )
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.25, length=0.16),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    model.articulation(
        "frame_to_head_feed",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.15,
            lower=0.0,
            upper=0.10,
        ),
    )
    model.articulation(
        "frame_to_table_bracket",
        ArticulationType.FIXED,
        parent=frame,
        child=table_bracket,
        origin=Origin(xyz=(0.0, 0.38, 0.52)),
    )
    model.articulation(
        "table_bracket_to_table",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=table,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.7,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    table_bracket = object_model.get_part("table_bracket")
    table = object_model.get_part("table")
    spindle_left = object_model.get_part("spindle_left")
    spindle_center = object_model.get_part("spindle_center")
    spindle_right = object_model.get_part("spindle_right")

    head_feed = object_model.get_articulation("frame_to_head_feed")
    table_tilt = object_model.get_articulation("table_bracket_to_table")
    left_spin = object_model.get_articulation("head_to_spindle_left")
    center_spin = object_model.get_articulation("head_to_spindle_center")
    right_spin = object_model.get_articulation("head_to_spindle_right")

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

    ctx.expect_contact(head, frame, name="head carriage bears on the column guides")
    ctx.expect_contact(table_bracket, frame, name="table bracket is clamped to the column")
    ctx.expect_contact(table, table_bracket, name="table trunnion sits in the tilt bracket")
    for spindle in (spindle_left, spindle_center, spindle_right):
        ctx.expect_contact(spindle, head, name=f"{spindle.name} is carried by the gang head")

    ctx.expect_origin_gap(
        spindle_center,
        spindle_left,
        axis="x",
        min_gap=0.23,
        max_gap=0.25,
        name="left and center spindle axes are equally spaced",
    )
    ctx.expect_origin_gap(
        spindle_right,
        spindle_center,
        axis="x",
        min_gap=0.23,
        max_gap=0.25,
        name="center and right spindle axes are equally spaced",
    )

    ctx.check(
        "feed joint is a downward prismatic slide",
        head_feed.articulation_type == ArticulationType.PRISMATIC
        and head_feed.axis == (0.0, 0.0, -1.0)
        and head_feed.motion_limits is not None
        and head_feed.motion_limits.lower == 0.0
        and head_feed.motion_limits.upper is not None
        and head_feed.motion_limits.upper >= 0.08,
        details=f"type={head_feed.articulation_type}, axis={head_feed.axis}, limits={head_feed.motion_limits}",
    )
    ctx.check(
        "table tilts about a left-right axle",
        table_tilt.articulation_type == ArticulationType.REVOLUTE
        and table_tilt.axis == (1.0, 0.0, 0.0)
        and table_tilt.motion_limits is not None
        and table_tilt.motion_limits.lower is not None
        and table_tilt.motion_limits.upper is not None
        and table_tilt.motion_limits.lower < 0.0 < table_tilt.motion_limits.upper,
        details=f"type={table_tilt.articulation_type}, axis={table_tilt.axis}, limits={table_tilt.motion_limits}",
    )
    ctx.check(
        "all three spindles spin on vertical continuous axles",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 0.0, 1.0)
            for joint in (left_spin, center_spin, right_spin)
        ),
        details=(
            f"left={left_spin.articulation_type, left_spin.axis}, "
            f"center={center_spin.articulation_type, center_spin.axis}, "
            f"right={right_spin.articulation_type, right_spin.axis}"
        ),
    )

    feed_upper = head_feed.motion_limits.upper if head_feed.motion_limits is not None else None
    if feed_upper is not None:
        rest_head_pos = ctx.part_world_position(head)
        with ctx.pose({head_feed: feed_upper}):
            advanced_head_pos = ctx.part_world_position(head)
            ctx.expect_contact(
                head,
                frame,
                name="head remains supported by the column at full feed",
            )
            ctx.expect_gap(
                spindle_center,
                table,
                axis="z",
                min_gap=0.003,
                max_gap=0.03,
                positive_elem="tool_stub",
                negative_elem="table_top",
                name="center spindle clears the table at full feed",
            )
        ctx.check(
            "positive feed motion lowers the gang head",
            rest_head_pos is not None
            and advanced_head_pos is not None
            and advanced_head_pos[2] < rest_head_pos[2] - 0.08,
            details=f"rest={rest_head_pos}, advanced={advanced_head_pos}",
        )

    tilt_upper = table_tilt.motion_limits.upper if table_tilt.motion_limits is not None else None
    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    if tilt_upper is not None and rest_table_aabb is not None:
        with ctx.pose({table_tilt: tilt_upper}):
            tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
            ctx.expect_contact(
                table,
                table_bracket,
                name="table remains supported when tilted",
            )
        if tilted_table_aabb is not None:
            rest_dx = rest_table_aabb[1][0] - rest_table_aabb[0][0]
            rest_dy = rest_table_aabb[1][1] - rest_table_aabb[0][1]
            rest_dz = rest_table_aabb[1][2] - rest_table_aabb[0][2]
            tilt_dx = tilted_table_aabb[1][0] - tilted_table_aabb[0][0]
            tilt_dy = tilted_table_aabb[1][1] - tilted_table_aabb[0][1]
            tilt_dz = tilted_table_aabb[1][2] - tilted_table_aabb[0][2]
            ctx.check(
                "tilted table rotates about the x-axis",
                abs(tilt_dx - rest_dx) <= 0.01 and tilt_dy < rest_dy - 0.05 and tilt_dz > rest_dz + 0.08,
                details=(
                    f"rest_extents={(rest_dx, rest_dy, rest_dz)}, "
                    f"tilted_extents={(tilt_dx, tilt_dy, tilt_dz)}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
