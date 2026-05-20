from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


def _yz_section(
    x: float,
    *,
    depth: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(height, depth, radius, corner_segments=8)
    ]


def _build_frame_geometry() -> object:
    geom = BoxGeometry((0.78, 0.46, 0.08)).translate(0.0, 0.0, 0.04)
    geom.merge(BoxGeometry((0.16, 0.12, 1.40)).translate(0.0, -0.17, 0.78))
    geom.merge(BoxGeometry((0.84, 0.08, 0.12)).translate(0.0, -0.11, 1.48))
    geom.merge(BoxGeometry((0.32, 0.03, 0.18)).translate(0.0, -0.115, 0.80))
    return geom


def _build_head_geometry() -> object:
    body = section_loft(
        [
            _yz_section(-0.11, depth=0.18, height=0.14, radius=0.026, z_center=0.11),
            _yz_section(0.0, depth=0.22, height=0.16, radius=0.030, z_center=0.11),
            _yz_section(0.11, depth=0.18, height=0.14, radius=0.026, z_center=0.11),
        ]
    )
    body.merge(BoxGeometry((0.14, 0.16, 0.04)).translate(0.0, -0.01, 0.16))
    body.merge(BoxGeometry((0.22, 0.20, 0.02)).translate(0.0, 0.0, 0.19))
    body.merge(CylinderGeometry(0.055, 0.03, radial_segments=28).translate(0.0, 0.0, 0.10))
    body.merge(CylinderGeometry(0.042, 0.09, radial_segments=28).translate(0.0, 0.0, 0.045))
    return body


def _build_spindle_geometry() -> object:
    geom = CylinderGeometry(0.028, 0.018, radial_segments=28).translate(0.0, 0.0, -0.009)
    geom.merge(CylinderGeometry(0.033, 0.05, radial_segments=32).translate(0.0, 0.0, -0.043))
    geom.merge(ConeGeometry(0.020, 0.04, radial_segments=28).translate(0.0, 0.0, -0.088))
    geom.merge(CylinderGeometry(0.0055, 0.09, radial_segments=18).translate(0.0, 0.0, -0.153))
    geom.merge(ConeGeometry(0.0055, 0.02, radial_segments=18).translate(0.0, 0.0, -0.208))
    geom.merge(
        CylinderGeometry(0.009, 0.028, radial_segments=18)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.038, -0.050)
    )
    return geom


def _build_feed_lever_geometry() -> object:
    geom = CylinderGeometry(0.015, 0.02, radial_segments=24).rotate_y(math.pi / 2.0)
    arm = wire_from_points(
        [(0.0, 0.0, -0.004), (0.0, 0.085, -0.018), (0.0, 0.130, -0.040)],
        radius=0.005,
        radial_segments=18,
        corner_mode="fillet",
        corner_radius=0.018,
        corner_segments=8,
        cap_ends=True,
    )
    geom.merge(arm)
    geom.merge(SphereGeometry(0.014, width_segments=18, height_segments=12).translate(0.0, 0.130, -0.040))
    return geom


def _build_tilt_bracket_geometry() -> object:
    geom = BoxGeometry((0.07, 0.14, 0.16)).translate(-0.24, -0.06, -0.01)
    geom.merge(BoxGeometry((0.07, 0.14, 0.16)).translate(0.24, -0.06, -0.01))
    geom.merge(BoxGeometry((0.54, 0.03, 0.06)).translate(0.0, -0.135, -0.075))
    geom.merge(BoxGeometry((0.32, 0.04, 0.08)).translate(0.0, -0.10, -0.06))
    geom.merge(BoxGeometry((0.10, 0.04, 0.05)).translate(-0.24, -0.01, 0.025))
    geom.merge(BoxGeometry((0.10, 0.04, 0.05)).translate(0.24, -0.01, 0.025))
    return geom


def _build_table_geometry() -> object:
    outer = rounded_rect_profile(0.88, 0.32, 0.024, corner_segments=8)
    slots = [
        [(px + x_off, py) for px, py in rounded_rect_profile(0.15, 0.026, 0.013, corner_segments=6)]
        for x_off in (-0.24, 0.0, 0.24)
    ]
    geom = ExtrudeWithHolesGeometry(outer, slots, 0.045, center=True)
    geom.translate(0.0, 0.19, -0.004)
    geom.merge(CylinderGeometry(0.020, 0.62, radial_segments=24).rotate_y(math.pi / 2.0))
    geom.merge(BoxGeometry((0.52, 0.05, 0.05)).translate(0.0, 0.085, -0.026))
    geom.merge(BoxGeometry((0.74, 0.05, 0.04)).translate(0.0, 0.150, -0.018))
    geom.merge(BoxGeometry((0.86, 0.02, 0.02)).translate(0.0, 0.340, 0.006))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gang_drill_press")

    machine_green = model.material("machine_green", rgba=(0.25, 0.42, 0.31, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.44, 0.46, 0.48, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.76, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.12, 0.12, 0.13, 1.0))

    frame_geom = _build_frame_geometry()
    head_geom = _build_head_geometry()
    spindle_geom = _build_spindle_geometry()
    lever_geom = _build_feed_lever_geometry()
    bracket_geom = _build_tilt_bracket_geometry()
    table_geom = _build_table_geometry()

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(frame_geom, "frame_shell"),
        material=machine_green,
        name="frame_shell",
    )
    frame.visual(
        Box((0.16, 0.06, 0.20)),
        origin=Origin(xyz=(-0.19, -0.04, 1.32)),
        material=machine_green,
        name="left_head_mount_pad",
    )
    frame.visual(
        Box((0.16, 0.06, 0.20)),
        origin=Origin(xyz=(0.19, -0.04, 1.32)),
        material=machine_green,
        name="right_head_mount_pad",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.46, 1.54)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
    )

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        mesh_from_geometry(bracket_geom, "tilt_bracket_shell"),
        material=machine_green,
        name="tilt_bracket_shell",
    )
    tilt_bracket.inertial = Inertial.from_geometry(
        Box((0.56, 0.18, 0.16)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -0.04, -0.02)),
    )

    tilt_table = model.part("tilt_table")
    tilt_table.visual(
        mesh_from_geometry(table_geom, "tilt_table_shell"),
        material=cast_iron,
        name="tilt_table_shell",
    )
    tilt_table.inertial = Inertial.from_geometry(
        Box((0.88, 0.32, 0.09)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.09, 0.04)),
    )

    left_head = model.part("left_head")
    left_head.visual(
        mesh_from_geometry(head_geom.copy(), "left_head_shell"),
        material=machine_green,
        name="head_shell",
    )
    left_head.visual(
        Box((0.16, 0.06, 0.16)),
        origin=Origin(xyz=(0.0, -0.08, 0.09)),
        material=machine_green,
        name="mount_shoe",
    )
    left_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.22, 0.20)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    right_head = model.part("right_head")
    right_head.visual(
        mesh_from_geometry(head_geom.copy(), "right_head_shell"),
        material=machine_green,
        name="head_shell",
    )
    right_head.visual(
        Box((0.16, 0.06, 0.16)),
        origin=Origin(xyz=(0.0, -0.08, 0.09)),
        material=machine_green,
        name="mount_shoe",
    )
    right_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.22, 0.20)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    left_spindle = model.part("left_spindle")
    left_spindle.visual(
        mesh_from_geometry(spindle_geom.copy(), "left_spindle_shell"),
        material=steel,
        name="spindle_shell",
    )
    left_spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.22),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
    )

    right_spindle = model.part("right_spindle")
    right_spindle.visual(
        mesh_from_geometry(spindle_geom.copy(), "right_spindle_shell"),
        material=steel,
        name="spindle_shell",
    )
    right_spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.22),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
    )

    left_feed_lever = model.part("left_feed_lever")
    left_feed_lever.visual(
        mesh_from_geometry(lever_geom.copy(), "left_feed_lever_shell"),
        material=dark_handle,
        name="feed_lever_shell",
    )
    left_feed_lever.inertial = Inertial.from_geometry(
        Box((0.04, 0.16, 0.08)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.07, -0.02)),
    )

    right_feed_lever = model.part("right_feed_lever")
    right_feed_lever.visual(
        mesh_from_geometry(lever_geom.copy(), "right_feed_lever_shell"),
        material=dark_handle,
        name="feed_lever_shell",
    )
    right_feed_lever.inertial = Inertial.from_geometry(
        Box((0.04, 0.16, 0.08)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.07, -0.02)),
    )

    model.articulation(
        "frame_to_tilt_bracket",
        ArticulationType.FIXED,
        parent=frame,
        child=tilt_bracket,
        origin=Origin(xyz=(0.0, 0.05, 0.82)),
    )
    model.articulation(
        "tilt_bracket_to_tilt_table",
        ArticulationType.REVOLUTE,
        parent=tilt_bracket,
        child=tilt_table,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=math.radians(-25.0),
            upper=math.radians(25.0),
        ),
    )
    model.articulation(
        "frame_to_left_head",
        ArticulationType.FIXED,
        parent=frame,
        child=left_head,
        origin=Origin(xyz=(-0.19, 0.10, 1.22)),
    )
    model.articulation(
        "frame_to_right_head",
        ArticulationType.FIXED,
        parent=frame,
        child=right_head,
        origin=Origin(xyz=(0.19, 0.10, 1.22)),
    )
    model.articulation(
        "left_head_to_spindle",
        ArticulationType.REVOLUTE,
        parent=left_head,
        child=left_spindle,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=12.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "right_head_to_spindle",
        ArticulationType.REVOLUTE,
        parent=right_head,
        child=right_spindle,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=12.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "left_head_to_feed_lever",
        ArticulationType.REVOLUTE,
        parent=left_head,
        child=left_feed_lever,
        origin=Origin(xyz=(-0.12, 0.0, 0.13)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=math.radians(-55.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "right_head_to_feed_lever",
        ArticulationType.REVOLUTE,
        parent=right_head,
        child=right_feed_lever,
        origin=Origin(xyz=(0.12, 0.0, 0.13)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=math.radians(-55.0),
            upper=math.radians(55.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tilt_bracket = object_model.get_part("tilt_bracket")
    tilt_table = object_model.get_part("tilt_table")
    left_head = object_model.get_part("left_head")
    right_head = object_model.get_part("right_head")
    left_spindle = object_model.get_part("left_spindle")
    right_spindle = object_model.get_part("right_spindle")
    left_feed_lever = object_model.get_part("left_feed_lever")
    right_feed_lever = object_model.get_part("right_feed_lever")

    table_tilt = object_model.get_articulation("tilt_bracket_to_tilt_table")
    left_chuck = object_model.get_articulation("left_head_to_spindle")
    right_chuck = object_model.get_articulation("right_head_to_spindle")
    left_feed_joint = object_model.get_articulation("left_head_to_feed_lever")
    right_feed_joint = object_model.get_articulation("right_head_to_feed_lever")

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
    ctx.allow_overlap(
        tilt_bracket,
        tilt_table,
        reason="The tilt-table hinge axle passes through the trunnion bracket ears.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(tilt_bracket, frame)
    ctx.expect_contact(tilt_table, tilt_bracket)
    ctx.expect_contact(left_head, frame)
    ctx.expect_contact(right_head, frame)
    ctx.expect_contact(left_spindle, left_head)
    ctx.expect_contact(right_spindle, right_head)
    ctx.expect_contact(left_feed_lever, left_head)
    ctx.expect_contact(right_feed_lever, right_head)

    ctx.expect_gap(left_spindle, tilt_table, axis="z", min_gap=0.08, max_gap=0.17)
    ctx.expect_gap(right_spindle, tilt_table, axis="z", min_gap=0.08, max_gap=0.17)
    ctx.expect_within(left_spindle, tilt_table, axes="xy", margin=0.0)
    ctx.expect_within(right_spindle, tilt_table, axes="xy", margin=0.0)
    ctx.expect_origin_distance(left_spindle, right_spindle, axes="x", min_dist=0.34, max_dist=0.42)

    ctx.check(
        "left_spindle_axis_is_vertical",
        tuple(round(v, 3) for v in left_chuck.axis) == (0.0, 0.0, 1.0),
        f"Unexpected left spindle axis: {left_chuck.axis}",
    )
    ctx.check(
        "right_spindle_axis_is_vertical",
        tuple(round(v, 3) for v in right_chuck.axis) == (0.0, 0.0, 1.0),
        f"Unexpected right spindle axis: {right_chuck.axis}",
    )
    ctx.check(
        "feed_lever_axes_are_horizontal",
        tuple(round(v, 3) for v in left_feed_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 3) for v in right_feed_joint.axis) == (1.0, 0.0, 0.0),
        f"Feed lever axes are {left_feed_joint.axis} and {right_feed_joint.axis}",
    )
    ctx.check(
        "table_tilt_axis_is_horizontal",
        tuple(round(v, 3) for v in table_tilt.axis) == (1.0, 0.0, 0.0),
        f"Unexpected table tilt axis: {table_tilt.axis}",
    )

    left_spindle_rest = ctx.part_world_aabb(left_spindle)
    assert left_spindle_rest is not None
    with ctx.pose({left_chuck: math.pi / 2.0}):
        left_spindle_rot = ctx.part_world_aabb(left_spindle)
        assert left_spindle_rot is not None
        ctx.expect_contact(left_spindle, left_head)
        ctx.check(
            "left_spindle_rotates_about_its_axle",
            left_spindle_rot[1][1] < left_spindle_rest[1][1] - 0.012,
            f"Left spindle rest ymax={left_spindle_rest[1][1]:.4f}, rotated ymax={left_spindle_rot[1][1]:.4f}",
        )

    left_lever_rest = ctx.part_world_aabb(left_feed_lever)
    assert left_lever_rest is not None
    with ctx.pose({left_feed_joint: math.radians(45.0)}):
        left_lever_pulled = ctx.part_world_aabb(left_feed_lever)
        assert left_lever_pulled is not None
        ctx.expect_contact(left_feed_lever, left_head)
        ctx.check(
            "left_feed_lever_swings_upward",
            left_lever_pulled[1][2] > left_lever_rest[1][2] + 0.04,
            f"Left feed lever rest zmax={left_lever_rest[1][2]:.4f}, pulled zmax={left_lever_pulled[1][2]:.4f}",
        )

    table_rest = ctx.part_world_aabb(tilt_table)
    assert table_rest is not None
    with ctx.pose({table_tilt: math.radians(20.0)}):
        table_raised = ctx.part_world_aabb(tilt_table)
        assert table_raised is not None
        ctx.expect_contact(tilt_table, tilt_bracket)
        ctx.expect_gap(left_spindle, tilt_table, axis="z", min_gap=0.04)
        ctx.expect_gap(right_spindle, tilt_table, axis="z", min_gap=0.04)
        ctx.check(
            "shared_table_tilts",
            table_raised[1][2] > table_rest[1][2] + 0.05,
            f"Table rest zmax={table_rest[1][2]:.4f}, tilted zmax={table_raised[1][2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
