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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(
    z: float,
    width: float,
    depth: float,
    radius: float,
    *,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    radius = min(radius, width * 0.49, depth * 0.49)
    return [
        (x + x_center, y + y_center, z)
        for x, y in rounded_rect_profile(width, depth, radius)
    ]


def _yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    radius = min(radius, width * 0.49, height * 0.49)
    return [
        (x, y + y_center, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius)
    ]


def _aabb_center(aabb):
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.30, 0.32, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.54, 0.57, 0.60, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.63, 0.67, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        _save_mesh(
            "base_casting",
            section_loft(
                [
                    _xy_section(0.000, 0.58, 0.42, 0.060),
                    _xy_section(0.038, 0.46, 0.34, 0.048),
                    _xy_section(0.075, 0.30, 0.24, 0.034),
                    _xy_section(0.080, 0.26, 0.22, 0.028),
                ]
            ),
        ),
        material=cast_iron,
        name="base_casting",
    )
    frame.visual(
        Cylinder(radius=0.075, length=0.060),
        origin=Origin(xyz=(-0.13, 0.0, 0.110)),
        material=cast_iron,
        name="column_socket",
    )
    frame.visual(
        Cylinder(radius=0.042, length=1.280),
        origin=Origin(xyz=(-0.13, 0.0, 0.780)),
        material=steel,
        name="column",
    )
    frame.visual(
        Cylinder(radius=0.080, length=0.100),
        origin=Origin(xyz=(-0.13, 0.0, 0.890)),
        material=cast_iron,
        name="table_collar",
    )
    frame.visual(
        Box((0.08, 0.10, 0.05)),
        origin=Origin(xyz=(-0.09, 0.0, 0.880)),
        material=cast_iron,
        name="table_arm",
    )
    frame.visual(
        Box((0.08, 0.06, 0.12)),
        origin=Origin(xyz=(-0.09, 0.0, 0.820)),
        material=cast_iron,
        name="table_web",
    )
    frame.visual(
        Box((0.08, 0.03, 0.05)),
        origin=Origin(xyz=(-0.01, 0.04, 0.870)),
        material=cast_iron,
        name="fork_upper_right",
    )
    frame.visual(
        Box((0.08, 0.03, 0.05)),
        origin=Origin(xyz=(-0.01, -0.04, 0.870)),
        material=cast_iron,
        name="fork_upper_left",
    )
    frame.visual(
        Box((0.20, 0.18, 0.18)),
        origin=Origin(xyz=(-0.13, 0.0, 1.460)),
        material=cast_iron,
        name="head_mount",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.42, 1.60)),
        mass=85.0,
        origin=Origin(xyz=(-0.03, 0.0, 0.80)),
    )

    head = model.part("head")
    head.visual(
        Box((0.02, 0.18, 0.14)),
        origin=Origin(xyz=(0.01, 0.0, 0.02)),
        material=cast_iron,
        name="rear_plate",
    )
    head.visual(
        _save_mesh(
            "head_shell",
            section_loft(
                [
                    _yz_section(0.00, 0.18, 0.16, 0.030, z_center=0.02),
                    _yz_section(0.08, 0.24, 0.22, 0.040, z_center=0.04),
                    _yz_section(0.18, 0.28, 0.24, 0.045, z_center=0.03),
                    _yz_section(0.28, 0.18, 0.14, 0.028, z_center=-0.01),
                ]
            ),
        ),
        material=machine_gray,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.072, length=0.180),
        origin=Origin(xyz=(0.11, 0.0, 0.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_gray,
        name="motor",
    )
    head.visual(
        Box((0.12, 0.08, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, -0.01)),
        material=machine_gray,
        name="quill_head",
    )
    head.visual(
        Box((0.06, 0.12, 0.10)),
        origin=Origin(xyz=(0.24, 0.0, -0.01)),
        material=machine_gray,
        name="nose_block",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.140),
        origin=Origin(xyz=(0.13, 0.0, -0.03)),
        material=polished_steel,
        name="quill_sleeve",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(xyz=(0.15, 0.12, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="feed_boss",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.28, 0.30)),
        mass=32.0,
        origin=Origin(xyz=(0.15, 0.0, 0.04)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=polished_steel,
        name="spindle_collar",
    )
    spindle.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=polished_steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        material=black,
        name="chuck_upper",
    )
    spindle.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.242)),
        material=black,
        name="chuck_lower",
    )
    spindle.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.021, 0.0, -0.225)),
        material=black,
        name="key_boss",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.300),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.025, length=0.080),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="trunnion",
    )
    table.visual(
        Box((0.10, 0.048, 0.09)),
        origin=Origin(xyz=(0.06, 0.0, 0.045)),
        material=cast_iron,
        name="support_block",
    )
    table.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.08, 0.0, 0.075)),
        material=cast_iron,
        name="table_boss",
    )
    table.visual(
        Cylinder(radius=0.170, length=0.028),
        origin=Origin(xyz=(0.09, 0.0, 0.100)),
        material=machine_gray,
        name="table_top",
    )
    table.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.18)),
        mass=12.0,
        origin=Origin(xyz=(0.09, 0.0, 0.08)),
    )

    feed_handle = model.part("feed_handle")
    feed_handle.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="feed_hub",
    )
    spoke_angles = (math.radians(10.0), math.radians(130.0), math.radians(250.0))
    for index, angle in enumerate(spoke_angles):
        vx = math.cos(angle)
        vz = math.sin(angle)
        feed_handle.visual(
            Cylinder(radius=0.008, length=0.170),
            origin=Origin(
                xyz=(0.085 * vx, 0.020, 0.085 * vz),
                rpy=(0.0, (math.pi / 2.0) - angle, 0.0),
            ),
            material=black,
            name=f"spoke_{index}",
        )
        feed_handle.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.170 * vx, 0.020, 0.170 * vz)),
            material=black,
            name=f"knob_{index}",
        )
    feed_handle.inertial = Inertial.from_geometry(
        Box((0.36, 0.05, 0.36)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    model.articulation(
        "frame_to_head",
        ArticulationType.FIXED,
        parent=frame,
        child=head,
        origin=Origin(xyz=(-0.03, 0.0, 1.460)),
    )
    model.articulation(
        "head_to_spindle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=spindle,
        origin=Origin(xyz=(0.13, 0.0, -0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=18.0,
            lower=-math.tau,
            upper=math.tau,
        ),
    )
    model.articulation(
        "frame_to_table",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=table,
        origin=Origin(xyz=(0.01, 0.0, 0.870)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "head_to_feed_handle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=feed_handle,
        origin=Origin(xyz=(0.15, 0.14, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.25,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    spindle = object_model.get_part("spindle")
    table = object_model.get_part("table")
    feed_handle = object_model.get_part("feed_handle")

    spindle_spin = object_model.get_articulation("head_to_spindle")
    table_tilt = object_model.get_articulation("frame_to_table")
    feed_turn = object_model.get_articulation("head_to_feed_handle")

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

    ctx.expect_contact(head, frame, contact_tol=5e-4, name="head seats on column mount")
    ctx.expect_contact(
        spindle,
        head,
        contact_tol=5e-4,
        name="spindle seats in quill sleeve",
    )
    ctx.expect_contact(
        feed_handle,
        head,
        contact_tol=5e-4,
        name="feed handle hub mounts to head boss",
    )
    ctx.expect_contact(
        table,
        frame,
        contact_tol=5e-4,
        name="table trunnion sits in bracket fork",
    )
    ctx.expect_overlap(
        spindle,
        table,
        axes="xy",
        min_overlap=0.05,
        name="spindle aligns over round table",
    )
    ctx.expect_gap(
        spindle,
        table,
        axis="z",
        min_gap=0.09,
        max_gap=0.14,
        positive_elem="chuck_lower",
        negative_elem="table_top",
        name="chuck clears tabletop",
    )

    ctx.check(
        "spindle axis is vertical",
        spindle_spin.axis == (0.0, 0.0, 1.0),
        details=f"axis={spindle_spin.axis}",
    )
    ctx.check(
        "table hinge axis is front-back",
        table_tilt.axis == (1.0, 0.0, 0.0),
        details=f"axis={table_tilt.axis}",
    )
    ctx.check(
        "feed handle axis is lateral",
        feed_turn.axis == (0.0, 1.0, 0.0),
        details=f"axis={feed_turn.axis}",
    )

    key_rest = ctx.part_element_world_aabb(spindle, elem="key_boss")
    assert key_rest is not None
    key_rest_center = _aabb_center(key_rest)
    with ctx.pose({spindle_spin: 0.9}):
        key_spun = ctx.part_element_world_aabb(spindle, elem="key_boss")
        assert key_spun is not None
        key_spun_center = _aabb_center(key_spun)
        ctx.check(
            "spindle rotation moves chuck detail around axle",
            abs(key_spun_center[1] - key_rest_center[1]) > 0.015,
            details=f"rest={key_rest_center}, spun={key_spun_center}",
        )

    table_rest = ctx.part_element_world_aabb(table, elem="table_top")
    assert table_rest is not None
    with ctx.pose({table_tilt: math.radians(28.0)}):
        table_tilted = ctx.part_element_world_aabb(table, elem="table_top")
        assert table_tilted is not None
        ctx.expect_contact(table, frame, contact_tol=5e-4, name="table stays hinged while tilted")
        ctx.check(
            "table tilt changes tabletop plane",
            table_tilted[1][2] > table_rest[1][2] + 0.05
            and table_tilted[0][2] < table_rest[0][2] - 0.05,
            details=f"rest={table_rest}, tilted={table_tilted}",
        )

    knob_rest = ctx.part_element_world_aabb(feed_handle, elem="knob_0")
    assert knob_rest is not None
    knob_rest_center = _aabb_center(knob_rest)
    with ctx.pose({feed_turn: 0.8}):
        knob_turned = ctx.part_element_world_aabb(feed_handle, elem="knob_0")
        assert knob_turned is not None
        knob_turned_center = _aabb_center(knob_turned)
        ctx.expect_contact(
            feed_handle,
            head,
            contact_tol=5e-4,
            name="feed handle stays mounted while turning",
        )
        ctx.check(
            "feed handle sweeps through an operating arc",
            abs(knob_turned_center[2] - knob_rest_center[2]) > 0.05,
            details=f"rest={knob_rest_center}, turned={knob_turned_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
