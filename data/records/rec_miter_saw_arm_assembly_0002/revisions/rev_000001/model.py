from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_miter_saw_arm_assembly", assets=ASSETS)

    base_paint = model.material("base_paint", rgba=(0.23, 0.25, 0.28, 1.0))
    cast_aluminum = model.material("cast_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    guard_finish = model.material("guard_finish", rgba=(0.64, 0.67, 0.71, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    handle_rubber = model.material("handle_rubber", rgba=(0.10, 0.11, 0.12, 1.0))

    def _save_mesh(filename: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))

    def _yz_section(
        x_pos: float,
        width_y: float,
        height_z: float,
        center_z: float,
        radius: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_val, center_z + z_val)
            for y_val, z_val in rounded_rect_profile(width_y, height_z, radius)
        ]

    def _arc_points_2d(
        radius: float,
        start_deg: float,
        end_deg: float,
        *,
        segments: int,
    ) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for index in range(segments + 1):
            t = index / segments
            angle = math.radians(start_deg + ((end_deg - start_deg) * t))
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
        return points

    def _guard_profile() -> list[tuple[float, float]]:
        outer = _arc_points_2d(0.105, 20.0, 215.0, segments=28)
        inner = _arc_points_2d(0.090, 215.0, 20.0, segments=24)
        return outer + inner

    def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
        return _arc_points_2d(radius, 0.0, 360.0, segments=segments)[:-1]

    base = model.part("base")
    base.visual(
        Box((0.44, 0.34, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=base_paint,
        name="base_plinth",
    )
    base.visual(
        Box((0.12, 0.18, 0.05)),
        origin=Origin(xyz=(-0.185, 0.0, 0.055)),
        material=base_paint,
        name="rear_support",
    )
    base.visual(
        Cylinder(radius=0.110, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast_aluminum,
        name="turntable_seat",
    )
    base.visual(
        Box((0.070, 0.070, 0.020)),
        origin=Origin(xyz=(-0.050, -0.095, 0.070)),
        material=cast_aluminum,
        name="left_fence",
    )
    base.visual(
        Box((0.070, 0.070, 0.020)),
        origin=Origin(xyz=(-0.050, 0.095, 0.070)),
        material=cast_aluminum,
        name="right_fence",
    )
    base.visual(
        Box((0.080, 0.260, 0.020)),
        origin=Origin(xyz=(-0.090, 0.0, 0.070)),
        material=cast_aluminum,
        name="fence_bridge",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.44, 0.34, 0.10)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    turntable = model.part("turntable")
    turntable_top = ExtrudeWithHolesGeometry(
        _circle_profile(0.095, segments=56),
        [[(-0.018, -0.006), (0.100, -0.006), (0.100, 0.006), (-0.018, 0.006)]],
        0.012,
        center=True,
    )
    turntable.visual(
        _save_mesh("miter_saw_turntable_top.obj", turntable_top),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cast_aluminum,
        name="table_top",
    )
    turntable.visual(
        Box((0.090, 0.110, 0.008)),
        origin=Origin(xyz=(0.060, 0.0, 0.010)),
        material=cast_aluminum,
        name="table_front",
    )
    turntable.visual(
        Box((0.055, 0.012, 0.020)),
        origin=Origin(xyz=(0.020, -0.078, 0.016)),
        material=base_paint,
        name="yaw_pointer",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.02),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    yoke = model.part("rear_pivot_yoke")
    yoke.visual(
        Box((0.030, 0.010, 0.245)),
        origin=Origin(xyz=(0.0, -0.031, 0.1225)),
        material=base_paint,
        name="left_plate",
    )
    yoke.visual(
        Box((0.030, 0.010, 0.245)),
        origin=Origin(xyz=(0.0, 0.031, 0.1225)),
        material=base_paint,
        name="right_plate",
    )
    yoke.visual(
        Box((0.020, 0.052, 0.145)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0725)),
        material=base_paint,
        name="rear_web",
    )
    yoke.visual(
        Box((0.024, 0.052, 0.012)),
        origin=Origin(xyz=(-0.002, 0.0, 0.239)),
        material=base_paint,
        name="top_bridge",
    )
    yoke.visual(
        Box((0.040, 0.080, 0.025)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0125)),
        material=base_paint,
        name="lower_brace",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.050, 0.080, 0.245)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
    )

    arm = model.part("swing_arm")
    arm_shell = section_loft(
        [
            _yz_section(0.015, 0.050, 0.052, 0.020, 0.010),
            _yz_section(0.070, 0.046, 0.044, 0.038, 0.009),
            _yz_section(0.115, 0.040, 0.034, 0.050, 0.007),
        ]
    )
    handle_geom = tube_from_spline_points(
        [
            (0.072, -0.020, 0.050),
            (0.080, -0.028, 0.074),
            (0.100, 0.000, 0.088),
            (0.080, 0.028, 0.074),
            (0.072, 0.020, 0.050),
        ],
        radius=0.0045,
        samples_per_segment=16,
        radial_segments=14,
        cap_ends=True,
    )
    arm.visual(
        Cylinder(radius=0.016, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=base_paint,
        name="pivot_barrel",
    )
    arm.visual(
        Box((0.040, 0.038, 0.030)),
        origin=Origin(xyz=(0.024, 0.0, 0.018)),
        material=base_paint,
        name="pivot_support",
    )
    arm.visual(
        _save_mesh("miter_saw_arm_shell.obj", arm_shell),
        material=base_paint,
        name="arm_shell",
    )
    arm.visual(
        Box((0.018, 0.040, 0.032)),
        origin=Origin(xyz=(0.120, 0.0, 0.055)),
        material=base_paint,
        name="front_pad",
    )
    arm.visual(
        _save_mesh("miter_saw_handle.obj", handle_geom),
        material=handle_rubber,
        name="top_handle",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.14, 0.06, 0.10)),
        mass=1.8,
        origin=Origin(xyz=(0.070, 0.0, 0.040)),
    )

    guard = model.part("blade_guard_housing")
    guard_shell = (
        ExtrudeGeometry(_guard_profile(), 0.032, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.106, 0.0, -0.025)
    )
    guard.visual(
        Box((0.032, 0.044, 0.024)),
        origin=Origin(xyz=(0.016, 0.0, 0.000)),
        material=guard_finish,
        name="mount_block",
    )
    guard.visual(
        Box((0.086, 0.034, 0.022)),
        origin=Origin(xyz=(0.059, 0.0, -0.002)),
        material=guard_finish,
        name="mount_neck",
    )
    guard.visual(
        Box((0.026, 0.020, 0.078)),
        origin=Origin(xyz=(0.096, 0.0, -0.025)),
        material=guard_finish,
        name="hub_web",
    )
    guard.visual(
        _save_mesh("miter_saw_guard_shell.obj", guard_shell),
        material=guard_finish,
        name="guard_shell",
    )
    guard.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.106, 0.0, -0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_finish,
        name="hub_core",
    )
    guard.visual(
        Cylinder(radius=0.088, length=0.003),
        origin=Origin(xyz=(0.106, 0.0, -0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blade_steel,
        name="blade_disc",
    )
    guard.inertial = Inertial.from_geometry(
        Box((0.20, 0.05, 0.18)),
        mass=2.4,
        origin=Origin(xyz=(0.106, 0.0, -0.010)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-math.radians(50.0),
            upper=math.radians(50.0),
        ),
    )
    model.articulation(
        "base_to_yoke",
        ArticulationType.FIXED,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(-0.145, 0.0, 0.080)),
    )
    model.articulation(
        "yoke_to_arm",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=arm,
        origin=Origin(xyz=(0.018, 0.0, 0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "arm_to_guard",
        ArticulationType.FIXED,
        parent=arm,
        child=guard,
        origin=Origin(xyz=(0.129, 0.0, 0.055)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    yoke = object_model.get_part("rear_pivot_yoke")
    arm = object_model.get_part("swing_arm")
    guard = object_model.get_part("blade_guard_housing")

    table_yaw = object_model.get_articulation("base_to_turntable")
    arm_drop = object_model.get_articulation("yoke_to_arm")

    turntable_seat = base.get_visual("turntable_seat")
    rear_support = base.get_visual("rear_support")
    left_plate = yoke.get_visual("left_plate")
    right_plate = yoke.get_visual("right_plate")
    pivot_barrel = arm.get_visual("pivot_barrel")
    front_pad = arm.get_visual("front_pad")
    table_top = turntable.get_visual("table_top")
    guard_mount = guard.get_visual("mount_block")
    blade_disc = guard.get_visual("blade_disc")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.check("base_part_present", base is not None)
    ctx.check("turntable_part_present", turntable is not None)
    ctx.check("rear_pivot_yoke_part_present", yoke is not None)
    ctx.check("swing_arm_part_present", arm is not None)
    ctx.check("blade_guard_part_present", guard is not None)

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=table_top,
        negative_elem=turntable_seat,
        name="turntable_sits_on_base_seat",
    )
    ctx.expect_overlap(
        turntable,
        base,
        axes="xy",
        min_overlap=0.15,
        elem_a=table_top,
        elem_b=turntable_seat,
        name="turntable_centered_over_seat",
    )
    ctx.expect_contact(
        yoke,
        base,
        elem_a=left_plate,
        elem_b=rear_support,
        name="left_yoke_plate_supported_by_base",
    )
    ctx.expect_contact(
        yoke,
        base,
        elem_a=right_plate,
        elem_b=rear_support,
        name="right_yoke_plate_supported_by_base",
    )
    ctx.expect_contact(
        arm,
        yoke,
        elem_a=pivot_barrel,
        elem_b=left_plate,
        name="arm_barrel_contacts_left_yoke_plate",
    )
    ctx.expect_contact(
        arm,
        yoke,
        elem_a=pivot_barrel,
        elem_b=right_plate,
        name="arm_barrel_contacts_right_yoke_plate",
    )
    ctx.expect_contact(
        guard,
        arm,
        elem_a=guard_mount,
        elem_b=front_pad,
        name="guard_mounts_to_arm_front_pad",
    )
    ctx.expect_gap(
        turntable,
        yoke,
        axis="x",
        min_gap=0.010,
        name="turntable_clears_rear_yoke",
    )

    guard_rest = ctx.part_world_position(guard)
    assert guard_rest is not None
    with ctx.pose({arm_drop: math.radians(50.0)}):
        guard_lowered = ctx.part_world_position(guard)
        assert guard_lowered is not None
        ctx.check(
            "arm_rotation_lowers_guard",
            guard_lowered[2] < guard_rest[2] - 0.090,
            details=f"rest_z={guard_rest[2]:.4f}, lowered_z={guard_lowered[2]:.4f}",
        )
        ctx.check(
            "arm_rotation_swings_guard_forward",
            guard_lowered[0] > guard_rest[0] - 0.030,
            details=f"rest_x={guard_rest[0]:.4f}, lowered_x={guard_lowered[0]:.4f}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no_part_overlap_when_arm_is_lowered_to_cut_pose"
        )

    with ctx.pose({table_yaw: math.radians(45.0)}):
        ctx.expect_gap(
            turntable,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=table_top,
            negative_elem=turntable_seat,
            name="turntable_stays_seated_when_yawed",
        )
        ctx.expect_overlap(
            turntable,
            base,
            axes="xy",
            min_overlap=0.15,
            elem_a=table_top,
            elem_b=turntable_seat,
            name="turntable_remains_over_base_seat_when_yawed",
        )
        ctx.expect_contact(
            yoke,
            base,
            elem_a=left_plate,
            elem_b=rear_support,
            name="yoke_remains_fixed_while_table_yaws",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
