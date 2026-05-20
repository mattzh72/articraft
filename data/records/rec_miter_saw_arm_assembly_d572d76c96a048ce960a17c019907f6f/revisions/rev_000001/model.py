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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _annular_sector_profile(
    *,
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    segments: int = 40,
) -> list[tuple[float, float]]:
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    if end <= start:
        end += 2.0 * math.pi

    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start + (end - start) * t
        y = outer_radius * math.cos(angle)
        z = outer_radius * math.sin(angle)
        outer.append((z, y))
    for index in range(segments, -1, -1):
        t = index / segments
        angle = start + (end - start) * t
        y = inner_radius * math.cos(angle)
        z = inner_radius * math.sin(angle)
        inner.append((z, y))
    return outer + inner


def _annular_sector_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    thickness: float,
) -> object:
    geom = ExtrudeGeometry(
        _annular_sector_profile(
            outer_radius=outer_radius,
            inner_radius=inner_radius,
            start_deg=start_deg,
            end_deg=end_deg,
        ),
        thickness,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _head_handle_mesh() -> object:
    geom = tube_from_spline_points(
        [
            (-0.006, 0.044, 0.004),
            (0.018, 0.075, 0.040),
            (0.008, 0.128, 0.060),
            (-0.030, 0.142, 0.042),
            (-0.041, 0.108, 0.004),
        ],
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(geom, "miter_saw_handle")


def _aabb_center(aabb):
    return (
        (aabb[0][0] + aabb[1][0]) * 0.5,
        (aabb[0][1] + aabb[1][1]) * 0.5,
        (aabb[0][2] + aabb[1][2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_bevel_sliding_miter_saw")

    base_cast = model.material("base_cast", rgba=(0.70, 0.72, 0.74, 1.0))
    fence_aluminum = model.material("fence_aluminum", rgba=(0.79, 0.81, 0.82, 1.0))
    table_aluminum = model.material("table_aluminum", rgba=(0.63, 0.65, 0.67, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.84, 0.86, 0.88, 1.0))
    housing_red = model.material("housing_red", rgba=(0.78, 0.15, 0.12, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.79, 0.81, 0.83, 1.0))
    guard_gray = model.material("guard_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    motor_black = model.material("motor_black", rgba=(0.16, 0.17, 0.18, 1.0))

    base_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.78, 0.48, 0.055, corner_segments=10),
            0.055,
            center=True,
            cap=True,
            closed=True,
        ),
        "miter_saw_base",
    )
    upper_cover_mesh = _annular_sector_mesh(
        "miter_saw_upper_cover",
        outer_radius=0.147,
        inner_radius=0.108,
        start_deg=30.0,
        end_deg=235.0,
        thickness=0.022,
    )
    lower_guard_mesh = _annular_sector_mesh(
        "miter_saw_lower_guard",
        outer_radius=0.140,
        inner_radius=0.106,
        start_deg=235.0,
        end_deg=385.0,
        thickness=0.022,
    )
    handle_mesh = _head_handle_mesh()

    base = model.part("base")
    base.visual(
        base_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=base_cast,
        name="base_casting",
    )
    base.visual(
        Box((0.15, 0.09, 0.030)),
        origin=Origin(xyz=(-0.24, 0.16, 0.015)),
        material=dark_trim,
        name="left_front_foot",
    )
    base.visual(
        Box((0.15, 0.09, 0.030)),
        origin=Origin(xyz=(0.24, 0.16, 0.015)),
        material=dark_trim,
        name="right_front_foot",
    )
    base.visual(
        Cylinder(radius=0.170, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=table_aluminum,
        name="turntable_seat",
    )
    base.visual(
        Box((0.14, 0.045, 0.058)),
        origin=Origin(xyz=(-0.22, -0.142, 0.084)),
        material=base_cast,
        name="left_fence_pedestal",
    )
    base.visual(
        Box((0.14, 0.045, 0.058)),
        origin=Origin(xyz=(0.22, -0.142, 0.084)),
        material=base_cast,
        name="right_fence_pedestal",
    )
    base.visual(
        Box((0.30, 0.018, 0.092)),
        origin=Origin(xyz=(-0.205, -0.160, 0.159)),
        material=fence_aluminum,
        name="left_fence",
    )
    base.visual(
        Box((0.30, 0.018, 0.092)),
        origin=Origin(xyz=(0.205, -0.160, 0.159)),
        material=fence_aluminum,
        name="right_fence",
    )
    base.visual(
        Box((0.16, 0.08, 0.255)),
        origin=Origin(xyz=(0.0, -0.187, 0.1825)),
        material=base_cast,
        name="rear_tower",
    )
    base.visual(
        Box((0.08, 0.08, 0.120)),
        origin=Origin(xyz=(-0.11, -0.190, 0.115)),
        material=base_cast,
        name="left_gusset",
    )
    base.visual(
        Box((0.08, 0.08, 0.120)),
        origin=Origin(xyz=(0.11, -0.190, 0.115)),
        material=base_cast,
        name="right_gusset",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.78, 0.48, 0.31)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.145, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=table_aluminum,
        name="table_disc",
    )
    table.visual(
        Cylinder(radius=0.058, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_trim,
        name="table_hub",
    )
    table.visual(
        Box((0.050, 0.024, 0.016)),
        origin=Origin(xyz=(0.105, 0.090, 0.013)),
        material=dark_trim,
        name="miter_handle",
    )
    table.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.120, 0.108, 0.024)),
        material=dark_trim,
        name="miter_handle_knob",
    )
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.060),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    arm = model.part("arm")
    arm.visual(
        Box((0.18, 0.09, 0.050)),
        origin=Origin(xyz=(0.0, 0.018, 0.000)),
        material=dark_trim,
        name="pivot_casting",
    )
    arm.visual(
        Box((0.12, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.055, 0.030)),
        material=dark_trim,
        name="rear_rail_block",
    )
    arm.visual(
        Box((0.12, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.030, 0.010)),
        material=dark_trim,
        name="carriage_stop",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.320),
        origin=Origin(xyz=(-0.065, 0.160, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="left_slide_rail",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.320),
        origin=Origin(xyz=(0.065, 0.160, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="right_slide_rail",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.20, 0.36, 0.10)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.150, 0.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.24, 0.10, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=dark_trim,
        name="upper_capture_plate",
    )
    carriage.visual(
        Box((0.24, 0.10, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=dark_trim,
        name="lower_capture_plate",
    )
    carriage.visual(
        Box((0.014, 0.10, 0.051)),
        origin=Origin(xyz=(-0.113, 0.0, 0.0015)),
        material=dark_trim,
        name="left_capture_cheek",
    )
    carriage.visual(
        Box((0.014, 0.10, 0.051)),
        origin=Origin(xyz=(0.113, 0.0, 0.0015)),
        material=dark_trim,
        name="right_capture_cheek",
    )
    carriage.visual(
        Box((0.10, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, -0.045, -0.018)),
        material=dark_trim,
        name="rear_bumper",
    )
    carriage.visual(
        Box((0.030, 0.030, 0.130)),
        origin=Origin(xyz=(-0.107, 0.045, -0.055)),
        material=dark_trim,
        name="left_bevel_cheek",
    )
    carriage.visual(
        Box((0.030, 0.030, 0.130)),
        origin=Origin(xyz=(0.107, 0.045, -0.055)),
        material=dark_trim,
        name="right_bevel_cheek",
    )
    carriage.visual(
        Box((0.07, 0.080, 0.080)),
        origin=Origin(xyz=(0.0, -0.018, -0.050)),
        material=dark_trim,
        name="center_web",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.24, 0.12, 0.18)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.010, -0.040)),
    )

    head = model.part("head")
    head.visual(
        Box((0.184, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.006, -0.075)),
        material=dark_trim,
        name="bevel_trunnion",
    )
    head.visual(
        Box((0.024, 0.062, 0.124)),
        origin=Origin(xyz=(0.035, 0.025, -0.002)),
        material=dark_trim,
        name="rise_arm",
    )
    head.visual(
        Box((0.068, 0.060, 0.074)),
        origin=Origin(xyz=(0.048, 0.078, 0.070)),
        material=dark_trim,
        name="saw_yoke",
    )
    head.visual(
        Box((0.050, 0.080, 0.090)),
        origin=Origin(xyz=(0.030, 0.054, -0.074)),
        material=housing_red,
        name="gearcase",
    )
    head.visual(
        Box((0.022, 0.112, 0.094)),
        origin=Origin(xyz=(-0.026, 0.088, -0.012)),
        material=housing_red,
        name="upper_blade_cover",
    )
    head.visual(
        Box((0.060, 0.032, 0.040)),
        origin=Origin(xyz=(0.000, 0.076, 0.000)),
        material=housing_red,
        name="cover_bridge",
    )
    head.visual(
        Box((0.018, 0.010, 0.040)),
        origin=Origin(xyz=(-0.030, 0.130, 0.000)),
        material=housing_red,
        name="guard_stop",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.012, 0.055, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="arbor_cap",
    )
    head.visual(
        Cylinder(radius=0.052, length=0.160),
        origin=Origin(xyz=(0.110, 0.060, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_black,
        name="motor_body",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.046),
        origin=Origin(xyz=(0.213, 0.060, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_black,
        name="motor_endbell",
    )
    head.visual(
        Box((0.026, 0.024, 0.082)),
        origin=Origin(xyz=(0.078, 0.095, 0.095)),
        material=handle_black,
        name="handle_post",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.100),
        origin=Origin(xyz=(0.114, 0.105, 0.136), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="top_handle",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.21, 0.22)),
        mass=6.4,
        origin=Origin(xyz=(0.10, 0.050, -0.060)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.127, length=0.004),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_steel,
        name="blade_disc",
    )
    blade.visual(
        Cylinder(radius=0.129, length=0.0015),
        origin=Origin(xyz=(-0.0018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_steel,
        name="blade_rim",
    )
    blade.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="blade_hub",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.129, length=0.016),
        mass=0.9,
    )

    guard = model.part("lower_guard")
    guard.visual(
        Box((0.022, 0.100, 0.120)),
        origin=Origin(xyz=(-0.030, 0.105, -0.070)),
        material=guard_gray,
        name="lower_guard_shell",
    )
    guard.visual(
        Box((0.022, 0.030, 0.042)),
        origin=Origin(xyz=(-0.030, 0.085, -0.020)),
        material=guard_gray,
        name="guard_link",
    )
    guard.inertial = Inertial.from_geometry(
        Box((0.05, 0.20, 0.18)),
        mass=0.55,
        origin=Origin(xyz=(-0.030, 0.015, -0.060)),
    )

    model.articulation(
        "base_to_table_miter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-math.radians(55.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "base_to_arm_drop",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, -0.180, 0.335)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "arm_to_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.130, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.7,
            lower=0.0,
            upper=0.150,
        ),
    )
    model.articulation(
        "carriage_to_head_bevel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.0, 0.060, -0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=-math.radians(47.0),
            upper=math.radians(47.0),
        ),
    )
    model.articulation(
        "head_to_blade_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=blade,
        origin=Origin(xyz=(0.0, 0.055, -0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=80.0),
    )
    model.articulation(
        "head_to_lower_guard",
        ArticulationType.REVOLUTE,
        parent=head,
        child=guard,
        origin=Origin(xyz=(0.0, 0.055, -0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    base = object_model.get_part("base")
    table = object_model.get_part("table")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("head")
    blade = object_model.get_part("blade")
    guard = object_model.get_part("lower_guard")

    miter = object_model.get_articulation("base_to_table_miter")
    drop = object_model.get_articulation("base_to_arm_drop")
    slide = object_model.get_articulation("arm_to_carriage_slide")
    bevel = object_model.get_articulation("carriage_to_head_bevel")
    blade_spin = object_model.get_articulation("head_to_blade_spin")
    guard_hinge = object_model.get_articulation("head_to_lower_guard")

    ctx.check("miter axis is vertical", miter.axis == (0.0, 0.0, 1.0), details=str(miter.axis))
    ctx.check("drop axis is horizontal x", drop.axis == (-1.0, 0.0, 0.0), details=str(drop.axis))
    ctx.check("slide axis follows rails", slide.axis == (0.0, 1.0, 0.0), details=str(slide.axis))
    ctx.check("bevel axis is horizontal y", bevel.axis == (0.0, 1.0, 0.0), details=str(bevel.axis))
    ctx.check("blade spins on arbor axis", blade_spin.axis == (1.0, 0.0, 0.0), details=str(blade_spin.axis))
    ctx.check("guard hinge parallels arbor", guard_hinge.axis == (1.0, 0.0, 0.0), details=str(guard_hinge.axis))

    ctx.expect_contact(table, base, name="table seated on turntable base")
    ctx.expect_contact(arm, base, name="drop arm mounted on rear tower")
    ctx.expect_contact(carriage, arm, name="carriage captured on twin rails")
    ctx.expect_contact(head, carriage, name="bevel head mounted to carriage")
    ctx.expect_contact(blade, head, name="blade mounted to arbor")
    ctx.expect_contact(guard, head, name="lower guard linked to blade cover")

    ctx.expect_overlap(table, base, axes="xy", min_overlap=0.28, name="table centered over base")
    ctx.expect_gap(blade, table, axis="z", min_gap=0.01, max_gap=0.20, name="raised blade clears table")
    ctx.expect_origin_gap(
        table,
        arm,
        axis="y",
        min_gap=0.10,
        name="rail arm pivots behind miter table",
    )

    head_rest = ctx.part_world_position(head)
    blade_rest = ctx.part_world_position(blade)
    assert head_rest is not None
    assert blade_rest is not None

    handle_rest_aabb = ctx.part_element_world_aabb(table, elem="miter_handle")
    guard_rest_aabb = ctx.part_element_world_aabb(guard, elem="lower_guard_shell")
    assert handle_rest_aabb is not None
    assert guard_rest_aabb is not None
    handle_rest_center = _aabb_center(handle_rest_aabb)
    guard_rest_center = _aabb_center(guard_rest_aabb)

    with ctx.pose({slide: 0.120}):
        head_slide = ctx.part_world_position(head)
        assert head_slide is not None
        ctx.check(
            "carriage extends forward on rails",
            head_slide[1] > head_rest[1] + 0.09,
            details=f"rest_y={head_rest[1]:.4f}, slide_y={head_slide[1]:.4f}",
        )
        ctx.expect_contact(carriage, arm, name="carriage remains captured when extended")

    with ctx.pose({drop: math.radians(42.0)}):
        blade_drop = ctx.part_world_position(blade)
        assert blade_drop is not None
        ctx.check(
            "drop pivot lowers blade toward table",
            blade_drop[2] < blade_rest[2] - 0.08,
            details=f"rest_z={blade_rest[2]:.4f}, drop_z={blade_drop[2]:.4f}",
        )

    with ctx.pose({bevel: math.radians(32.0)}):
        blade_bevel = ctx.part_world_position(blade)
        assert blade_bevel is not None
        ctx.check(
            "dual bevel shifts blade laterally",
            abs(blade_bevel[0] - blade_rest[0]) > 0.03,
            details=f"rest_x={blade_rest[0]:.4f}, bevel_x={blade_bevel[0]:.4f}",
        )

    with ctx.pose({miter: math.radians(35.0)}):
        handle_turn_aabb = ctx.part_element_world_aabb(table, elem="miter_handle")
        assert handle_turn_aabb is not None
        handle_turn_center = _aabb_center(handle_turn_aabb)
        ctx.check(
            "miter table rotates about vertical axis",
            abs(handle_turn_center[0] - handle_rest_center[0]) > 0.03
            and abs(handle_turn_center[1] - handle_rest_center[1]) > 0.02,
            details=(
                f"rest_xy=({handle_rest_center[0]:.4f}, {handle_rest_center[1]:.4f}), "
                f"turn_xy=({handle_turn_center[0]:.4f}, {handle_turn_center[1]:.4f})"
            ),
        )

    with ctx.pose({guard_hinge: math.radians(55.0)}):
        guard_open_aabb = ctx.part_element_world_aabb(guard, elem="lower_guard_shell")
        assert guard_open_aabb is not None
        guard_open_center = _aabb_center(guard_open_aabb)
        ctx.check(
            "lower guard retracts upward",
            guard_open_center[2] > guard_rest_center[2] + 0.03,
            details=f"rest_z={guard_rest_center[2]:.4f}, open_z={guard_open_center[2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
