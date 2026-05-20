from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _arm_point(distance: float, *, pitch: float, z_offset: float = 0.0, y: float = 0.0) -> tuple[float, float, float]:
    return (cos(pitch) * distance, y, sin(pitch) * distance + z_offset)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_miter_saw")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    darker_cast = model.material("darker_cast", rgba=(0.60, 0.62, 0.65, 1.0))
    steel = model.material("steel", rgba=(0.44, 0.46, 0.49, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    saw_guard = model.material("saw_guard", rgba=(0.93, 0.71, 0.14, 1.0))
    blade_silver = model.material("blade_silver", rgba=(0.82, 0.83, 0.85, 1.0))
    knob_red = model.material("knob_red", rgba=(0.72, 0.14, 0.12, 1.0))

    arm_raise = 0.72
    arm_pitch_rpy = (0.0, -arm_raise, 0.0)

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.70, 0.62, 0.34)),
        mass=18.0,
        origin=Origin(xyz=(-0.04, 0.0, 0.17)),
    )

    base_casting = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.64, 0.46, 0.055, corner_segments=10),
        0.055,
        cap=True,
        closed=True,
    )
    base.visual(_save_mesh("miter_saw_base_casting", base_casting), material=cast_aluminum, name="base_casting")
    base.visual(Box((0.09, 0.12, 0.020)), origin=Origin(xyz=(0.22, 0.15, -0.010)), material=rubber, name="foot_front_right")
    base.visual(Box((0.09, 0.12, 0.020)), origin=Origin(xyz=(0.22, -0.15, -0.010)), material=rubber, name="foot_front_left")
    base.visual(Box((0.10, 0.13, 0.020)), origin=Origin(xyz=(-0.23, 0.15, -0.010)), material=rubber, name="foot_rear_right")
    base.visual(Box((0.10, 0.13, 0.020)), origin=Origin(xyz=(-0.23, -0.15, -0.010)), material=rubber, name="foot_rear_left")
    base.visual(
        Cylinder(radius=0.20, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=darker_cast,
        name="turntable_seat",
    )
    base.visual(
        Box((0.10, 0.11, 0.060)),
        origin=Origin(xyz=(0.25, 0.0, 0.060)),
        material=darker_cast,
        name="detent_housing",
    )
    base.visual(
        Box((0.05, 0.36, 0.125)),
        origin=Origin(xyz=(-0.215, 0.0, 0.1175)),
        material=cast_aluminum,
        name="rear_support_block",
    )
    base.visual(
        Box((0.22, 0.025, 0.095)),
        origin=Origin(xyz=(-0.10, 0.205, 0.1025)),
        material=cast_aluminum,
        name="fence_right",
    )
    base.visual(
        Box((0.22, 0.025, 0.095)),
        origin=Origin(xyz=(-0.10, -0.205, 0.1025)),
        material=cast_aluminum,
        name="fence_left",
    )
    base.visual(
        Box((0.040, 0.39, 0.040)),
        origin=Origin(xyz=(-0.17, 0.0, 0.065)),
        material=darker_cast,
        name="fence_bridge",
    )
    base.visual(
        Box((0.09, 0.055, 0.23)),
        origin=Origin(xyz=(-0.24, 0.145, 0.170)),
        material=steel,
        name="yoke_right",
    )
    base.visual(
        Box((0.09, 0.055, 0.23)),
        origin=Origin(xyz=(-0.24, -0.145, 0.170)),
        material=steel,
        name="yoke_left",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.055),
        origin=Origin(xyz=(-0.24, 0.175, 0.265), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_stub_right",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.055),
        origin=Origin(xyz=(-0.24, -0.175, 0.265), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_stub_left",
    )
    turntable = model.part("turntable")
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.060),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )
    turntable.visual(
        Cylinder(radius=0.18, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=cast_aluminum,
        name="table_disc",
    )
    turntable.visual(
        Box((0.29, 0.22, 0.014)),
        origin=Origin(xyz=(0.02, 0.0, 0.035)),
        material=cast_aluminum,
        name="table_top",
    )
    turntable.visual(
        Box((0.14, 0.08, 0.030)),
        origin=Origin(xyz=(0.16, 0.0, 0.015)),
        material=darker_cast,
        name="miter_front_lug",
    )
    turntable.visual(
        Box((0.085, 0.050, 0.060)),
        origin=Origin(xyz=(0.02, 0.155, 0.050)),
        material=darker_cast,
        name="clamp_mount_block",
    )
    turntable.visual(
        Box((0.12, 0.07, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, 0.046)),
        material=dark_steel,
        name="table_insert",
    )

    saw_arm = model.part("saw_arm")
    saw_arm.inertial = Inertial.from_geometry(
        Box((0.60, 0.26, 0.36)),
        mass=9.0,
        origin=Origin(xyz=(0.24, 0.0, 0.10)),
    )

    blade_center = _arm_point(0.32, pitch=arm_raise, z_offset=-0.090)
    motor_center = _arm_point(0.45, pitch=arm_raise, y=0.070, z_offset=0.020)
    guard_center = _arm_point(0.29, pitch=arm_raise, z_offset=-0.005)

    saw_arm.visual(
        Cylinder(radius=0.033, length=0.235),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="arm_pivot_barrel",
    )
    saw_arm.visual(
        Box((0.12, 0.11, 0.08)),
        origin=Origin(xyz=_arm_point(0.05, pitch=arm_raise), rpy=arm_pitch_rpy),
        material=steel,
        name="pivot_head",
    )
    saw_arm.visual(
        Box((0.34, 0.085, 0.060)),
        origin=Origin(xyz=_arm_point(0.19, pitch=arm_raise, z_offset=0.005), rpy=arm_pitch_rpy),
        material=steel,
        name="arm_beam",
    )
    saw_arm.visual(
        Box((0.16, 0.10, 0.11)),
        origin=Origin(xyz=_arm_point(0.35, pitch=arm_raise, z_offset=0.010), rpy=arm_pitch_rpy),
        material=steel,
        name="gear_case",
    )
    saw_arm.visual(
        Box((0.13, 0.055, 0.055)),
        origin=Origin(
            xyz=((blade_center[0] + motor_center[0]) * 0.5, 0.035, (blade_center[2] + motor_center[2]) * 0.5),
            rpy=arm_pitch_rpy,
        ),
        material=dark_steel,
        name="motor_bridge",
    )
    saw_arm.visual(
        Cylinder(radius=0.072, length=0.15),
        origin=Origin(xyz=motor_center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="motor_housing",
    )
    saw_arm.visual(
        Box((0.10, 0.11, 0.16)),
        origin=Origin(xyz=guard_center, rpy=arm_pitch_rpy),
        material=saw_guard,
        name="upper_guard",
    )
    saw_arm.visual(
        Box((0.12, 0.05, 0.11)),
        origin=Origin(
            xyz=_arm_point(0.26, pitch=arm_raise, y=-0.025, z_offset=0.055),
            rpy=arm_pitch_rpy,
        ),
        material=saw_guard,
        name="guard_strut",
    )
    saw_arm.visual(
        Cylinder(radius=0.128, length=0.004),
        origin=Origin(xyz=blade_center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=blade_silver,
        name="blade_disc",
    )
    saw_arm.visual(
        Cylinder(radius=0.032, length=0.055),
        origin=Origin(xyz=blade_center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="blade_hub",
    )
    handle_geom = tube_from_spline_points(
        [
            _arm_point(0.21, pitch=arm_raise, y=-0.015, z_offset=0.080),
            _arm_point(0.25, pitch=arm_raise, y=-0.015, z_offset=0.130),
            _arm_point(0.31, pitch=arm_raise, y=-0.010, z_offset=0.155),
            _arm_point(0.39, pitch=arm_raise, y=-0.005, z_offset=0.105),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    saw_arm.visual(_save_mesh("miter_saw_handle", handle_geom), material=dark_steel, name="handle_loop")
    saw_arm.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=_arm_point(0.40, pitch=arm_raise, y=-0.005, z_offset=0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )
    saw_arm.visual(
        Box((0.040, 0.040, 0.055)),
        origin=Origin(xyz=_arm_point(0.37, pitch=arm_raise, y=0.0, z_offset=0.100), rpy=arm_pitch_rpy),
        material=knob_red,
        name="trigger_housing",
    )

    clamp_arm = model.part("clamp_arm")
    clamp_arm.inertial = Inertial.from_geometry(
        Box((0.09, 0.22, 0.14)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.09, 0.02)),
    )
    clamp_arm.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="clamp_knuckle",
    )
    clamp_arm.visual(
        Box((0.060, 0.13, 0.020)),
        origin=Origin(xyz=(0.0, -0.065, 0.018)),
        material=steel,
        name="clamp_beam",
    )
    clamp_arm.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.0, -0.125, -0.006)),
        material=dark_steel,
        name="clamp_post",
    )
    clamp_arm.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, -0.125, -0.038)),
        material=knob_red,
        name="clamp_pad",
    )
    clamp_arm.visual(
        Box((0.035, 0.045, 0.030)),
        origin=Origin(xyz=(0.0, -0.010, 0.014)),
        material=dark_steel,
        name="clamp_hinge_block",
    )

    model.articulation(
        "table_rotation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.6, lower=-0.87, upper=0.87),
    )
    model.articulation(
        "chop_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=saw_arm,
        origin=Origin(xyz=(-0.24, 0.0, 0.265)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=75.0, velocity=1.8, lower=0.0, upper=0.78),
    )
    model.articulation(
        "clamp_pivot",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=clamp_arm,
        origin=Origin(xyz=(0.02, 0.175, 0.0975)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    saw_arm = object_model.get_part("saw_arm")
    clamp_arm = object_model.get_part("clamp_arm")
    table_rotation = object_model.get_articulation("table_rotation")
    chop_hinge = object_model.get_articulation("chop_hinge")
    clamp_pivot = object_model.get_articulation("clamp_pivot")

    ctx.check("base exists", base is not None)
    ctx.check("turntable exists", turntable is not None)
    ctx.check("saw arm exists", saw_arm is not None)
    ctx.check("clamp arm exists", clamp_arm is not None)

    ctx.check(
        "table rotates on vertical axis",
        tuple(round(v, 6) for v in table_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={table_rotation.axis}",
    )
    ctx.check(
        "saw arm uses transverse hinge",
        tuple(round(v, 6) for v in chop_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={chop_hinge.axis}",
    )
    ctx.check(
        "clamp arm uses its own pivot",
        tuple(round(v, 6) for v in clamp_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"axis={clamp_pivot.axis}",
    )

    with ctx.pose({table_rotation: 0.0, chop_hinge: 0.0, clamp_pivot: 0.0}):
        ctx.expect_contact(turntable, base, elem_a="table_disc", elem_b="turntable_seat", contact_tol=0.0015, name="turntable seats on base")
        ctx.expect_contact(
            saw_arm,
            base,
            elem_a="arm_pivot_barrel",
            elem_b="yoke_right",
            contact_tol=0.0005,
            name="arm pivot barrel bears on right yoke",
        )
        ctx.expect_contact(
            saw_arm,
            base,
            elem_a="arm_pivot_barrel",
            elem_b="yoke_left",
            contact_tol=0.0005,
            name="arm pivot barrel bears on left yoke",
        )
        ctx.expect_gap(
            saw_arm,
            turntable,
            axis="z",
            positive_elem="blade_disc",
            negative_elem="table_top",
            min_gap=0.004,
            max_gap=0.20,
            name="raised blade clears table surface",
        )
        ctx.expect_origin_gap(
            clamp_arm,
            turntable,
            axis="y",
            min_gap=0.12,
            max_gap=0.22,
            name="clamp pivot sits to one side of table",
        )
        ctx.expect_overlap(
            clamp_arm,
            turntable,
            axes="xy",
            elem_a="clamp_pad",
            elem_b="table_disc",
            min_overlap=0.010,
            name="clamp pad projects over table footprint",
        )

    clamp_rest = ctx.part_world_position(clamp_arm)
    with ctx.pose({table_rotation: 0.45}):
        clamp_rotated = ctx.part_world_position(clamp_arm)
    ctx.check(
        "table rotation carries clamp around center",
        clamp_rest is not None
        and clamp_rotated is not None
        and ((clamp_rotated[0] - clamp_rest[0]) ** 2 + (clamp_rotated[1] - clamp_rest[1]) ** 2) ** 0.5 > 0.07,
        details=f"rest={clamp_rest}, rotated={clamp_rotated}",
    )

    blade_rest = ctx.part_element_world_aabb(saw_arm, elem="blade_disc")
    with ctx.pose({chop_hinge: 0.35}):
        blade_lowered = ctx.part_element_world_aabb(saw_arm, elem="blade_disc")
    ctx.check(
        "chopping arm lowers blade toward table",
        blade_rest is not None
        and blade_lowered is not None
        and blade_lowered[0][2] < blade_rest[0][2] - 0.03,
        details=f"rest={blade_rest}, lowered={blade_lowered}",
    )

    pad_rest = ctx.part_element_world_aabb(clamp_arm, elem="clamp_pad")
    with ctx.pose({clamp_pivot: 0.45}):
        pad_lowered = ctx.part_element_world_aabb(clamp_arm, elem="clamp_pad")
    ctx.check(
        "clamp arm swings downward on its pivot",
        pad_rest is not None
        and pad_lowered is not None
        and pad_lowered[0][2] < pad_rest[0][2] - 0.02,
        details=f"rest={pad_rest}, lowered={pad_lowered}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
