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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_rect_loop_xy(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x_center + x, y_center + y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _rounded_rect_loop_yz(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z_center + z) for y, z in rounded_rect_profile(width_y, height_z, radius)]


def _aabb_center_z(aabb) -> float:
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_mixer_with_blender_jar")

    body_enamel = model.material("body_enamel", rgba=(0.76, 0.12, 0.11, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.82, 0.90, 0.96, 0.32))
    lid_black = model.material("lid_black", rgba=(0.17, 0.18, 0.19, 1.0))

    base = model.part("base")
    pedestal_foot = section_loft(
        [
            _rounded_rect_loop_xy(0.34, 0.24, 0.055, 0.0, x_center=0.03),
            _rounded_rect_loop_xy(0.31, 0.21, 0.048, 0.022, x_center=0.03),
            _rounded_rect_loop_xy(0.27, 0.19, 0.040, 0.045, x_center=0.03),
        ]
    )
    pedestal_neck = section_loft(
        [
            _rounded_rect_loop_xy(0.13, 0.11, 0.026, 0.045, x_center=-0.05),
            _rounded_rect_loop_xy(0.11, 0.10, 0.024, 0.14, x_center=-0.05),
            _rounded_rect_loop_xy(0.09, 0.08, 0.020, 0.22, x_center=-0.05),
        ]
    )
    base.visual(_mesh("pedestal_foot", pedestal_foot), material=body_enamel, name="pedestal_foot")
    base.visual(_mesh("pedestal_neck", pedestal_neck), material=body_enamel, name="pedestal_neck")
    base.visual(
        Box((0.045, 0.022, 0.075)),
        origin=Origin(xyz=(-0.06, 0.063, 0.2575)),
        material=body_enamel,
        name="left_cheek",
    )
    base.visual(
        Box((0.045, 0.022, 0.075)),
        origin=Origin(xyz=(-0.06, -0.063, 0.2575)),
        material=body_enamel,
        name="right_cheek",
    )
    base.visual(
        Box((0.027, 0.034, 0.038)),
        origin=Origin(xyz=(-0.096, 0.057, 0.226)),
        material=body_enamel,
        name="left_gusset",
    )
    base.visual(
        Box((0.027, 0.034, 0.038)),
        origin=Origin(xyz=(-0.096, -0.057, 0.226)),
        material=body_enamel,
        name="right_gusset",
    )
    base.visual(
        Box((0.11, 0.09, 0.012)),
        origin=Origin(xyz=(0.09, 0.0, 0.051)),
        material=trim_dark,
        name="base_trim_pad",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.018, length=0.104),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.09, 0.10, 0.08)),
        origin=Origin(xyz=(-0.005, 0.0, 0.04)),
        material=body_enamel,
        name="hinge_bridge",
    )
    motor_shell = section_loft(
        [
            _rounded_rect_loop_yz(0.04, 0.115, 0.11, 0.026, z_center=0.055),
            _rounded_rect_loop_yz(0.16, 0.155, 0.148, 0.038, z_center=0.072),
            _rounded_rect_loop_yz(0.29, 0.110, 0.102, 0.026, z_center=0.050),
        ]
    )
    head.visual(_mesh("motor_shell", motor_shell), material=body_enamel, name="motor_shell")
    head.visual(
        Cylinder(radius=0.056, length=0.016),
        origin=Origin(xyz=(0.15, 0.0, 0.153)),
        material=trim_dark,
        name="head_hub",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.31, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_cap",
    )

    jar = model.part("jar")
    jar.visual(
        Cylinder(radius=0.058, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=trim_dark,
        name="mount_collar",
    )
    jar.visual(
        Box((0.135, 0.005, 0.230)),
        origin=Origin(xyz=(0.0, 0.070, 0.135)),
        material=glass_clear,
        name="front_wall",
    )
    jar.visual(
        Box((0.135, 0.005, 0.230)),
        origin=Origin(xyz=(0.0, -0.070, 0.135)),
        material=glass_clear,
        name="back_wall",
    )
    jar.visual(
        Box((0.005, 0.135, 0.230)),
        origin=Origin(xyz=(0.070, 0.0, 0.135)),
        material=glass_clear,
        name="right_wall",
    )
    jar.visual(
        Box((0.005, 0.135, 0.230)),
        origin=Origin(xyz=(-0.070, 0.0, 0.135)),
        material=glass_clear,
        name="left_wall",
    )
    jar.visual(
        Box((0.135, 0.015, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, 0.026)),
        material=trim_dark,
        name="bottom_ring_front",
    )
    jar.visual(
        Box((0.135, 0.015, 0.012)),
        origin=Origin(xyz=(0.0, -0.060, 0.026)),
        material=trim_dark,
        name="bottom_ring_back",
    )
    jar.visual(
        Box((0.015, 0.105, 0.012)),
        origin=Origin(xyz=(0.060, 0.0, 0.026)),
        material=trim_dark,
        name="bottom_ring_right",
    )
    jar.visual(
        Box((0.015, 0.105, 0.012)),
        origin=Origin(xyz=(-0.060, 0.0, 0.026)),
        material=trim_dark,
        name="bottom_ring_left",
    )
    jar.visual(
        Box((0.120, 0.120, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=glass_clear,
        name="bottom_plate",
    )
    jar.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=steel,
        name="blade_seat",
    )
    jar.visual(
        Box((0.135, 0.135, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        material=lid_black,
        name="lid",
    )
    jar.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=lid_black,
        name="lid_cap",
    )

    blades = model.part("blades")
    blades.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(),
        material=steel,
        name="hub",
    )
    blades.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="shaft",
    )
    blade_specs = (
        ("blade_0", (0.022, 0.0, 0.002), (0.0, 0.28, 0.0)),
        ("blade_1", (-0.022, 0.0, 0.002), (0.0, -0.28, math.pi)),
        ("blade_2", (0.0, 0.022, 0.002), (0.28, 0.0, math.pi / 2.0)),
        ("blade_3", (0.0, -0.022, 0.002), (-0.28, 0.0, -math.pi / 2.0)),
    )
    for blade_name, xyz, rpy in blade_specs:
        blades.visual(
            Box((0.055, 0.011, 0.003)),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=steel,
            name=blade_name,
        )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.06, 0.0, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "head_to_jar",
        ArticulationType.FIXED,
        parent=head,
        child=jar,
        origin=Origin(xyz=(0.15, 0.0, 0.161)),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=blades,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=24.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    jar = object_model.get_part("jar")
    blades = object_model.get_part("blades")
    head_tilt = object_model.get_articulation("head_tilt")
    blade_spin = object_model.get_articulation("blade_spin")

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

    ctx.expect_contact(head, base, elem_a="hinge_barrel", elem_b="left_cheek")
    ctx.expect_contact(jar, head, elem_a="mount_collar", elem_b="head_hub")
    ctx.expect_within(blades, jar, axes="xy", margin=0.0)
    ctx.expect_contact(blades, jar, elem_a="hub", elem_b="blade_seat")
    ctx.check(
        "head_tilt_axis_is_rear_hinge",
        tuple(round(v, 3) for v in head_tilt.axis) == (0.0, -1.0, 0.0),
        details=f"unexpected head tilt axis: {head_tilt.axis}",
    )
    ctx.check(
        "blade_spin_axis_is_vertical",
        tuple(round(v, 3) for v in blade_spin.axis) == (0.0, 0.0, 1.0),
        details=f"unexpected blade spin axis: {blade_spin.axis}",
    )

    front_cap_rest = ctx.part_element_world_aabb(head, elem="front_cap")
    mount_rest = ctx.part_element_world_aabb(jar, elem="mount_collar")
    if front_cap_rest is None or mount_rest is None:
        ctx.fail("tilt_pose_measurements_available", "Could not resolve rest-pose head or jar feature AABB.")
    else:
        with ctx.pose({head_tilt: math.radians(50.0)}):
            front_cap_open = ctx.part_element_world_aabb(head, elem="front_cap")
            mount_open = ctx.part_element_world_aabb(jar, elem="mount_collar")
            if front_cap_open is None or mount_open is None:
                ctx.fail("tilt_pose_measurements_available_open", "Could not resolve tilted head or jar feature AABB.")
            else:
                ctx.check(
                    "head_tilt_lifts_motor_nose",
                    _aabb_center_z(front_cap_open) > _aabb_center_z(front_cap_rest) + 0.10,
                    details="Motor nose did not rise enough in the tilted pose.",
                )
                ctx.check(
                    "jar_mount_rises_with_tilt_head",
                    _aabb_center_z(mount_open) > _aabb_center_z(mount_rest) + 0.04,
                    details="Blender jar mount did not rise with the motor head in the tilted pose.",
                )
                ctx.expect_gap(
                    jar,
                    base,
                    axis="z",
                    min_gap=0.12,
                    positive_elem="mount_collar",
                    negative_elem="pedestal_foot",
                    name="jar_stays_above_pedestal_when_tilted",
                )

    with ctx.pose({blade_spin: math.pi / 2.0}):
        ctx.expect_within(
            blades,
            jar,
            axes="xy",
            margin=0.0,
            name="blades_remain_inside_jar_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
