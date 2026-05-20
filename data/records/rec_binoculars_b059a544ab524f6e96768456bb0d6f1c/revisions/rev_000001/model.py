from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    tube_from_spline_points,
)


def _add_box(part, size, xyz, *, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder_x(part, radius, length, xyz, *, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(part, radius, length, xyz, *, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_z(part, radius, length, xyz, *, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_optical_body(
    part,
    *,
    side,
    prefix,
    mount_material,
    armor_material,
    glass_material,
    x_offset=0.16,
):
    body_y = 0.145 * side
    eyepiece_y = 0.072 * side
    bridge_y = 0.085 * side
    bridge_width = 0.06

    _add_box(
        part,
        (0.135, bridge_width, 0.055),
        (0.02 + x_offset, bridge_y, -0.01),
        material=mount_material,
        name=f"{prefix}_bridge",
    )
    _add_box(
        part,
        (0.24, 0.12, 0.17),
        (0.0 + x_offset, body_y, -0.02),
        material=mount_material,
        name=f"{prefix}_prism_housing",
    )
    _add_cylinder_x(
        part,
        0.077,
        0.44,
        (0.27 + x_offset, body_y, -0.04),
        material=armor_material,
        name=f"{prefix}_objective_shell",
    )
    _add_cylinder_x(
        part,
        0.084,
        0.06,
        (0.52 + x_offset, body_y, -0.04),
        material=mount_material,
        name=f"{prefix}_objective_cell",
    )
    _add_cylinder_x(
        part,
        0.071,
        0.004,
        (0.548 + x_offset, body_y, -0.04),
        material=glass_material,
        name=f"{prefix}_objective_lens",
    )
    _add_box(
        part,
        (0.14, 0.08, 0.10),
        (-0.12 + x_offset, 0.105 * side, -0.005),
        material=mount_material,
        name=f"{prefix}_ocular_shoulder",
    )
    _add_cylinder_x(
        part,
        0.022,
        0.12,
        (-0.245 + x_offset, eyepiece_y, 0.005),
        material=armor_material,
        name=f"{prefix}_eyepiece_tube",
    )
    _add_cylinder_x(
        part,
        0.029,
        0.054,
        (-0.327 + x_offset, eyepiece_y, 0.005),
        material=armor_material,
        name=f"{prefix}_eyecup",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="giant_observation_binocular")

    mount_finish = model.material("mount_finish", rgba=(0.24, 0.25, 0.27, 1.0))
    body_metal = model.material("body_metal", rgba=(0.30, 0.31, 0.33, 1.0))
    armor_black = model.material("armor_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.16, 0.23, 0.30, 1.0))

    base = model.part("pedestal_base")
    _add_cylinder_z(
        base,
        0.23,
        0.04,
        (0.0, 0.0, 0.02),
        material=mount_finish,
        name="base_foot",
    )
    _add_cylinder_z(
        base,
        0.08,
        0.78,
        (0.0, 0.0, 0.43),
        material=mount_finish,
        name="pedestal_column",
    )
    _add_cylinder_z(
        base,
        0.11,
        0.08,
        (0.0, 0.0, 0.82),
        material=body_metal,
        name="azimuth_head",
    )
    _add_box(
        base,
        (0.18, 0.18, 0.05),
        (0.0, 0.0, 0.785),
        material=body_metal,
        name="head_block",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.90)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    fork = model.part("fork_mount")
    _add_cylinder_z(
        fork,
        0.105,
        0.05,
        (0.0, 0.0, 0.025),
        material=body_metal,
        name="pan_turntable",
    )
    _add_box(
        fork,
        (0.14, 0.42, 0.06),
        (0.01, 0.0, 0.08),
        material=body_metal,
        name="fork_bridge",
    )
    _add_box(
        fork,
        (0.08, 0.04, 0.42),
        (0.02, -0.19, 0.29),
        material=body_metal,
        name="left_fork_arm",
    )
    _add_box(
        fork,
        (0.08, 0.04, 0.42),
        (0.02, 0.19, 0.29),
        material=body_metal,
        name="right_fork_arm",
    )
    _add_box(
        fork,
        (0.08, 0.34, 0.03),
        (0.02, 0.0, 0.515),
        material=body_metal,
        name="fork_top_brace",
    )
    _add_box(
        fork,
        (0.05, 0.07, 0.10),
        (-0.045, 0.0, 0.15),
        material=body_metal,
        name="pan_handle_root",
    )
    pan_handle_geom = tube_from_spline_points(
        [
            (-0.05, 0.0, 0.16),
            (-0.14, 0.0, 0.20),
            (-0.28, -0.02, 0.24),
            (-0.42, -0.02, 0.22),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    fork.visual(
        mesh_from_geometry(pan_handle_geom, "pan_handle"),
        material=armor_black,
        name="pan_handle",
    )
    fork.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(-0.42, -0.02, 0.22)),
        material=knob_black,
        name="pan_handle_knob",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.55, 0.48, 0.55)),
        mass=16.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.25)),
    )

    core = model.part("binocular_core")
    _add_cylinder_y(
        core,
        0.018,
        0.34,
        (0.0, 0.0, 0.0),
        material=body_metal,
        name="tilt_trunnion",
    )
    _add_box(
        core,
        (0.12, 0.09, 0.13),
        (-0.015, -0.005, 0.03),
        material=body_metal,
        name="center_mount_block",
    )
    _add_box(
        core,
        (0.05, 0.05, 0.13),
        (0.038, 0.0, 0.08),
        material=body_metal,
        name="hinge_spine",
    )
    _add_box(
        core,
        (0.10, 0.12, 0.035),
        (0.11, -0.06, 0.0),
        material=body_metal,
        name="left_hinge_arm",
    )
    _add_cylinder_z(
        core,
        0.026,
        0.03,
        (0.085, 0.0, 0.055),
        material=body_metal,
        name="hinge_lower_knuckle",
    )
    _add_cylinder_z(
        core,
        0.026,
        0.03,
        (0.085, 0.0, 0.125),
        material=body_metal,
        name="hinge_upper_knuckle",
    )
    _add_box(
        core,
        (0.09, 0.018, 0.05),
        (-0.102, -0.034, 0.08),
        material=body_metal,
        name="focus_support_left",
    )
    _add_box(
        core,
        (0.09, 0.018, 0.05),
        (-0.102, 0.034, 0.08),
        material=body_metal,
        name="focus_support_right",
    )
    _add_optical_body(
        core,
        side=-1,
        prefix="left",
        mount_material=body_metal,
        armor_material=armor_black,
        glass_material=glass_dark,
    )
    core.inertial = Inertial.from_geometry(
        Box((0.90, 0.40, 0.24)),
        mass=13.0,
        origin=Origin(xyz=(0.10, -0.08, 0.00)),
    )

    right_body = model.part("right_barrel")
    _add_cylinder_z(
        right_body,
        0.031,
        0.04,
        (0.0, 0.0, 0.0),
        material=body_metal,
        name="hinge_center_knuckle",
    )
    _add_box(
        right_body,
        (0.12, 0.12, 0.03),
        (0.04, 0.06, 0.0),
        material=body_metal,
        name="hinge_web",
    )
    _add_optical_body(
        right_body,
        side=1,
        prefix="right",
        mount_material=body_metal,
        armor_material=armor_black,
        glass_material=glass_dark,
    )
    right_body.inertial = Inertial.from_geometry(
        Box((0.80, 0.30, 0.22)),
        mass=8.5,
        origin=Origin(xyz=(0.15, 0.12, -0.06)),
    )

    focus_wheel = model.part("focus_wheel")
    _add_cylinder_y(
        focus_wheel,
        0.030,
        0.045,
        (0.0, 0.0, 0.0),
        material=knob_black,
        name="focus_wheel_body",
    )
    _add_cylinder_y(
        focus_wheel,
        0.034,
        0.014,
        (0.0, 0.0, 0.0),
        material=armor_black,
        name="focus_wheel_rim",
    )
    _add_cylinder_y(
        focus_wheel,
        0.008,
        0.05,
        (0.0, 0.0, 0.0),
        material=body_metal,
        name="focus_axle",
    )
    _add_box(
        focus_wheel,
        (0.012, 0.012, 0.012),
        (0.0, 0.0, 0.036),
        material=body_metal,
        name="focus_indicator",
    )
    focus_wheel.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.08)),
        mass=0.35,
        origin=Origin(),
    )

    model.articulation(
        "pan_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "tilt_clutch",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=core,
        origin=Origin(xyz=(0.02, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.7,
            lower=-0.6,
            upper=1.0,
        ),
    )
    model.articulation(
        "center_hinge",
        ArticulationType.REVOLUTE,
        parent=core,
        child=right_body,
        origin=Origin(xyz=(0.095, 0.0, 0.09)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.6,
            lower=-0.10,
            upper=0.24,
        ),
    )
    model.articulation(
        "focus_wheel_spin",
        ArticulationType.REVOLUTE,
        parent=core,
        child=focus_wheel,
        origin=Origin(xyz=(-0.12, 0.0, 0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=6.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("pedestal_base")
    fork = object_model.get_part("fork_mount")
    core = object_model.get_part("binocular_core")
    right_body = object_model.get_part("right_barrel")
    focus_wheel = object_model.get_part("focus_wheel")
    pan_swivel = object_model.get_articulation("pan_swivel")
    tilt_clutch = object_model.get_articulation("tilt_clutch")
    center_hinge = object_model.get_articulation("center_hinge")
    focus_spin = object_model.get_articulation("focus_wheel_spin")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    def _dims_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(maxs[i] - mins[i] for i in range(3))

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

    ctx.expect_contact(base, fork, name="fork turntable seats on the pedestal head")
    ctx.expect_contact(fork, core, name="binocular core is carried by the fork trunnion")
    ctx.expect_contact(core, right_body, name="right optical body is captured by the center hinge")
    ctx.expect_contact(core, focus_wheel, name="focus wheel axle sits in the bridge supports")

    left_objective_dims = _dims_from_aabb(
        ctx.part_element_world_aabb(core, elem="left_objective_shell")
    )
    ctx.check(
        "objectives read as giant binocular barrels",
        left_objective_dims is not None
        and left_objective_dims[0] > 0.40
        and left_objective_dims[1] > 0.14
        and left_objective_dims[2] > 0.14,
        details=f"left objective dims={left_objective_dims}",
    )

    rest_handle = _center_from_aabb(ctx.part_element_world_aabb(fork, elem="pan_handle_knob"))
    with ctx.pose({pan_swivel: 0.9}):
        turned_handle = _center_from_aabb(
            ctx.part_element_world_aabb(fork, elem="pan_handle_knob")
        )
    ctx.check(
        "pan arm swings around the vertical axis",
        rest_handle is not None
        and turned_handle is not None
        and turned_handle[1] < rest_handle[1] - 0.15,
        details=f"rest={rest_handle}, turned={turned_handle}",
    )

    rest_left_objective = _center_from_aabb(
        ctx.part_element_world_aabb(core, elem="left_objective_cell")
    )
    with ctx.pose({tilt_clutch: 0.55}):
        raised_left_objective = _center_from_aabb(
            ctx.part_element_world_aabb(core, elem="left_objective_cell")
        )
    ctx.check(
        "tilt clutch lifts the binocular assembly upward",
        rest_left_objective is not None
        and raised_left_objective is not None
        and raised_left_objective[2] > rest_left_objective[2] + 0.12,
        details=f"rest={rest_left_objective}, tilted={raised_left_objective}",
    )

    rest_right_objective = _center_from_aabb(
        ctx.part_element_world_aabb(right_body, elem="right_objective_cell")
    )
    with ctx.pose({center_hinge: 0.18}):
        opened_right_objective = _center_from_aabb(
            ctx.part_element_world_aabb(right_body, elem="right_objective_cell")
        )
    ctx.check(
        "center hinge splays the right barrel outward for interpupillary adjustment",
        rest_right_objective is not None
        and opened_right_objective is not None
        and opened_right_objective[1] > rest_right_objective[1] + 0.05,
        details=f"rest={rest_right_objective}, opened={opened_right_objective}",
    )

    rest_indicator = _center_from_aabb(
        ctx.part_element_world_aabb(focus_wheel, elem="focus_indicator")
    )
    with ctx.pose({focus_spin: 0.9}):
        spun_indicator = _center_from_aabb(
            ctx.part_element_world_aabb(focus_wheel, elem="focus_indicator")
        )
    ctx.check(
        "focus wheel rotates about its axle",
        rest_indicator is not None
        and spun_indicator is not None
        and spun_indicator[0] > rest_indicator[0] + 0.01,
        details=f"rest={rest_indicator}, spun={spun_indicator}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
