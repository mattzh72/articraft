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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tapered_blade_geometry(
    *,
    blade_length: float,
    blade_chord: float,
    blade_thickness: float,
):
    profile = [
        (0.0, -blade_chord * 0.38),
        (0.024, -blade_chord * 0.49),
        (0.148, -blade_chord * 0.42),
        (0.214, -blade_chord * 0.29),
        (blade_length * 0.97, -blade_chord * 0.11),
        (blade_length, 0.0),
        (blade_length * 0.97, blade_chord * 0.11),
        (0.214, blade_chord * 0.29),
        (0.148, blade_chord * 0.42),
        (0.024, blade_chord * 0.49),
        (0.0, blade_chord * 0.38),
    ]
    return (
        ExtrudeGeometry.centered(profile, blade_thickness, cap=True)
        .rotate_y(math.pi * 0.5)
        .rotate_x(-math.pi * 0.5)
    )


def _add_three_blade_set(
    blade_part,
    *,
    outward_sign: float,
    hub_material,
    blade_material,
    mesh_prefix: str,
) -> None:
    hub_length = 0.054
    hub_radius = 0.050
    spinner_radius = 0.024
    blade_length = 0.245
    blade_chord = 0.062
    blade_thickness = 0.010
    blade_pitch = 0.22 * outward_sign
    x_center = outward_sign * (hub_length * 0.5)
    spinner_x = outward_sign * 0.042
    blade_root_radius = hub_radius - 0.010

    blade_part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(
            xyz=(x_center, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hub_material,
        name="hub_drum",
    )
    blade_part.visual(
        Cylinder(radius=hub_radius * 0.72, length=0.012),
        origin=Origin(
            xyz=(outward_sign * 0.006, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hub_material,
        name="hub_backplate",
    )
    blade_part.visual(
        Sphere(radius=spinner_radius),
        origin=Origin(xyz=(spinner_x, 0.0, 0.0)),
        material=hub_material,
        name="spinner_cap",
    )

    for index, angle in enumerate((0.0, math.tau / 3.0, 2.0 * math.tau / 3.0), start=1):
        blade_part.visual(
            mesh_from_geometry(
                _tapered_blade_geometry(
                    blade_length=blade_length,
                    blade_chord=blade_chord,
                    blade_thickness=blade_thickness,
                ),
                f"{mesh_prefix}_blade_body_{index}",
            ),
            origin=Origin(
                xyz=(
                    outward_sign * 0.034,
                    blade_root_radius * math.cos(angle),
                    blade_root_radius * math.sin(angle),
                ),
                rpy=(angle, blade_pitch, 0.0),
            ),
            material=blade_material,
            name=f"blade_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_motor_ceiling_fan")

    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.20, 1.0))
    brushed_bronze = model.material("brushed_bronze", rgba=(0.39, 0.33, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.56, 0.44, 0.31, 1.0))

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.078, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=satin_black,
        name="ceiling_canopy",
    )
    mount.visual(
        Cylinder(radius=0.012, length=0.325),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=dark_steel,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.412)),
        material=dark_steel,
        name="lower_coupler",
    )
    mount.visual(
        Box((0.094, 0.070, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.458)),
        material=brushed_bronze,
        name="mount_saddle",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.078, length=0.460),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.640, 0.120, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=brushed_bronze,
        name="housing_shell",
    )
    housing.visual(
        Box((0.520, 0.086, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.127)),
        material=dark_steel,
        name="lower_trim",
    )
    housing.visual(
        Cylinder(radius=0.056, length=0.060),
        origin=Origin(
            xyz=(-0.290, 0.0, -0.070),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_steel,
        name="left_motor_pod",
    )
    housing.visual(
        Cylinder(radius=0.056, length=0.060),
        origin=Origin(
            xyz=(0.290, 0.0, -0.070),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_steel,
        name="right_motor_pod",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.640, 0.140, 0.160)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    left_blade_set = model.part("left_blade_set")
    _add_three_blade_set(
        left_blade_set,
        outward_sign=-1.0,
        hub_material=satin_black,
        blade_material=warm_wood,
        mesh_prefix="left_set",
    )
    left_blade_set.inertial = Inertial.from_geometry(
        Cylinder(radius=0.270, length=0.060),
        mass=2.4,
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
    )

    right_blade_set = model.part("right_blade_set")
    _add_three_blade_set(
        right_blade_set,
        outward_sign=1.0,
        hub_material=satin_black,
        blade_material=warm_wood,
        mesh_prefix="right_set",
    )
    right_blade_set.inertial = Inertial.from_geometry(
        Cylinder(radius=0.270, length=0.060),
        mass=2.4,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
    )

    model.articulation(
        "mount_to_housing",
        ArticulationType.FIXED,
        parent=mount,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, -0.474)),
    )
    model.articulation(
        "housing_to_left_blade_set",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_blade_set,
        origin=Origin(xyz=(-0.320, 0.0, -0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=18.0),
    )
    model.articulation(
        "housing_to_right_blade_set",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_blade_set,
        origin=Origin(xyz=(0.320, 0.0, -0.070)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=18.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


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

    mount = object_model.get_part("mount")
    housing = object_model.get_part("housing")
    left_blade_set = object_model.get_part("left_blade_set")
    right_blade_set = object_model.get_part("right_blade_set")

    mount_to_housing = object_model.get_articulation("mount_to_housing")
    left_motor = object_model.get_articulation("housing_to_left_blade_set")
    right_motor = object_model.get_articulation("housing_to_right_blade_set")

    ctx.check(
        "housing is fixed to the downrod",
        mount_to_housing.articulation_type == ArticulationType.FIXED,
        details=f"type={mount_to_housing.articulation_type}",
    )
    ctx.check(
        "dual motors use continuous rotation joints",
        left_motor.articulation_type == ArticulationType.CONTINUOUS
        and right_motor.articulation_type == ArticulationType.CONTINUOUS
        and left_motor.motion_limits is not None
        and right_motor.motion_limits is not None
        and left_motor.motion_limits.lower is None
        and left_motor.motion_limits.upper is None
        and right_motor.motion_limits.lower is None
        and right_motor.motion_limits.upper is None,
        details=(
            f"left_type={left_motor.articulation_type}, right_type={right_motor.articulation_type}, "
            f"left_limits={left_motor.motion_limits}, right_limits={right_motor.motion_limits}"
        ),
    )
    ctx.check(
        "motor axes are horizontal and counter-rotating",
        left_motor.axis == (1.0, 0.0, 0.0) and right_motor.axis == (-1.0, 0.0, 0.0),
        details=f"left_axis={left_motor.axis}, right_axis={right_motor.axis}",
    )

    ctx.expect_contact(
        mount,
        housing,
        elem_a="mount_saddle",
        elem_b="housing_shell",
        contact_tol=0.001,
        name="mount saddle seats on the housing",
    )
    ctx.expect_contact(
        left_blade_set,
        housing,
        elem_a="hub_drum",
        elem_b="left_motor_pod",
        contact_tol=0.001,
        name="left blade hub mounts to left motor pod",
    )
    ctx.expect_contact(
        right_blade_set,
        housing,
        elem_a="hub_drum",
        elem_b="right_motor_pod",
        contact_tol=0.001,
        name="right blade hub mounts to right motor pod",
    )

    left_rest = _aabb_center(ctx.part_element_world_aabb(left_blade_set, elem="blade_1"))
    with ctx.pose({left_motor: math.pi * 0.5}):
        left_spun = _aabb_center(ctx.part_element_world_aabb(left_blade_set, elem="blade_1"))
    ctx.check(
        "left blade set visibly rotates around a horizontal axis",
        left_rest is not None
        and left_spun is not None
        and abs(left_spun[0] - left_rest[0]) < 0.01
        and abs(left_spun[2] - left_rest[2]) > 0.08,
        details=f"rest={left_rest}, spun={left_spun}",
    )

    right_rest = _aabb_center(ctx.part_element_world_aabb(right_blade_set, elem="blade_1"))
    with ctx.pose({right_motor: math.pi * 0.5}):
        right_spun = _aabb_center(ctx.part_element_world_aabb(right_blade_set, elem="blade_1"))
    ctx.check(
        "right blade set visibly rotates around a horizontal axis",
        right_rest is not None
        and right_spun is not None
        and abs(right_spun[0] - right_rest[0]) < 0.01
        and abs(right_spun[2] - right_rest[2]) > 0.08,
        details=f"rest={right_rest}, spun={right_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
