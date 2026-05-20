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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _ray_profile(
    *,
    start_radius: float,
    total_length: float,
    base_half_width: float,
    shoulder_half_width: float,
    mid_half_width: float,
) -> list[tuple[float, float]]:
    tip_x = start_radius + total_length
    shoulder_x = start_radius + total_length * 0.18
    mid_x = start_radius + total_length * 0.48
    pre_tip_x = start_radius + total_length * 0.90
    return [
        (start_radius, -base_half_width),
        (shoulder_x, -shoulder_half_width),
        (mid_x, -mid_half_width),
        (pre_tip_x, -mid_half_width * 0.48),
        (tip_x, 0.0),
        (pre_tip_x, mid_half_width * 0.48),
        (mid_x, mid_half_width),
        (shoulder_x, shoulder_half_width),
        (start_radius, base_half_width),
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    return (
        0.5 * (xmin + xmax),
        0.5 * (ymin + ymax),
        0.5 * (zmin + zmax),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="art_deco_sun_ray_ceiling_fixture")

    antique_brass = model.material("antique_brass", rgba=(0.74, 0.64, 0.36, 1.0))
    aged_gold = model.material("aged_gold", rgba=(0.82, 0.73, 0.49, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.93, 0.91, 0.84, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.18, 0.15, 1.0))

    plate_radius = 0.24
    plate_thickness = 0.018
    ray_thickness = 0.006

    long_ray_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            _ray_profile(
                start_radius=0.118,
                total_length=0.269,
                base_half_width=0.013,
                shoulder_half_width=0.018,
                mid_half_width=0.010,
            ),
            ray_thickness,
        ),
        "sunburst_ray_long",
    )
    short_ray_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            _ray_profile(
                start_radius=0.126,
                total_length=0.213,
                base_half_width=0.011,
                shoulder_half_width=0.015,
                mid_half_width=0.008,
            ),
            ray_thickness,
        ),
        "sunburst_ray_short",
    )
    orientation_key_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            [
                (0.078, -0.007),
                (0.091, -0.010),
                (0.104, -0.007),
                (0.114, 0.0),
                (0.104, 0.007),
                (0.091, 0.010),
                (0.078, 0.007),
            ],
            0.007,
        ),
        "dome_orientation_key",
    )

    dome_height = 0.126
    dome_outer_profile = [
        (0.020, 0.000),
        (0.044, 0.012),
        (0.072, 0.038),
        (0.094, 0.078),
        (0.104, 0.108),
        (0.108, dome_height),
    ]
    dome_inner_profile = [
        (0.008, 0.006),
        (0.034, 0.016),
        (0.060, 0.040),
        (0.081, 0.078),
        (0.092, 0.110),
        (0.094, dome_height - 0.004),
    ]
    dome_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            dome_outer_profile,
            dome_inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        )
        .rotate_x(math.pi)
        .translate(0.0, 0.0, dome_height),
        "central_dome_shell",
    )

    backplate = model.part("backplate")
    backplate.visual(
        Cylinder(radius=plate_radius, length=plate_thickness),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness * 0.5)),
        material=antique_brass,
        name="main_backplate",
    )
    backplate.visual(
        Cylinder(radius=0.086, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + 0.003)),
        material=aged_gold,
        name="inner_step_ring",
    )
    backplate.visual(
        Cylinder(radius=0.072, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + 0.005)),
        material=aged_gold,
        name="center_boss",
    )
    backplate.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + 0.010)),
        material=dark_metal,
        name="dome_pivot_housing",
    )

    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        is_long = index % 2 == 0
        backplate.visual(
            long_ray_mesh if is_long else short_ray_mesh,
            origin=Origin(
                xyz=(0.0, 0.0, plate_thickness - 0.0005),
                rpy=(0.0, 0.0, angle),
            ),
            material=antique_brass if is_long else aged_gold,
            name=f"ray_{index + 1:02d}",
        )

    backplate.inertial = Inertial.from_geometry(
        Box((0.80, 0.80, 0.16)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    central_dome = model.part("central_dome")
    central_dome.visual(
        dome_shell_mesh,
        material=frosted_glass,
        name="dome_shell",
    )
    central_dome.visual(
        orientation_key_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=aged_gold,
        name="orientation_tab",
    )
    central_dome.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=aged_gold,
        name="bottom_finial",
    )
    central_dome.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.13),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    model.articulation(
        "backplate_to_central_dome",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=central_dome,
        origin=Origin(xyz=(0.0, 0.0, plate_thickness)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
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
    backplate = object_model.get_part("backplate")
    central_dome = object_model.get_part("central_dome")
    dome_joint = object_model.get_articulation("backplate_to_central_dome")

    ray_names = [visual.name for visual in backplate.visuals if visual.name and visual.name.startswith("ray_")]
    ctx.check(
        "twelve ray arms present",
        len(ray_names) == 12,
        details=f"ray_names={ray_names}",
    )
    ctx.check(
        "dome joint is centered vertical revolute",
        dome_joint.articulation_type == ArticulationType.REVOLUTE
        and dome_joint.axis == (0.0, 0.0, 1.0)
        and dome_joint.motion_limits is not None
        and dome_joint.motion_limits.lower is not None
        and dome_joint.motion_limits.upper is not None
        and dome_joint.motion_limits.lower <= -3.0
        and dome_joint.motion_limits.upper >= 3.0,
        details=(
            f"type={dome_joint.articulation_type}, axis={dome_joint.axis}, "
            f"limits={dome_joint.motion_limits}"
        ),
    )

    with ctx.pose({dome_joint: 0.0}):
        ctx.expect_overlap(
            central_dome,
            backplate,
            axes="xy",
            elem_a="dome_shell",
            elem_b="main_backplate",
            min_overlap=0.19,
            name="dome stays centered within the backplate footprint",
        )
        ctx.expect_contact(
            central_dome,
            backplate,
            elem_a="dome_shell",
            elem_b="main_backplate",
            contact_tol=0.001,
            name="dome shell seats against the backplate",
        )

        rest_tab_center = _aabb_center(ctx.part_element_world_aabb(central_dome, elem="orientation_tab"))
        rest_dome_pos = ctx.part_world_position(central_dome)

    with ctx.pose({dome_joint: math.pi / 2.0}):
        turned_tab_center = _aabb_center(ctx.part_element_world_aabb(central_dome, elem="orientation_tab"))
        turned_dome_pos = ctx.part_world_position(central_dome)

    rotation_ok = (
        rest_tab_center is not None
        and turned_tab_center is not None
        and rest_dome_pos is not None
        and turned_dome_pos is not None
        and abs(rest_tab_center[0]) > 0.08
        and abs(turned_tab_center[1]) > 0.08
        and abs(turned_dome_pos[0] - rest_dome_pos[0]) < 1e-6
        and abs(turned_dome_pos[1] - rest_dome_pos[1]) < 1e-6
        and abs(turned_dome_pos[2] - rest_dome_pos[2]) < 1e-6
    )
    ctx.check(
        "central dome rotates in place for orientation",
        rotation_ok,
        details=(
            f"rest_tab_center={rest_tab_center}, turned_tab_center={turned_tab_center}, "
            f"rest_dome_pos={rest_dome_pos}, turned_dome_pos={turned_dome_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
