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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(x_pos: float, width_y: float, height_z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for z, y in rounded_rect_profile(height_z, width_y, radius)]


def _offset_section(
    section: list[tuple[float, float, float]],
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y_pos + y, z_pos + z) for x_pos, y_pos, z_pos in section]


def _mirror_bar_points(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mtb_fork_stem_and_riser_bar")

    lowers_black = model.material("lowers_black", rgba=(0.12, 0.12, 0.13, 1.0))
    crown_black = model.material("crown_black", rgba=(0.17, 0.17, 0.18, 1.0))
    stanchion_gold = model.material("stanchion_gold", rgba=(0.73, 0.58, 0.22, 1.0))
    alloy_gray = model.material("alloy_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    cockpit_black = model.material("cockpit_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_black = model.material("grip_black", rgba=(0.06, 0.06, 0.07, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    fork = model.part("fork")

    steerer_outer = [
        (0.0190, 0.000),
        (0.0190, 0.070),
        (0.0165, 0.125),
        (0.0144, 0.165),
        (0.0144, 0.305),
    ]
    steerer_inner = [
        (0.0164, 0.000),
        (0.0164, 0.070),
        (0.0139, 0.125),
        (0.0118, 0.165),
        (0.0118, 0.305),
    ]
    fork.visual(
        _mesh(
            "tapered_steerer_shell",
            LatheGeometry.from_shell_profiles(steerer_outer, steerer_inner, segments=64),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=alloy_gray,
        name="steerer_shell",
    )

    crown_geom = section_loft(
        [
            _yz_section(-0.016, 0.054, 0.048, 0.012),
            _yz_section(0.020, 0.165, 0.054, 0.018),
            _yz_section(0.068, 0.235, 0.040, 0.015),
        ]
    )
    fork.visual(
        _mesh("fork_crown", crown_geom),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=crown_black,
        name="crown",
    )
    fork.visual(
        Cylinder(radius=0.023, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, -0.004)),
        material=crown_black,
        name="crown_center_boss",
    )
    fork.visual(
        _mesh(
            "headset_spacer",
            LatheGeometry.from_shell_profiles(
                [(0.0175, 0.000), (0.0175, 0.010)],
                [(0.0147, 0.000), (0.0147, 0.010)],
                segments=48,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=alloy_gray,
        name="headset_spacer",
    )

    stanchion_y = 0.109
    fork.visual(
        Cylinder(radius=0.0170, length=0.185),
        origin=Origin(xyz=(0.045, stanchion_y, -0.0925)),
        material=stanchion_gold,
        name="left_stanchion",
    )
    fork.visual(
        Cylinder(radius=0.0170, length=0.185),
        origin=Origin(xyz=(0.045, -stanchion_y, -0.0925)),
        material=stanchion_gold,
        name="right_stanchion",
    )

    fork.visual(
        Cylinder(radius=0.0265, length=0.430),
        origin=Origin(xyz=(0.050, 0.113, -0.335)),
        material=lowers_black,
        name="left_lower_leg",
    )
    fork.visual(
        Cylinder(radius=0.0265, length=0.430),
        origin=Origin(xyz=(0.050, -0.113, -0.335)),
        material=lowers_black,
        name="right_lower_leg",
    )

    arch_geom = tube_from_spline_points(
        [
            (0.048, -0.112, -0.236),
            (0.080, -0.065, -0.205),
            (0.091, 0.000, -0.193),
            (0.080, 0.065, -0.205),
            (0.048, 0.112, -0.236),
        ],
        radius=0.023,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    fork.visual(
        _mesh("fork_arch", arch_geom),
        material=lowers_black,
        name="arch",
    )

    for side_name, side_y in (("left", 0.113), ("right", -0.113)):
        fork.visual(
            Box((0.030, 0.046, 0.040)),
            origin=Origin(xyz=(0.058, side_y, -0.555)),
            material=lowers_black,
            name=f"{side_name}_dropout_foot",
        )
        fork.visual(
            Cylinder(radius=0.011, length=0.030),
            origin=Origin(
                xyz=(0.062, side_y, -0.555),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=lowers_black,
            name=f"{side_name}_axle_boss",
        )

    fork.inertial = Inertial.from_geometry(
        Box((0.28, 0.30, 0.88)),
        mass=2.25,
        origin=Origin(xyz=(0.045, 0.0, -0.150)),
    )

    stem_guide = model.part("stem_guide")
    guide_outer = [(0.0168, 0.000), (0.0168, 0.100)]
    guide_inner = [(0.0152, 0.000), (0.0152, 0.100)]
    stem_guide.visual(
        _mesh(
            "stem_guide_shell",
            LatheGeometry.from_shell_profiles(guide_outer, guide_inner, segments=48),
        ),
        material=alloy_gray,
        name="guide_outer",
    )
    stem_guide.inertial = Inertial.from_geometry(
        Box((0.036, 0.036, 0.100)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    stem = model.part("stem")
    stem_clamp_outer = [(0.0265, 0.000), (0.0280, 0.006), (0.0280, 0.039), (0.0265, 0.045)]
    stem_clamp_inner = [(0.0168, 0.000), (0.0168, 0.045)]
    stem.visual(
        _mesh(
            "stem_steerer_clamp",
            LatheGeometry.from_shell_profiles(stem_clamp_outer, stem_clamp_inner, segments=56),
        ),
        material=cockpit_black,
        name="steerer_clamp",
    )
    stem_body_sections = [
        _offset_section(_yz_section(0.020, 0.032, 0.020, 0.007), z=0.012),
        _offset_section(_yz_section(0.030, 0.027, 0.024, 0.007), z=0.014),
        _offset_section(_yz_section(0.042, 0.024, 0.020, 0.006), z=0.015),
        _offset_section(_yz_section(0.054, 0.022, 0.016, 0.005), z=0.016),
    ]
    stem.visual(
        _mesh("stem_body_beam", section_loft(stem_body_sections)),
        material=cockpit_black,
        name="body_beam",
    )

    bar_clamp_outer = [(0.0245, -0.026), (0.0255, -0.010), (0.0255, 0.010), (0.0245, 0.026)]
    bar_clamp_inner = [(0.0159, -0.026), (0.0159, 0.026)]
    stem.visual(
        _mesh(
            "stem_bar_clamp",
            LatheGeometry.from_shell_profiles(bar_clamp_outer, bar_clamp_inner, segments=56),
        ),
        origin=Origin(
            xyz=(0.075, 0.000, 0.026),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=cockpit_black,
        name="bar_clamp",
    )
    for bolt_name, bolt_z in (("upper", 0.040), ("lower", 0.012)):
        stem.visual(
            Cylinder(radius=0.0045, length=0.014),
            origin=Origin(
                xyz=(0.102, 0.000, bolt_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_steel,
            name=f"{bolt_name}_faceplate_bolt",
        )

    stem.inertial = Inertial.from_geometry(
        Box((0.115, 0.055, 0.060)),
        mass=0.34,
        origin=Origin(xyz=(0.050, 0.000, 0.026)),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.0159, length=0.140),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=cockpit_black,
        name="center_bulge",
    )

    right_bar_points = [
        (0.000, 0.070, 0.000),
        (-0.010, 0.150, 0.015),
        (-0.026, 0.260, 0.028),
        (-0.044, 0.385, 0.017),
    ]
    left_bar_points = _mirror_bar_points(right_bar_points)

    handlebar.visual(
        _mesh(
            "right_riser_bar",
            tube_from_spline_points(
                right_bar_points,
                radius=0.0112,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=cockpit_black,
        name="right_bar",
    )
    handlebar.visual(
        _mesh(
            "left_riser_bar",
            tube_from_spline_points(
                left_bar_points,
                radius=0.0112,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=cockpit_black,
        name="left_bar",
    )

    handlebar.visual(
        Cylinder(radius=0.0160, length=0.130),
        origin=Origin(
            xyz=(-0.044, 0.450, 0.017),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grip_black,
        name="right_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.0160, length=0.130),
        origin=Origin(
            xyz=(-0.044, -0.450, 0.017),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grip_black,
        name="left_grip",
    )

    handlebar.inertial = Inertial.from_geometry(
        Box((0.12, 0.94, 0.10)),
        mass=0.42,
        origin=Origin(xyz=(-0.020, 0.000, 0.015)),
    )

    model.articulation(
        "fork_to_stem_swivel",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem_guide,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-1.10,
            upper=1.10,
        ),
    )

    model.articulation(
        "stem_swivel_to_stem",
        ArticulationType.PRISMATIC,
        parent=stem_guide,
        child=stem,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.15,
            lower=0.0,
            upper=0.045,
        ),
    )

    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.075, 0.0, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-0.50,
            upper=0.50,
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

    fork = object_model.get_part("fork")
    stem_guide = object_model.get_part("stem_guide")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")

    stem_swivel = object_model.get_articulation("fork_to_stem_swivel")
    stem_height = object_model.get_articulation("stem_swivel_to_stem")
    bar_roll = object_model.get_articulation("stem_to_handlebar")

    ctx.check("fork part exists", fork is not None)
    ctx.check("stem guide part exists", stem_guide is not None)
    ctx.check("stem part exists", stem is not None)
    ctx.check("handlebar part exists", handlebar is not None)

    ctx.allow_overlap(
        stem_guide,
        stem,
        elem_a="guide_outer",
        elem_b="steerer_clamp",
        reason=(
            "The threadless stem's steerer interface is modeled as a watertight "
            "sleeve proxy for a slit clamp sliding on the steerer."
        ),
    )
    ctx.allow_overlap(
        handlebar,
        stem,
        elem_a="center_bulge",
        elem_b="bar_clamp",
        reason=(
            "The handlebar center and stem bar clamp use watertight closed-shell "
            "proxies for the split faceplate clamp around the bar."
        ),
    )

    ctx.expect_overlap(
        stem_guide,
        stem,
        axes="z",
        elem_a="guide_outer",
        elem_b="steerer_clamp",
        min_overlap=0.040,
        name="stem clamp retains substantial overlap on guide at rest",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="xz",
        elem_a="center_bulge",
        elem_b="bar_clamp",
        min_overlap=0.028,
        name="bar center section stays captured in the stem clamp envelope",
    )
    ctx.expect_within(
        handlebar,
        stem,
        axes="xz",
        inner_elem="center_bulge",
        outer_elem="bar_clamp",
        margin=0.012,
        name="bar center stays centered within the clamp region",
    )

    stem_rest = ctx.part_world_position(stem)
    with ctx.pose({stem_height: 0.045}):
        stem_high = ctx.part_world_position(stem)
        ctx.expect_overlap(
            stem_guide,
            stem,
            axes="z",
            elem_a="guide_outer",
            elem_b="steerer_clamp",
            min_overlap=0.040,
            name="stem retains insertion at full height adjustment",
        )
    ctx.check(
        "stem slides upward on the steerer",
        stem_rest is not None and stem_high is not None and stem_high[2] > stem_rest[2] + 0.040,
        details=f"rest={stem_rest}, raised={stem_high}",
    )

    rest_bar_clamp_center = _aabb_center(ctx.part_element_world_aabb(stem, elem="bar_clamp"))
    with ctx.pose({stem_swivel: 0.70}):
        turned_bar_clamp_center = _aabb_center(ctx.part_element_world_aabb(stem, elem="bar_clamp"))
    ctx.check(
        "stem rotates around the steerer axis",
        rest_bar_clamp_center is not None
        and turned_bar_clamp_center is not None
        and turned_bar_clamp_center[1] > rest_bar_clamp_center[1] + 0.040,
        details=f"rest={rest_bar_clamp_center}, turned={turned_bar_clamp_center}",
    )

    rest_left_grip_center = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="left_grip"))
    with ctx.pose({bar_roll: 0.40}):
        rolled_left_grip_center = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="left_grip"))
    ctx.check(
        "handlebar rolls within the stem clamp",
        rest_left_grip_center is not None
        and rolled_left_grip_center is not None
        and rolled_left_grip_center[2] > rest_left_grip_center[2] + 0.010,
        details=f"rest={rest_left_grip_center}, rolled={rolled_left_grip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
