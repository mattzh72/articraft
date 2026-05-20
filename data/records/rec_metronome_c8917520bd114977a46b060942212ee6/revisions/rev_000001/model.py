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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _rect_loop(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d, z),
        (half_w, -half_d, z),
        (half_w, half_d, z),
        (-half_w, half_d, z),
    ]


def _collar_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    half_len = length * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_len), (outer_radius, half_len)],
        [(inner_radius, -half_len), (inner_radius, half_len)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pyramid_metronome")

    wood = model.material("wood", rgba=(0.47, 0.31, 0.18, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.27, 0.17, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.63, 0.30, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.57, 0.47, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.75, 0.77, 0.80, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.12, 1.0))

    housing = model.part("housing")

    housing.visual(
        Box((0.132, 0.096, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_wood,
        name="base_plinth",
    )

    body_geom = section_loft(
        [
            _rect_loop(0.106, 0.074, 0.0),
            _rect_loop(0.080, 0.056, 0.070),
            _rect_loop(0.050, 0.034, 0.132),
        ]
    )
    housing.visual(
        mesh_from_geometry(body_geom, "metronome_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=wood,
        name="body_shell",
    )
    housing.visual(
        Box((0.056, 0.036, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=dark_wood,
        name="top_cap",
    )
    housing.visual(
        Box((0.022, 0.012, 0.007)),
        origin=Origin(xyz=(0.024, 0.0, 0.159)),
        material=black,
        name="hinge_saddle",
    )
    housing.visual(
        Box((0.012, 0.026, 0.034)),
        origin=Origin(xyz=(-0.041, 0.0, 0.066)),
        material=dark_wood,
        name="rear_key_pad",
    )
    housing.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(-0.051, 0.0, 0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_key_boss",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.132, 0.096, 0.176)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    pendulum.visual(
        Cylinder(radius=0.0018, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.1225), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Sphere(radius=0.0036),
        origin=Origin(xyz=(0.0, 0.0, 0.2486)),
        material=aged_brass,
        name="rod_tip",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.258)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
    )

    coarse_weight = model.part("coarse_weight")
    coarse_weight.visual(
        _collar_mesh(0.015, 0.0018, 0.022, "coarse_weight_collar"),
        material=aged_brass,
        name="coarse_collar",
    )
    coarse_weight.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.022)),
        mass=0.08,
    )

    fine_weight = model.part("fine_weight")
    fine_weight.visual(
        _collar_mesh(0.0105, 0.0018, 0.014, "fine_weight_collar"),
        material=brass,
        name="fine_collar",
    )
    fine_weight.inertial = Inertial.from_geometry(
        Box((0.021, 0.021, 0.014)),
        mass=0.04,
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="key_stem",
    )
    winding_key.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(-0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="key_hub",
    )
    winding_key.visual(
        Cylinder(radius=0.0032, length=0.028),
        origin=Origin(xyz=(-0.023, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="key_handle",
    )
    winding_key.inertial = Inertial.from_geometry(
        Box((0.030, 0.028, 0.010)),
        mass=0.03,
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.024, 0.0, 0.167)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.45,
            upper=0.45,
        ),
    )

    model.articulation(
        "pendulum_to_coarse_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=coarse_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=-0.025,
            upper=0.025,
        ),
    )

    model.articulation(
        "pendulum_to_fine_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=fine_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=-0.015,
            upper=0.015,
        ),
    )

    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(-0.055, 0.0, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
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

    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    coarse_weight = object_model.get_part("coarse_weight")
    fine_weight = object_model.get_part("fine_weight")
    winding_key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    coarse_slide = object_model.get_articulation("pendulum_to_coarse_weight")
    fine_slide = object_model.get_articulation("pendulum_to_fine_weight")
    key_joint = object_model.get_articulation("housing_to_winding_key")

    ctx.check(
        "metronome articulation types and axes are correct",
        pendulum_joint.articulation_type == ArticulationType.REVOLUTE
        and pendulum_joint.axis == (1.0, 0.0, 0.0)
        and coarse_slide.articulation_type == ArticulationType.PRISMATIC
        and coarse_slide.axis == (0.0, 0.0, 1.0)
        and fine_slide.articulation_type == ArticulationType.PRISMATIC
        and fine_slide.axis == (0.0, 0.0, 1.0)
        and key_joint.articulation_type == ArticulationType.CONTINUOUS
        and key_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"pendulum_axis={pendulum_joint.axis}, coarse_axis={coarse_slide.axis}, "
            f"fine_axis={fine_slide.axis}, key_axis={key_joint.axis}"
        ),
    )

    key_limits = key_joint.motion_limits
    ctx.check(
        "winding key spins continuously without angle bounds",
        key_limits is not None and key_limits.lower is None and key_limits.upper is None,
        details=f"key_limits={key_limits}",
    )

    ctx.expect_overlap(
        coarse_weight,
        pendulum,
        axes="xy",
        elem_a="coarse_collar",
        elem_b="pendulum_rod",
        min_overlap=0.002,
        name="coarse collar stays coaxial with the pendulum rod",
    )
    ctx.expect_overlap(
        coarse_weight,
        pendulum,
        axes="z",
        elem_a="coarse_collar",
        elem_b="pendulum_rod",
        min_overlap=0.020,
        name="coarse collar remains engaged on the rod",
    )
    ctx.expect_overlap(
        fine_weight,
        pendulum,
        axes="xy",
        elem_a="fine_collar",
        elem_b="pendulum_rod",
        min_overlap=0.002,
        name="fine collar stays coaxial with the pendulum rod",
    )
    ctx.expect_overlap(
        fine_weight,
        pendulum,
        axes="z",
        elem_a="fine_collar",
        elem_b="pendulum_rod",
        min_overlap=0.012,
        name="fine collar remains engaged on the rod",
    )

    with ctx.pose(
        {
            coarse_slide: coarse_slide.motion_limits.upper,
            fine_slide: fine_slide.motion_limits.lower,
        }
    ):
        ctx.expect_gap(
            fine_weight,
            coarse_weight,
            axis="z",
            min_gap=0.006,
            name="coarse and fine sliders keep a clear gap at their closest settings",
        )

    rest_coarse = _aabb_center(ctx.part_world_aabb(coarse_weight))
    rest_fine = _aabb_center(ctx.part_world_aabb(fine_weight))
    with ctx.pose({coarse_slide: coarse_slide.motion_limits.upper}):
        moved_coarse = _aabb_center(ctx.part_world_aabb(coarse_weight))
        stationary_fine = _aabb_center(ctx.part_world_aabb(fine_weight))
    ctx.check(
        "coarse weight slides without dragging the fine weight",
        rest_coarse is not None
        and moved_coarse is not None
        and rest_fine is not None
        and stationary_fine is not None
        and moved_coarse[2] > rest_coarse[2] + 0.020
        and abs(stationary_fine[2] - rest_fine[2]) < 1e-5,
        details=(
            f"rest_coarse={rest_coarse}, moved_coarse={moved_coarse}, "
            f"rest_fine={rest_fine}, stationary_fine={stationary_fine}"
        ),
    )

    with ctx.pose({fine_slide: fine_slide.motion_limits.upper}):
        stationary_coarse = _aabb_center(ctx.part_world_aabb(coarse_weight))
        moved_fine = _aabb_center(ctx.part_world_aabb(fine_weight))
    ctx.check(
        "fine weight slides without dragging the coarse weight",
        rest_coarse is not None
        and stationary_coarse is not None
        and rest_fine is not None
        and moved_fine is not None
        and moved_fine[2] > rest_fine[2] + 0.010
        and abs(stationary_coarse[2] - rest_coarse[2]) < 1e-5,
        details=(
            f"rest_coarse={rest_coarse}, stationary_coarse={stationary_coarse}, "
            f"rest_fine={rest_fine}, moved_fine={moved_fine}"
        ),
    )

    rest_tip = _aabb_center(ctx.part_element_world_aabb(pendulum, elem="rod_tip"))
    with ctx.pose({pendulum_joint: 0.35}):
        swung_tip = _aabb_center(ctx.part_element_world_aabb(pendulum, elem="rod_tip"))
    ctx.check(
        "pendulum swings laterally from the top shaft",
        rest_tip is not None
        and swung_tip is not None
        and abs(swung_tip[1] - rest_tip[1]) > 0.070
        and swung_tip[2] < rest_tip[2] - 0.010,
        details=f"rest_tip={rest_tip}, swung_tip={swung_tip}",
    )

    ctx.expect_contact(
        winding_key,
        housing,
        elem_a="key_stem",
        elem_b="rear_key_boss",
        name="winding key stem seats against the rear boss",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
