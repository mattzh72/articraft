from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    ConeGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _ellipse_profile(
    center_radius: float,
    radial_radius: float,
    half_width: float,
    *,
    samples: int = 40,
) -> list[tuple[float, float]]:
    return [
        (
            center_radius + radial_radius * math.cos((2.0 * math.pi * i) / samples),
            half_width * math.sin((2.0 * math.pi * i) / samples),
        )
        for i in range(samples)
    ]


def _spike_origin(radius: float, angle: float, z_pos: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z_pos),
        rpy=(0.0, math.pi / 2.0, angle),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="acupressure_ring")

    polished_steel = model.material("polished_steel", rgba=(0.85, 0.87, 0.90, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))

    band_center_radius = 0.01115
    band_radial_radius = 0.00175
    band_half_width = 0.00315
    band_outer_radius = band_center_radius + band_radial_radius

    lip_center_radius = 0.01250
    lip_tube_radius = 0.00070
    lip_z_offset = 0.00298

    roller_center_radius = 0.01485
    roller_radial_radius = 0.00195
    roller_half_width = 0.00200
    roller_outer_radius = roller_center_radius + roller_radial_radius

    spike_base_radius = 0.00055
    spike_length = 0.00135
    spike_embed = 0.00045
    spike_center_radius = roller_outer_radius + (0.5 * spike_length) - spike_embed

    band_mesh = mesh_from_geometry(
        LatheGeometry(
            _ellipse_profile(
                band_center_radius,
                band_radial_radius,
                band_half_width,
                samples=52,
            ),
            segments=96,
        ),
        "inner_band_body",
    )
    lip_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=lip_center_radius,
            tube=lip_tube_radius,
            radial_segments=18,
            tubular_segments=96,
        ),
        "band_retainer_lip",
    )
    roller_mesh = mesh_from_geometry(
        LatheGeometry(
            _ellipse_profile(
                roller_center_radius,
                roller_radial_radius,
                roller_half_width,
                samples=52,
            ),
            segments=96,
        ),
        "outer_roller_body",
    )
    spike_mesh = mesh_from_geometry(
        ConeGeometry(
            radius=spike_base_radius,
            height=spike_length,
            radial_segments=16,
            closed=True,
        ),
        "roller_spike",
    )

    inner_band = model.part("inner_band")
    inner_band.visual(band_mesh, material=polished_steel, name="band_body")
    inner_band.visual(
        lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, lip_z_offset)),
        material=satin_steel,
        name="lip_upper",
    )
    inner_band.visual(
        lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, -lip_z_offset)),
        material=satin_steel,
        name="lip_lower",
    )
    inner_band.inertial = Inertial.from_geometry(
        Cylinder(radius=band_outer_radius + 0.0006, length=2.0 * (band_half_width + lip_tube_radius)),
        mass=0.0045,
    )

    outer_roller = model.part("outer_roller")
    outer_roller.visual(roller_mesh, material=dark_steel, name="roller_body")

    spike_index = 0
    for row_index, (count, z_pos, phase) in enumerate(
        (
            (20, 0.0, 0.0),
            (20, 0.00115, math.pi / 20.0),
            (20, -0.00115, math.pi / 20.0),
        )
    ):
        for spike_number in range(count):
            angle = phase + (2.0 * math.pi * spike_number) / count
            outer_roller.visual(
                spike_mesh,
                origin=_spike_origin(spike_center_radius, angle, z_pos),
                material=satin_steel if row_index == 0 else dark_steel,
                name=f"spike_{spike_index:02d}",
            )
            spike_index += 1

    outer_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=roller_outer_radius + spike_length, length=2.0 * roller_half_width),
        mass=0.0055,
    )

    model.articulation(
        "roller_spin",
        ArticulationType.CONTINUOUS,
        parent=inner_band,
        child=outer_roller,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=18.0),
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

    # For bounded REVOLUTE/PRISMATIC joints, add exact lower/upper motion-limit
    # checks for prompt-critical contacts, clearances, and motion direction. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.expect_contact(lid, body, elem_a="hinge_leaf", elem_b="body_leaf")

    inner_band = object_model.get_part("inner_band")
    outer_roller = object_model.get_part("outer_roller")
    roller_spin = object_model.get_articulation("roller_spin")

    ctx.expect_origin_distance(
        outer_roller,
        inner_band,
        axes="xy",
        max_dist=1e-6,
        name="roller is concentric with band",
    )
    ctx.expect_origin_gap(
        outer_roller,
        inner_band,
        axis="z",
        min_gap=-1e-6,
        max_gap=1e-6,
        name="roller stays centered on the band plane",
    )
    ctx.expect_overlap(
        outer_roller,
        inner_band,
        axes="xy",
        min_overlap=0.024,
        name="roller encircles the inner band",
    )

    rest_spike_aabb = ctx.part_element_world_aabb(outer_roller, elem="spike_00")
    rest_spike_center = _aabb_center(rest_spike_aabb)
    with ctx.pose({roller_spin: math.pi / 2.0}):
        spun_spike_aabb = ctx.part_element_world_aabb(outer_roller, elem="spike_00")
        spun_spike_center = _aabb_center(spun_spike_aabb)
        ctx.expect_origin_distance(
            outer_roller,
            inner_band,
            axes="xy",
            max_dist=1e-6,
            name="roller remains concentric while spinning",
        )

    spike_motion = None
    if rest_spike_center is not None and spun_spike_center is not None:
        spike_motion = math.hypot(
            spun_spike_center[0] - rest_spike_center[0],
            spun_spike_center[1] - rest_spike_center[1],
        )
    ctx.check(
        "outer roller visibly rotates",
        spike_motion is not None and spike_motion > 0.020,
        details=f"rest={rest_spike_center}, spun={spun_spike_center}, motion={spike_motion}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
