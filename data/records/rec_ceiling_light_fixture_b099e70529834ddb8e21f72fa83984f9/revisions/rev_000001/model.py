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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _semi_disc_profile(
    radius: float,
    *,
    x_offset: float = 0.0,
    segments: int = 48,
) -> list[tuple[float, float]]:
    return [
        (x_offset + radius * cos(theta), radius * sin(theta))
        for theta in [(-pi * 0.5) + (pi * step / segments) for step in range(segments + 1)]
    ]


def _semi_disc_section(
    radius: float,
    *,
    z: float,
    x_offset: float = 0.0,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in _semi_disc_profile(radius, x_offset=x_offset, segments=segments)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_circular_oyster_ceiling_light")

    painted_steel = model.material("painted_steel", rgba=(0.92, 0.93, 0.92, 1.0))
    soft_white = model.material("soft_white", rgba=(0.93, 0.93, 0.90, 1.0))
    frosted_diffuser = model.material("frosted_diffuser", rgba=(0.96, 0.96, 0.95, 0.92))
    satin_metal = model.material("satin_metal", rgba=(0.65, 0.66, 0.68, 1.0))

    outer_radius = 0.165
    rim_wall = 0.012
    back_plate_thickness = 0.004
    housing_depth = 0.072
    opening_radius = outer_radius - rim_wall
    opening_x_offset = rim_wall
    hinge_axis_x = opening_x_offset + 0.006
    hinge_axis_z = housing_depth + 0.002

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((outer_radius, outer_radius * 2.0, housing_depth)),
        mass=2.4,
        origin=Origin(xyz=(outer_radius * 0.55, 0.0, housing_depth * 0.5)),
    )

    outer_profile = _semi_disc_profile(outer_radius, x_offset=0.0, segments=64)
    inner_profile = _semi_disc_profile(opening_radius, x_offset=opening_x_offset, segments=64)

    housing_back = ExtrudeGeometry.from_z0(outer_profile, back_plate_thickness)
    housing.visual(
        _save_mesh("housing_back", housing_back),
        material=painted_steel,
        name="housing_back",
    )

    housing_rim = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        housing_depth,
        center=False,
        cap=True,
    )
    housing.visual(
        _save_mesh("housing_rim", housing_rim),
        material=soft_white,
        name="housing_rim",
    )

    housing.visual(
        Cylinder(radius=0.038, length=0.022),
        origin=Origin(
            xyz=(outer_radius * 0.60, 0.0, back_plate_thickness + 0.011),
        ),
        material=painted_steel,
        name="lamp_socket_boss",
    )
    housing.visual(
        Box((0.050, 0.110, 0.020)),
        origin=Origin(
            xyz=(outer_radius * 0.38, 0.0, back_plate_thickness + 0.010),
        ),
        material=satin_metal,
        name="mount_bridge",
    )

    knuckle_radius = 0.006
    housing_knuckle_length = 0.070
    housing_knuckle_y = 0.113
    housing.visual(
        Cylinder(radius=knuckle_radius, length=housing_knuckle_length),
        origin=Origin(
            xyz=(hinge_axis_x, -housing_knuckle_y, hinge_axis_z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=satin_metal,
        name="hinge_knuckle_lower",
    )
    housing.visual(
        Cylinder(radius=knuckle_radius, length=housing_knuckle_length),
        origin=Origin(
            xyz=(hinge_axis_x, housing_knuckle_y, hinge_axis_z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=satin_metal,
        name="hinge_knuckle_upper",
    )
    housing.visual(
        Box((0.018, 0.060, 0.018)),
        origin=Origin(xyz=(hinge_axis_x * 0.5, -housing_knuckle_y, hinge_axis_z - 0.002)),
        material=painted_steel,
        name="hinge_mount_lower",
    )
    housing.visual(
        Box((0.018, 0.060, 0.018)),
        origin=Origin(xyz=(hinge_axis_x * 0.5, housing_knuckle_y, hinge_axis_z - 0.002)),
        material=painted_steel,
        name="hinge_mount_upper",
    )

    diffuser_panel = model.part("diffuser_panel")
    diffuser_panel.inertial = Inertial.from_geometry(
        Box((opening_radius * 0.92, opening_radius * 2.0, 0.028)),
        mass=0.9,
        origin=Origin(xyz=(opening_radius * 0.50, 0.0, 0.010)),
    )

    diffuser_inset = 0.006
    diffuser_radius = opening_radius - 0.008
    diffuser_geom = section_loft(
        [
            _semi_disc_section(
                diffuser_radius,
                z=0.000,
                x_offset=diffuser_inset,
                segments=64,
            ),
            _semi_disc_section(
                diffuser_radius * 0.985,
                z=0.006,
                x_offset=diffuser_inset + 0.004,
                segments=64,
            ),
            _semi_disc_section(
                diffuser_radius * 0.940,
                z=0.014,
                x_offset=diffuser_inset + 0.012,
                segments=64,
            ),
            _semi_disc_section(
                diffuser_radius * 0.840,
                z=0.021,
                x_offset=diffuser_inset + 0.028,
                segments=64,
            ),
        ]
    )
    diffuser_panel.visual(
        _save_mesh("diffuser_lens", diffuser_geom),
        material=frosted_diffuser,
        name="diffuser_lens",
    )
    diffuser_panel.visual(
        Box((diffuser_inset + 0.006, 0.148, 0.008)),
        origin=Origin(
            xyz=((diffuser_inset + 0.006) * 0.5, 0.0, 0.002),
        ),
        material=soft_white,
        name="diffuser_frame",
    )
    diffuser_panel.visual(
        Cylinder(radius=knuckle_radius * 0.92, length=0.140),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=satin_metal,
        name="diffuser_knuckle",
    )

    model.articulation(
        "housing_to_diffuser",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=diffuser_panel,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
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
    diffuser_panel = object_model.get_part("diffuser_panel")
    hinge = object_model.get_articulation("housing_to_diffuser")
    housing_rim = housing.get_visual("housing_rim")
    diffuser_lens = diffuser_panel.get_visual("diffuser_lens")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            diffuser_panel,
            housing,
            axes="xy",
            elem_a=diffuser_lens,
            elem_b=housing_rim,
            min_overlap=0.13,
            name="diffuser covers the front opening when closed",
        )

    closed_aabb = None
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(diffuser_panel)

    open_aabb = None
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(diffuser_panel)

    ctx.check(
        "diffuser swings downward for bulb access",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.08,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
