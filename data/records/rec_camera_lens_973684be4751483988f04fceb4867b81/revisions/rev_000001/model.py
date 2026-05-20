from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


OPTICAL_AXIS_ROTATION = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _circle_profile(radius: float, *, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _gear_profile(
    root_radius: float,
    tip_radius: float,
    *,
    teeth: int,
    tip_fraction: float = 0.54,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    step = (2.0 * math.pi) / teeth
    root_fraction = 0.5 * (1.0 - tip_fraction)
    for tooth_index in range(teeth):
        start = tooth_index * step
        tip_start = start + (root_fraction * step)
        tip_end = start + ((root_fraction + tip_fraction) * step)
        end = start + step
        for angle, radius in (
            (start, root_radius),
            (tip_start, tip_radius),
            (tip_end, tip_radius),
            (end, root_radius),
        ):
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _shell_from_profiles(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _straight_shell(
    *,
    inner_radius: float,
    outer_radius: float,
    x0: float,
    x1: float,
) -> MeshGeometry:
    return _shell_from_profiles(
        [(outer_radius, x0), (outer_radius, x1)],
        [(inner_radius, x0), (inner_radius, x1)],
    )


def _gear_band(
    *,
    inner_radius: float,
    root_radius: float,
    tip_radius: float,
    x_center: float,
    width: float,
    teeth: int,
) -> MeshGeometry:
    return ExtrudeWithHolesGeometry(
        _gear_profile(root_radius, tip_radius, teeth=teeth),
        [_circle_profile(inner_radius, segments=56)],
        width,
        center=True,
        cap=True,
        closed=True,
    ).translate(0.0, 0.0, x_center)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _focus_barrel_geometry() -> MeshGeometry:
    return _merge_geometries(
        _straight_shell(inner_radius=0.0550, outer_radius=0.0585, x0=0.056, x1=0.0755),
        _gear_band(
            inner_radius=0.0550,
            root_radius=0.0582,
            tip_radius=0.0620,
            x_center=0.090,
            width=0.031,
            teeth=72,
        ),
        _straight_shell(inner_radius=0.0550, outer_radius=0.0585, x0=0.1045, x1=0.136),
    )


def _aperture_ring_geometry() -> MeshGeometry:
    return _merge_geometries(
        _straight_shell(inner_radius=0.0550, outer_radius=0.0566, x0=0.028, x1=0.0358),
        _gear_band(
            inner_radius=0.0550,
            root_radius=0.0562,
            tip_radius=0.0590,
            x_center=0.040,
            width=0.010,
            teeth=54,
        ),
        _straight_shell(inner_radius=0.0550, outer_radius=0.0566, x0=0.0442, x1=0.052),
    )


def _mount_shell_geometry() -> MeshGeometry:
    return _shell_from_profiles(
        [
            (0.024, 0.000),
            (0.031, 0.005),
            (0.034, 0.009),
            (0.040, 0.014),
            (0.041, 0.018),
        ],
        [
            (0.019, 0.000),
            (0.020, 0.006),
            (0.021, 0.010),
            (0.023, 0.018),
        ],
    )


def _main_body_geometry() -> MeshGeometry:
    return _merge_geometries(
        _shell_from_profiles(
            [
                (0.041, 0.018),
                (0.055, 0.024),
            ],
            [
                (0.023, 0.018),
                (0.033, 0.024),
            ],
        ),
        _straight_shell(inner_radius=0.033, outer_radius=0.055, x0=0.024, x1=0.028),
        _straight_shell(inner_radius=0.033, outer_radius=0.054, x0=0.028, x1=0.052),
        _straight_shell(inner_radius=0.0345, outer_radius=0.055, x0=0.052, x1=0.056),
        _straight_shell(inner_radius=0.0355, outer_radius=0.054, x0=0.056, x1=0.136),
        _straight_shell(inner_radius=0.0345, outer_radius=0.055, x0=0.136, x1=0.140),
        _shell_from_profiles(
            [
                (0.055, 0.140),
                (0.050, 0.158),
                (0.053, 0.165),
            ],
            [
                (0.0345, 0.140),
                (0.033, 0.158),
                (0.032, 0.165),
            ],
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_pl_mount_prime_lens")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.16, 0.16, 0.17, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.70, 0.71, 0.74, 1.0))
    index_white = model.material("index_white", rgba=(0.90, 0.90, 0.86, 1.0))

    lens_body = model.part("lens_body")
    lens_body.visual(
        mesh_from_geometry(_mount_shell_geometry(), "pl_mount_shell"),
        origin=OPTICAL_AXIS_ROTATION,
        material=mount_metal,
        name="pl_mount_shell",
    )
    lens_body.visual(
        mesh_from_geometry(_main_body_geometry(), "prime_lens_main_shell"),
        origin=OPTICAL_AXIS_ROTATION,
        material=matte_black,
        name="main_barrel_shell",
    )
    lens_body.visual(
        Box((0.006, 0.013, 0.008)),
        origin=Origin(xyz=(0.014, 0.0265, 0.000)),
        material=mount_metal,
        name="lug_upper",
    )
    lens_body.visual(
        Box((0.006, 0.013, 0.008)),
        origin=Origin(xyz=(0.014, -0.0265, 0.000)),
        material=mount_metal,
        name="lug_lower",
    )
    lens_body.visual(
        Box((0.006, 0.008, 0.013)),
        origin=Origin(xyz=(0.014, 0.000, 0.0265)),
        material=mount_metal,
        name="lug_left",
    )
    lens_body.visual(
        Box((0.006, 0.008, 0.013)),
        origin=Origin(xyz=(0.014, 0.000, -0.0265)),
        material=mount_metal,
        name="lug_right",
    )
    lens_body.visual(
        Box((0.008, 0.0015, 0.006)),
        origin=Origin(xyz=(0.146, 0.000, 0.0505)),
        material=index_white,
        name="front_index_mark",
    )
    lens_body.inertial = Inertial.from_geometry(
        Box((0.165, 0.112, 0.112)),
        mass=1.7,
        origin=Origin(xyz=(0.0825, 0.000, 0.000)),
    )

    focus_barrel = model.part("focus_barrel")
    focus_barrel.visual(
        mesh_from_geometry(_focus_barrel_geometry(), "focus_barrel_shell"),
        origin=OPTICAL_AXIS_ROTATION,
        material=rubber_black,
        name="focus_barrel_shell",
    )
    focus_barrel.visual(
        Box((0.022, 0.010, 0.0025)),
        origin=Origin(xyz=(0.090, 0.000, 0.0622)),
        material=index_white,
        name="focus_scale_pad",
    )
    focus_barrel.inertial = Inertial.from_geometry(
        Box((0.080, 0.124, 0.124)),
        mass=0.34,
        origin=Origin(xyz=(0.096, 0.000, 0.000)),
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_geometry(_aperture_ring_geometry(), "aperture_ring_shell"),
        origin=OPTICAL_AXIS_ROTATION,
        material=satin_black,
        name="aperture_ring_shell",
    )
    aperture_ring.visual(
        Box((0.014, 0.008, 0.0025)),
        origin=Origin(xyz=(0.040, 0.000, 0.0593)),
        material=index_white,
        name="aperture_scale_pad",
    )
    aperture_ring.inertial = Inertial.from_geometry(
        Box((0.026, 0.118, 0.118)),
        mass=0.18,
        origin=Origin(xyz=(0.040, 0.000, 0.000)),
    )

    model.articulation(
        "body_to_focus_barrel",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=focus_barrel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "body_to_aperture_ring",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=aperture_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-0.9,
            upper=0.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lens_body = object_model.get_part("lens_body")
    focus_barrel = object_model.get_part("focus_barrel")
    aperture_ring = object_model.get_part("aperture_ring")
    focus_joint = object_model.get_articulation("body_to_focus_barrel")
    aperture_joint = object_model.get_articulation("body_to_aperture_ring")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    ctx.check(
        "focus barrel uses a revolute optical-axis joint",
        focus_joint.articulation_type == ArticulationType.REVOLUTE and focus_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}",
    )
    ctx.check(
        "aperture ring uses a revolute optical-axis joint",
        aperture_joint.articulation_type == ArticulationType.REVOLUTE
        and aperture_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={aperture_joint.articulation_type}, axis={aperture_joint.axis}",
    )

    with ctx.pose({focus_joint: 0.0, aperture_joint: 0.0}):
        ctx.expect_overlap(
            focus_barrel,
            lens_body,
            axes="yz",
            min_overlap=0.105,
            name="focus barrel stays coaxial with the main barrel",
        )
        ctx.expect_overlap(
            aperture_ring,
            lens_body,
            axes="yz",
            min_overlap=0.105,
            name="aperture ring stays coaxial with the main barrel",
        )
        ctx.expect_overlap(
            focus_barrel,
            lens_body,
            axes="x",
            min_overlap=0.078,
            name="focus barrel wraps the forward body section",
        )
        ctx.expect_overlap(
            aperture_ring,
            lens_body,
            axes="x",
            min_overlap=0.022,
            name="aperture ring wraps the rear control section",
        )

    with ctx.pose({focus_joint: 0.0}):
        focus_rest = _aabb_center(ctx.part_element_world_aabb(focus_barrel, elem="focus_scale_pad"))
    with ctx.pose({focus_joint: 1.5}):
        focus_rotated = _aabb_center(ctx.part_element_world_aabb(focus_barrel, elem="focus_scale_pad"))
    ctx.check(
        "focus barrel visibly rotates around the optical axis",
        focus_rest is not None
        and focus_rotated is not None
        and focus_rotated[1] < focus_rest[1] - 0.045
        and focus_rotated[2] < focus_rest[2] - 0.045,
        details=f"rest={focus_rest}, rotated={focus_rotated}",
    )

    with ctx.pose({aperture_joint: 0.0}):
        aperture_rest = _aabb_center(ctx.part_element_world_aabb(aperture_ring, elem="aperture_scale_pad"))
    with ctx.pose({aperture_joint: -0.8}):
        aperture_rotated = _aabb_center(ctx.part_element_world_aabb(aperture_ring, elem="aperture_scale_pad"))
    ctx.check(
        "aperture ring visibly rotates around the optical axis",
        aperture_rest is not None
        and aperture_rotated is not None
        and aperture_rotated[1] > aperture_rest[1] + 0.035
        and aperture_rotated[2] < aperture_rest[2] - 0.012,
        details=f"rest={aperture_rest}, rotated={aperture_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
