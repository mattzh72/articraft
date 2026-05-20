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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


LENS_AXIS_LENGTH = 0.145
LENS_BARREL_RADIUS = 0.041
GEAR_INNER_RADIUS = 0.044
GEAR_ROOT_RADIUS = 0.0495
GEAR_TIP_RADIUS = 0.0525
GEAR_WIDTH = 0.018
CLAMP_OUTER_FACE_Y = 0.062
SCREW_TRAVEL = 0.003


def circular_profile(
    radius: float,
    *,
    segments: int = 64,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        if clockwise:
            angle = -angle
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def gear_outer_profile(
    *,
    root_radius: float,
    tip_radius: float,
    teeth: int = 48,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    tooth_angle = 2.0 * math.pi / teeth
    for tooth in range(teeth):
        start = tooth * tooth_angle
        points.append((root_radius * math.cos(start), root_radius * math.sin(start)))
        lead = start + 0.28 * tooth_angle
        trail = start + 0.72 * tooth_angle
        points.append((tip_radius * math.cos(lead), tip_radius * math.sin(lead)))
        points.append((tip_radius * math.cos(trail), tip_radius * math.sin(trail)))
    return points


def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="follow_focus_gear_ring")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.13, 0.13, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))

    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        Cylinder(radius=LENS_BARREL_RADIUS, length=LENS_AXIS_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="lens_main_barrel",
    )
    lens_barrel.visual(
        Cylinder(radius=0.046, length=0.024),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="front_filter_ring",
    )
    lens_barrel.visual(
        Cylinder(radius=0.038, length=0.030),
        origin=Origin(xyz=(-0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="rear_barrel_step",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0425, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="focus_grip_zone",
    )
    lens_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=LENS_BARREL_RADIUS, length=LENS_AXIS_LENGTH),
        mass=0.85,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    gear_ring = model.part("gear_ring")
    gear_band_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            gear_outer_profile(
                root_radius=GEAR_ROOT_RADIUS,
                tip_radius=GEAR_TIP_RADIUS,
                teeth=48,
            ),
            [circular_profile(GEAR_INNER_RADIUS, segments=72, clockwise=True)],
            GEAR_WIDTH,
            center=True,
        ),
        "gear_band",
    )
    gear_ring.visual(
        gear_band_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="gear_band",
    )
    for pad_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        gear_ring.visual(
            Cylinder(radius=0.0022, length=GEAR_WIDTH),
            origin=Origin(
                xyz=(0.0, 0.0432 * math.cos(angle), 0.0432 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rubber_black,
            name=f"inner_liner_pad_{pad_index}",
        )
    gear_ring.visual(
        Box((0.020, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.055, 0.008)),
        material=anodized_black,
        name="clamp_housing_top",
    )
    gear_ring.visual(
        Box((0.020, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.055, -0.008)),
        material=anodized_black,
        name="clamp_housing_bottom",
    )
    gear_ring.visual(
        Box((0.004, 0.014, 0.022)),
        origin=Origin(xyz=(-0.008, 0.055, 0.0)),
        material=anodized_black,
        name="clamp_housing_left_web",
    )
    gear_ring.visual(
        Box((0.004, 0.014, 0.022)),
        origin=Origin(xyz=(0.008, 0.055, 0.0)),
        material=anodized_black,
        name="clamp_housing_right_web",
    )
    gear_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=GEAR_TIP_RADIUS, length=GEAR_WIDTH),
        mass=0.12,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    clamp_screw = model.part("clamp_screw")
    clamp_screw.visual(
        Cylinder(radius=0.0024, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="screw_shaft",
    )
    clamp_screw.visual(
        Cylinder(radius=0.0031, length=0.002),
        origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tip_pad",
    )
    clamp_screw.visual(
        Cylinder(radius=0.0065, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="thumb_head",
    )
    clamp_screw.visual(
        Cylinder(radius=0.008, length=0.0015),
        origin=Origin(xyz=(0.0, 0.00575, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knurl_ring",
    )
    clamp_screw.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0065, length=0.022),
        mass=0.02,
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "lens_to_gear_ring",
        ArticulationType.REVOLUTE,
        parent=lens_barrel,
        child=gear_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=8.0,
            lower=-3.0,
            upper=3.0,
        ),
    )
    model.articulation(
        "gear_ring_to_clamp_screw",
        ArticulationType.PRISMATIC,
        parent=gear_ring,
        child=clamp_screw,
        origin=Origin(xyz=(0.0, CLAMP_OUTER_FACE_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.02,
            lower=0.0,
            upper=SCREW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lens_barrel = object_model.get_part("lens_barrel")
    gear_ring = object_model.get_part("gear_ring")
    clamp_screw = object_model.get_part("clamp_screw")
    ring_joint = object_model.get_articulation("lens_to_gear_ring")
    screw_joint = object_model.get_articulation("gear_ring_to_clamp_screw")

    ctx.check("lens barrel exists", lens_barrel is not None)
    ctx.check("gear ring exists", gear_ring is not None)
    ctx.check("clamp screw exists", clamp_screw is not None)

    ctx.expect_origin_distance(
        lens_barrel,
        gear_ring,
        axes="yz",
        min_dist=0.0,
        max_dist=1e-6,
        name="gear ring stays concentric with the lens barrel",
    )
    ctx.expect_overlap(
        lens_barrel,
        gear_ring,
        axes="x",
        min_overlap=GEAR_WIDTH - 0.002,
        name="gear ring spans a lens focus band width",
    )

    ctx.expect_gap(
        clamp_screw,
        lens_barrel,
        axis="y",
        positive_elem="tip_pad",
        negative_elem="focus_grip_zone",
        max_gap=0.0035,
        max_penetration=0.0,
        name="loose screw tip starts just off the barrel",
    )

    top_aabb_rest = ctx.part_element_world_aabb(gear_ring, elem="clamp_housing_top")
    with ctx.pose({ring_joint: math.pi / 2.0}):
        top_aabb_rotated = ctx.part_element_world_aabb(gear_ring, elem="clamp_housing_top")
    top_center_rest = aabb_center(top_aabb_rest)
    top_center_rotated = aabb_center(top_aabb_rotated)
    ctx.check(
        "gear band rotates about the lens axis",
        top_center_rest is not None
        and top_center_rotated is not None
        and top_center_rotated[2] > top_center_rest[2] + 0.04
        and abs(top_center_rotated[1]) < abs(top_center_rest[1]) * 0.35,
        details=f"rest={top_center_rest}, rotated={top_center_rotated}",
    )

    screw_rest_pos = ctx.part_world_position(clamp_screw)
    with ctx.pose({screw_joint: SCREW_TRAVEL}):
        screw_tight_pos = ctx.part_world_position(clamp_screw)
        ctx.expect_contact(
            clamp_screw,
            lens_barrel,
            elem_a="tip_pad",
            elem_b="focus_grip_zone",
            contact_tol=0.0007,
            name="tightened screw tip contacts the lens barrel",
        )
    ctx.check(
        "clamp screw tightens inward perpendicular to the lens axis",
        screw_rest_pos is not None
        and screw_tight_pos is not None
        and screw_tight_pos[1] < screw_rest_pos[1] - 0.0025
        and abs(screw_tight_pos[0] - screw_rest_pos[0]) < 1e-6
        and abs(screw_tight_pos[2] - screw_rest_pos[2]) < 1e-6,
        details=f"rest={screw_rest_pos}, tight={screw_tight_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
