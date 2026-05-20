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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float, *, segments: int = 64, clockwise: bool = False
) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    if clockwise:
        points.reverse()
    return points


def _annulus_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=72),
            [_circle_profile(inner_radius, segments=72, clockwise=True)],
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rising_bollard_barrier")

    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    reflector = model.material("reflector", rgba=(0.88, 0.66, 0.12, 1.0))

    housing_depth = 1.25
    housing_outer_radius = 0.18
    rim_inner_radius = 0.156
    guide_radius = 0.112
    floor_thickness = 0.025
    top_lip_thickness = 0.012

    bollard_radius = 0.094
    head_radius = 0.097
    exposed_height = 0.90
    lowered_offset = -0.86

    skirt_outer_radius = 0.176
    skirt_inner_radius = 0.100
    skirt_thickness = 0.006
    skirt_hinge_y = -0.176
    guide_pad_radius = 0.102

    housing_profile = [
        (0.0, -housing_depth),
        (housing_outer_radius, -housing_depth),
        (housing_outer_radius, -top_lip_thickness),
        (housing_outer_radius, 0.0),
        (rim_inner_radius, 0.0),
        (rim_inner_radius, -top_lip_thickness),
        (guide_radius, -top_lip_thickness),
        (guide_radius, -(housing_depth - floor_thickness)),
        (0.0, -(housing_depth - floor_thickness)),
    ]
    housing_mesh = mesh_from_geometry(
        LatheGeometry(housing_profile, segments=80, closed=True),
        "bollard_housing_shell",
    )
    skirt_mesh = _annulus_mesh(
        skirt_outer_radius,
        skirt_inner_radius,
        skirt_thickness,
        "bollard_skirt_ring",
    )

    housing = model.part("housing")
    housing.visual(housing_mesh, material=dark_steel, name="housing_shell")
    housing.visual(
        Box((0.12, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.191, 0.010)),
        material=dark_steel,
        name="hinge_boss",
    )
    housing.visual(
        Box((0.012, 0.060, 0.090)),
        origin=Origin(xyz=(guide_pad_radius + 0.006, 0.0, -0.120)),
        material=matte_black,
        name="guide_pad_xp",
    )
    housing.visual(
        Box((0.012, 0.060, 0.090)),
        origin=Origin(xyz=(-(guide_pad_radius + 0.006), 0.0, -0.120)),
        material=matte_black,
        name="guide_pad_xn",
    )
    housing.visual(
        Box((0.060, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, guide_pad_radius + 0.006, -0.120)),
        material=matte_black,
        name="guide_pad_yp",
    )
    housing.visual(
        Box((0.060, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, -(guide_pad_radius + 0.006), -0.120)),
        material=matte_black,
        name="guide_pad_yn",
    )
    housing.inertial = Inertial.from_geometry(
        Cylinder(radius=housing_outer_radius, length=housing_depth),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, -housing_depth * 0.5)),
    )

    bollard = model.part("bollard")
    bollard.visual(
        Cylinder(radius=bollard_radius, length=1.12),
        origin=Origin(xyz=(0.0, 0.0, -0.56)),
        material=steel,
        name="post_body",
    )
    bollard.visual(
        Cylinder(radius=head_radius, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=steel,
        name="head",
    )
    bollard.visual(
        Cylinder(radius=bollard_radius + 0.0015, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=reflector,
        name="reflective_band",
    )
    bollard.visual(
        Cylinder(radius=guide_pad_radius, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -1.02)),
        material=matte_black,
        name="guide_collar",
    )
    bollard.inertial = Inertial.from_geometry(
        Cylinder(radius=guide_radius, length=1.14),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, -0.57)),
    )

    skirt = model.part("skirt")
    skirt.visual(
        skirt_mesh,
        origin=Origin(xyz=(0.0, abs(skirt_hinge_y), 0.0)),
        material=matte_black,
        name="skirt_plate",
    )
    skirt.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.03)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.18, 0.006)),
    )

    model.articulation(
        "bollard_lift",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=bollard,
        origin=Origin(xyz=(0.0, 0.0, exposed_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.22,
            lower=lowered_offset,
            upper=0.0,
        ),
    )
    model.articulation(
        "skirt_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=skirt,
        origin=Origin(xyz=(0.0, skirt_hinge_y, 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    bollard = object_model.get_part("bollard")
    skirt = object_model.get_part("skirt")
    lift = object_model.get_articulation("bollard_lift")
    skirt_hinge = object_model.get_articulation("skirt_hinge")

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

    ctx.check(
        "lift_joint_is_vertical_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower is not None
        and lift.motion_limits.upper == 0.0
        and lift.motion_limits.lower < -0.80,
        details=f"unexpected lift joint configuration: axis={lift.axis}, limits={lift.motion_limits}",
    )
    ctx.check(
        "skirt_joint_is_rim_hinge",
        skirt_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(skirt_hinge.axis) == (1.0, 0.0, 0.0)
        and skirt_hinge.motion_limits is not None
        and skirt_hinge.motion_limits.upper is not None
        and skirt_hinge.motion_limits.upper > 1.2,
        details=f"unexpected skirt joint configuration: axis={skirt_hinge.axis}, limits={skirt_hinge.motion_limits}",
    )

    ctx.expect_origin_distance(
        bollard,
        housing,
        axes="xy",
        max_dist=0.001,
        name="bollard_stays_centered_in_housing",
    )
    ctx.expect_contact(
        skirt,
        housing,
        elem_a="skirt_plate",
        elem_b="housing_shell",
        name="skirt_lies_flat_on_housing_rim",
    )
    ctx.expect_gap(
        bollard,
        housing,
        axis="z",
        min_gap=0.60,
        positive_elem="reflective_band",
        name="bollard_is_deployed_above_housing",
    )

    lowered_position = (
        lift.motion_limits.lower
        if lift.motion_limits is not None and lift.motion_limits.lower is not None
        else -0.86
    )

    with ctx.pose({lift: lowered_position, skirt_hinge: 1.20}):
        ctx.expect_gap(
            bollard,
            housing,
            axis="z",
            min_gap=0.0,
            max_gap=0.03,
            positive_elem="head",
            negative_elem="housing_shell",
            name="bollard_retracts_nearly_flush",
        )
        skirt_plate_aabb = ctx.part_element_world_aabb(skirt, elem="skirt_plate")
        ctx.check(
            "skirt_swings_clear_when_raised",
            skirt_plate_aabb is not None and skirt_plate_aabb[1][2] > 0.22,
            details=f"skirt plate AABB in raised pose: {skirt_plate_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
