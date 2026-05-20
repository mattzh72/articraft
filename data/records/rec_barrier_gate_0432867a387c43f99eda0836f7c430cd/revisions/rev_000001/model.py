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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_rise_rod_blocker")

    galvanized = model.material("galvanized", rgba=(0.61, 0.63, 0.66, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    warning_red = model.material("warning_red", rgba=(0.77, 0.12, 0.10, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.94, 0.95, 0.93, 1.0))
    asphalt_grey = model.material("asphalt_grey", rgba=(0.36, 0.36, 0.37, 1.0))

    sleeve_center = (0.085, 0.0)

    road_flange = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.155, segments=72),
            [_circle_profile(0.026, center=sleeve_center, segments=48)],
            0.022,
            center=True,
        ),
        "road_flange_plate",
    )
    guide_sleeve = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.034, center=sleeve_center, segments=56),
            [_circle_profile(0.0245, center=sleeve_center, segments=48)],
            0.090,
            center=True,
        ),
        "guide_sleeve_shell",
    )
    socket_tube = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.046, center=sleeve_center, segments=56),
            [_circle_profile(0.0265, center=sleeve_center, segments=48)],
            0.300,
            center=True,
        ),
        "socket_tube_shell",
    )

    base = model.part("base")
    base.visual(
        road_flange,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=galvanized,
        name="road_flange",
    )
    base.visual(
        guide_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=brushed_steel,
        name="guide_sleeve",
    )
    base.visual(
        socket_tube,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=galvanized,
        name="socket_tube",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.070),
        origin=Origin(xyz=(-0.040, 0.0, 0.035)),
        material=galvanized,
        name="post_base_collar",
    )
    base.visual(
        Cylinder(radius=0.063, length=0.860),
        origin=Origin(xyz=(-0.040, 0.0, 0.430)),
        material=warning_red,
        name="post_body",
    )
    base.visual(
        Sphere(radius=0.066),
        origin=Origin(xyz=(-0.040, 0.0, 0.860)),
        material=warning_red,
        name="post_top_dome",
    )
    base.visual(
        Cylinder(radius=0.0645, length=0.120),
        origin=Origin(xyz=(-0.040, 0.0, 0.540)),
        material=reflector_white,
        name="reflective_band",
    )

    bolt_radius = 0.010
    for index, angle in enumerate((0.55, 2.10, 3.55, 5.15)):
        base.visual(
            Cylinder(radius=bolt_radius, length=0.018),
            origin=Origin(
                xyz=(0.112 * math.cos(angle), 0.112 * math.sin(angle), 0.009),
            ),
            material=brushed_steel,
            name=f"anchor_bolt_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.98)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    rising_rod = model.part("rising_rod")
    rising_rod.visual(
        Cylinder(radius=0.020, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="rod_shaft",
    )
    rising_rod.visual(
        Cylinder(radius=0.0215, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=warning_red,
        name="rod_warning_band",
    )
    rising_rod.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=galvanized,
        name="rest_collar",
    )
    rising_rod.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=brushed_steel,
        name="rod_cap",
    )
    rising_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.580),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    lift_joint = model.articulation(
        "rod_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=rising_rod,
        origin=Origin(xyz=(sleeve_center[0], sleeve_center[1], 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.16,
            lower=0.0,
            upper=0.220,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    rising_rod = object_model.get_part("rising_rod")
    rod_lift = object_model.get_articulation("rod_lift")

    ctx.check("base part exists", base is not None)
    ctx.check("rising rod part exists", rising_rod is not None)
    ctx.check(
        "rod lift is vertical prismatic",
        rod_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(value, 6) for value in rod_lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={rod_lift.articulation_type}, axis={rod_lift.axis}",
    )

    with ctx.pose({rod_lift: 0.0}):
        ctx.expect_contact(
            rising_rod,
            base,
            elem_a="rest_collar",
            elem_b="guide_sleeve",
            contact_tol=0.001,
            name="rest collar seats on the guide sleeve",
        )
        ctx.expect_overlap(
            rising_rod,
            base,
            axes="xy",
            elem_a="rod_shaft",
            elem_b="guide_sleeve",
            min_overlap=0.040,
            name="rest rod stays centered in the guide sleeve",
        )
        ctx.expect_overlap(
            rising_rod,
            base,
            axes="z",
            elem_a="rod_shaft",
            elem_b="socket_tube",
            min_overlap=0.030,
            name="rest rod remains inserted in the lower socket",
        )

    rest_position = ctx.part_world_position(rising_rod)
    with ctx.pose({rod_lift: 0.220}):
        ctx.expect_overlap(
            rising_rod,
            base,
            axes="xy",
            elem_a="rod_shaft",
            elem_b="guide_sleeve",
            min_overlap=0.040,
            name="extended rod stays centered in the guide sleeve",
        )
        ctx.expect_overlap(
            rising_rod,
            base,
            axes="z",
            elem_a="rod_shaft",
            elem_b="socket_tube",
            min_overlap=0.030,
            name="extended rod keeps retained insertion in the socket",
        )
        raised_position = ctx.part_world_position(rising_rod)

    ctx.check(
        "rod rises upward at positive extension",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.18,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
