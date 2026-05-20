from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ClevisBracketGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_axis_yaw_turntable")

    dark_cast = model.material("dark_cast_metal", rgba=(0.035, 0.038, 0.042, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.008, 0.008, 0.007, 1.0))
    blue_anodized = model.material("blue_anodized_bracket", rgba=(0.04, 0.18, 0.48, 1.0))
    brass = model.material("brass_bearing_cage", rgba=(0.78, 0.58, 0.23, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.24, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_cast,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.13, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark_cast,
        name="raised_plinth",
    )
    base.visual(
        Cylinder(radius=0.047, length=0.049),
        origin=Origin(xyz=(0.0, 0.0, 0.0595)),
        material=satin_steel,
        name="fixed_hub",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=satin_steel,
        name="lower_race",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.076, tube=0.008, radial_segments=18, tubular_segments=64),
            "bearing_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0765)),
        material=brass,
        name="bearing_ring",
    )
    base.visual(
        Cylinder(radius=0.090, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.08375)),
        material=satin_steel,
        name="thrust_washer",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.205, tube=0.007, radial_segments=14, tubular_segments=72),
            "base_outer_lip",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=black_rubber,
        name="outer_lip",
    )

    upper_plate = model.part("upper_plate")
    upper_plate.visual(
        Cylinder(radius=0.092, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_steel,
        name="rotating_race",
    )
    upper_plate.visual(
        Cylinder(radius=0.19, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=brushed_aluminum,
        name="round_plate",
    )
    upper_plate.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.181, tube=0.006, radial_segments=14, tubular_segments=72),
            "plate_grip_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=black_rubber,
        name="grip_ring",
    )
    upper_plate.visual(
        Cylinder(radius=0.056, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=satin_steel,
        name="hub_cap",
    )
    upper_plate.visual(
        Box((0.125, 0.085, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=blue_anodized,
        name="bracket_foot",
    )
    upper_plate.visual(
        mesh_from_geometry(
            ClevisBracketGeometry(
                (0.112, 0.070, 0.078),
                gap_width=0.054,
                bore_diameter=0.018,
                bore_center_z=0.054,
                base_thickness=0.012,
                corner_radius=0.004,
                center=False,
            ),
            "payload_clevis",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=blue_anodized,
        name="payload_clevis",
    )
    upper_plate.visual(
        Cylinder(radius=0.009, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="payload_pin",
    )

    model.articulation(
        "base_to_upper_plate",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.0845)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_plate = object_model.get_part("upper_plate")
    yaw = object_model.get_articulation("base_to_upper_plate")

    ctx.check(
        "upper plate has one vertical yaw joint",
        tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.expect_within(
        upper_plate,
        base,
        axes="xy",
        margin=0.0,
        elem_a="round_plate",
        elem_b="base_disc",
        name="rotating plate stays inside base footprint",
    )
    ctx.expect_gap(
        upper_plate,
        base,
        axis="z",
        positive_elem="rotating_race",
        negative_elem="thrust_washer",
        max_gap=0.0015,
        max_penetration=0.0,
        name="upper race is seated just above bearing stack",
    )
    ctx.expect_overlap(
        upper_plate,
        base,
        axes="xy",
        elem_a="rotating_race",
        elem_b="lower_race",
        min_overlap=0.08,
        name="bearing races are concentric in plan",
    )

    rest_origin = ctx.part_world_position(upper_plate)
    rest_clevis_aabb = ctx.part_element_world_aabb(upper_plate, elem="payload_clevis")
    with ctx.pose({yaw: pi / 2.0}):
        turned_origin = ctx.part_world_position(upper_plate)
        turned_clevis_aabb = ctx.part_element_world_aabb(upper_plate, elem="payload_clevis")

    def _aabb_size(aabb, index: int) -> float | None:
        if aabb is None:
            return None
        return float(aabb[1][index] - aabb[0][index])

    ctx.check(
        "yaw rotation keeps centered hub fixed",
        rest_origin is not None
        and turned_origin is not None
        and abs(rest_origin[0] - turned_origin[0]) < 1e-6
        and abs(rest_origin[1] - turned_origin[1]) < 1e-6
        and abs(rest_origin[2] - turned_origin[2]) < 1e-6,
        details=f"rest={rest_origin}, turned={turned_origin}",
    )
    ctx.check(
        "payload bracket visibly yaws with upper plate",
        rest_clevis_aabb is not None
        and turned_clevis_aabb is not None
        and abs(_aabb_size(rest_clevis_aabb, 0) - _aabb_size(turned_clevis_aabb, 1)) < 0.004
        and abs(_aabb_size(rest_clevis_aabb, 1) - _aabb_size(turned_clevis_aabb, 0)) < 0.004,
        details=f"rest={rest_clevis_aabb}, turned={turned_clevis_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
