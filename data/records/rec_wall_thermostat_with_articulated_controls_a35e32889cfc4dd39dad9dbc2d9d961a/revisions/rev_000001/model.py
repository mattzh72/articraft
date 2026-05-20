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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_plate_mesh(name: str, width: float, height: float, radius: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius, corner_segments=8),
            thickness,
            cap=True,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_large_dial")

    wall_white = model.material("wall_white", rgba=(0.95, 0.95, 0.93, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.83, 0.84, 0.82, 1.0))
    dial_graphite = model.material("dial_graphite", rgba=(0.21, 0.23, 0.25, 1.0))
    dial_silver = model.material("dial_silver", rgba=(0.77, 0.79, 0.80, 1.0))
    accent_black = model.material("accent_black", rgba=(0.08, 0.09, 0.10, 1.0))

    wall_plate_mesh = _rounded_plate_mesh("thermostat_wall_plate", 0.132, 0.132, 0.016, 0.003)
    body_mesh = _rounded_plate_mesh("thermostat_body_shell", 0.110, 0.110, 0.020, 0.016)

    housing = model.part("housing")
    housing.visual(
        wall_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=wall_white,
        name="wall_plate",
    )
    housing.visual(
        body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=housing_gray,
        name="body_shell",
    )
    housing.visual(
        Cylinder(radius=0.048, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=housing_gray,
        name="front_bezel",
    )
    housing.visual(
        Cylinder(radius=0.020, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
        material=dial_silver,
        name="bearing_collar",
    )
    housing.visual(
        Box((0.004, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.046, 0.023)),
        material=accent_black,
        name="index_mark",
    )
    housing.visual(
        Box((0.018, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, -0.050, 0.003)),
        material=wall_white,
        name="cable_cover_lip",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.132, 0.132, 0.029)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dial_silver,
        name="rear_hub",
    )
    dial.visual(
        Cylinder(radius=0.029, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dial_silver,
        name="inner_shoulder",
    )
    dial.visual(
        Cylinder(radius=0.041, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dial_graphite,
        name="outer_ring",
    )
    dial.visual(
        Cylinder(radius=0.036, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dial_graphite,
        name="front_lip",
    )
    dial.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dial_silver,
        name="center_cap",
    )
    dial.visual(
        Box((0.006, 0.015, 0.0015)),
        origin=Origin(xyz=(0.0, 0.027, 0.02075)),
        material=accent_black,
        name="dial_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.021),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    dial = object_model.get_part("dial")
    dial_spin = object_model.get_articulation("dial_spin")

    ctx.expect_origin_distance(
        housing,
        dial,
        axes="xy",
        max_dist=0.001,
        name="dial stays centered on the housing",
    )
    ctx.expect_overlap(
        dial,
        housing,
        axes="xy",
        min_overlap=0.080,
        elem_a="outer_ring",
        elem_b="front_bezel",
        name="dial ring covers the central bezel",
    )
    ctx.expect_gap(
        dial,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rear_hub",
        negative_elem="bearing_collar",
        name="dial hub seats against the bearing collar",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_spin: 1.8 * math.pi}):
        turned_pos = ctx.part_world_position(dial)
        ctx.expect_origin_distance(
            housing,
            dial,
            axes="xy",
            max_dist=0.001,
            name="dial remains coaxial while turned",
        )

    ctx.check(
        "dial spins in place without translating",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
