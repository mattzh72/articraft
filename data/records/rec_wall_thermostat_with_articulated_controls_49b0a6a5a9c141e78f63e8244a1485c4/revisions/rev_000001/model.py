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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_large_dial")

    plate_white = model.material("plate_white", rgba=(0.95, 0.95, 0.93, 1.0))
    housing_white = model.material("housing_white", rgba=(0.90, 0.90, 0.88, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    soft_shadow = model.material("soft_shadow", rgba=(0.72, 0.73, 0.75, 1.0))

    wall_plate = model.part("wall_plate")
    plate_profile = rounded_rect_profile(0.112, 0.150, 0.012, corner_segments=10)
    plate_mesh = mesh_from_geometry(ExtrudeGeometry(plate_profile, 0.007), "wall_plate")
    wall_plate.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=plate_white,
        name="plate",
    )
    wall_plate.visual(
        Cylinder(radius=0.041, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=housing_white,
        name="body_shell",
    )
    wall_plate.visual(
        Cylinder(radius=0.033, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0345)),
        material=soft_shadow,
        name="face_disc",
    )
    wall_plate.visual(
        Cylinder(radius=0.039, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=housing_white,
        name="body_flange",
    )
    wall_plate.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=soft_shadow,
        name="spindle",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.112, 0.150, 0.036)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    dial = model.part("dial")
    dial_ring = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.050, 0.000), (0.050, 0.010)],
            inner_profile=[(0.0435, 0.0012), (0.0435, 0.0088)],
            segments=56,
        ),
        "dial_ring",
    )
    dial.visual(
        dial_ring,
        material=graphite,
        name="dial_ring",
    )
    dial.visual(
        Box((0.096, 0.007, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=graphite,
        name="dial_bar",
    )
    dial.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=graphite,
        name="dial_hub",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.012)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=wall_plate,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_plate = object_model.get_part("wall_plate")
    dial = object_model.get_part("dial")
    dial_spin = object_model.get_articulation("dial_spin")

    def _span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.expect_contact(
        dial,
        wall_plate,
        elem_a="dial_hub",
        elem_b="spindle",
        name="dial hub is supported by the center spindle",
    )
    ctx.expect_gap(
        dial,
        wall_plate,
        axis="z",
        positive_elem="dial_bar",
        negative_elem="face_disc",
        min_gap=0.0005,
        max_gap=0.0025,
        name="dial bar sits slightly proud of the thermostat face",
    )
    ctx.expect_within(
        dial,
        wall_plate,
        axes="xy",
        inner_elem="dial_ring",
        outer_elem="plate",
        margin=0.0,
        name="dial ring stays within the wall plate footprint",
    )

    rest_bar_aabb = ctx.part_element_world_aabb(dial, elem="dial_bar")
    with ctx.pose({dial_spin: math.pi / 2.0}):
        quarter_turn_bar_aabb = ctx.part_element_world_aabb(dial, elem="dial_bar")

    rest_x = _span(rest_bar_aabb, 0)
    rest_y = _span(rest_bar_aabb, 1)
    quarter_x = _span(quarter_turn_bar_aabb, 0)
    quarter_y = _span(quarter_turn_bar_aabb, 1)
    ctx.check(
        "dial bar rotates from horizontal to vertical",
        rest_x is not None
        and rest_y is not None
        and quarter_x is not None
        and quarter_y is not None
        and rest_x > rest_y * 5.0
        and quarter_y > quarter_x * 5.0,
        details=(
            f"rest_spans=({rest_x}, {rest_y}), "
            f"quarter_turn_spans=({quarter_x}, {quarter_y})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
