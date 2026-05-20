from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


OPEN_ANGLE = 0.48
BASE_LENGTH = 0.34
BASE_WIDTH = 0.24
BASE_THICKNESS = 0.006
PLATFORM_LENGTH = 0.29
PLATFORM_WIDTH = 0.235
PLATFORM_THICKNESS = 0.006
PLATFORM_FRONT_OFFSET = 0.010
HINGE_X = -0.125
HINGE_Z = 0.014
REAR_PIVOT_X = 0.145
REAR_PIVOT_Z = 0.020


def _rounded_plate(length: float, width: float, thickness: float, *, slots: tuple[tuple[float, float, float, float], ...] = ()) -> cq.Workplane:
    plate = cq.Workplane("XY").box(length, width, thickness)
    plate = plate.edges("|Z").fillet(0.012)
    for x, y, slot_length, slot_width in slots:
        plate = plate.faces(">Z").workplane().center(x, y).rect(slot_length, slot_width).cutThruAll()
    return plate


def _panel_point(distance: float, normal_offset: float = 0.0) -> tuple[float, float]:
    """Return (x, z) in the platform part frame for a point on the tilted panel."""
    c = math.cos(OPEN_ANGLE)
    s = math.sin(OPEN_ANGLE)
    return distance * c - normal_offset * s, distance * s + normal_offset * c


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_laptop_stand")

    anodized = model.material("dark_anodized_aluminum", color=(0.10, 0.11, 0.12, 1.0))
    satin = model.material("satin_aluminum_edges", color=(0.55, 0.58, 0.58, 1.0))
    steel = model.material("brushed_steel_pins", color=(0.78, 0.76, 0.70, 1.0))
    rubber = model.material("matte_black_rubber", color=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_plate(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS), "base_panel"),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=anodized,
        name="base_panel",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.260),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_pin",
    )
    for i, y in enumerate((-0.112, 0.112)):
        base.visual(
            Box((0.020, 0.014, 0.020)),
            origin=Origin(xyz=(HINGE_X - 0.010, y, 0.0095)),
            material=satin,
            name=f"front_hinge_ear_{i}",
        )

    base.visual(
        Cylinder(radius=0.005, length=0.085),
        origin=Origin(xyz=(REAR_PIVOT_X, 0.0, REAR_PIVOT_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_pin",
    )
    for i, y in enumerate((-0.033, 0.033)):
        base.visual(
            Box((0.026, 0.010, 0.034)),
            origin=Origin(xyz=(REAR_PIVOT_X, y, 0.017)),
            material=satin,
            name=f"rear_clevis_ear_{i}",
        )
    for i, x in enumerate((0.050, 0.080, 0.110, 0.140)):
        base.visual(
            Box((0.012, 0.150, 0.006)),
            origin=Origin(xyz=(x, 0.0, BASE_THICKNESS + 0.003)),
            material=rubber,
            name=f"height_detent_{i}",
        )

    platform = model.part("platform")
    platform_slots = tuple((0.010, y, 0.150, 0.014) for y in (-0.066, -0.022, 0.022, 0.066))
    panel_center = _panel_point(PLATFORM_FRONT_OFFSET + PLATFORM_LENGTH / 2.0, 0.0)
    platform.visual(
        mesh_from_cadquery(
            _rounded_plate(PLATFORM_LENGTH, PLATFORM_WIDTH, PLATFORM_THICKNESS, slots=platform_slots),
            "platform_panel",
        ),
        origin=Origin(xyz=(panel_center[0], 0.0, panel_center[1]), rpy=(0.0, -OPEN_ANGLE, 0.0)),
        material=anodized,
        name="platform_panel",
    )
    for i, y in enumerate((-0.070, 0.0, 0.070)):
        platform.visual(
            Cylinder(radius=0.0075, length=0.046),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin,
            name=f"front_sleeve_{i}",
        )
        leaf_x, leaf_z = _panel_point(0.012, 0.0)
        platform.visual(
            Box((0.012, 0.040, 0.004)),
            origin=Origin(xyz=(leaf_x, y, leaf_z), rpy=(0.0, -OPEN_ANGLE, 0.0)),
            material=satin,
            name=f"front_hinge_leaf_{i}",
        )

    for i, (d, y) in enumerate(((0.070, -0.070), (0.070, 0.070), (0.205, -0.070), (0.205, 0.070))):
        x, z = _panel_point(d, PLATFORM_THICKNESS / 2.0 + 0.0018)
        platform.visual(
            Box((0.052, 0.024, 0.0036)),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, -OPEN_ANGLE, 0.0)),
            material=rubber,
            name=f"rubber_pad_{i}",
        )
    lip_x, lip_z = _panel_point(0.030, PLATFORM_THICKNESS / 2.0 + 0.006)
    platform.visual(
        Box((0.014, 0.180, 0.012)),
        origin=Origin(xyz=(lip_x, 0.0, lip_z), rpy=(0.0, -OPEN_ANGLE, 0.0)),
        material=rubber,
        name="front_lip",
    )

    socket_x, socket_z = _panel_point(0.250, -(PLATFORM_THICKNESS / 2.0 + 0.012))
    platform.visual(
        Cylinder(radius=0.009, length=0.086),
        origin=Origin(xyz=(socket_x, 0.0, socket_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_socket",
    )
    for i, y in enumerate((-0.030, 0.030)):
        platform.visual(
            Box((0.020, 0.010, 0.030)),
            origin=Origin(xyz=(socket_x, y, socket_z), rpy=(0.0, -OPEN_ANGLE, 0.0)),
            material=satin,
            name=f"rear_socket_ear_{i}",
        )

    support_leg = model.part("support_leg")
    upper_world = (HINGE_X + socket_x, 0.0, HINGE_Z + socket_z)
    leg_vec_x = upper_world[0] - REAR_PIVOT_X
    leg_vec_z = upper_world[2] - REAR_PIVOT_Z
    leg_length = math.hypot(leg_vec_x, leg_vec_z)
    leg_pitch = math.atan2(leg_vec_x, leg_vec_z)
    visible_leg_length = max(0.020, leg_length - 0.020)
    support_leg.visual(
        Box((0.012, 0.034, visible_leg_length)),
        origin=Origin(xyz=(leg_vec_x / 2.0, 0.0, leg_vec_z / 2.0), rpy=(0.0, leg_pitch, 0.0)),
        material=satin,
        name="flat_strut",
    )
    support_leg.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lower_bushing",
    )
    support_leg.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(leg_vec_x, 0.0, leg_vec_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="upper_bushing",
    )

    model.articulation(
        "front_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platform,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-OPEN_ANGLE, upper=0.25),
    )
    model.articulation(
        "rear_leg_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=support_leg,
        origin=Origin(xyz=(REAR_PIVOT_X, 0.0, REAR_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-0.90, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    support_leg = object_model.get_part("support_leg")
    front_hinge = object_model.get_articulation("front_hinge")
    rear_leg_pivot = object_model.get_articulation("rear_leg_pivot")

    for sleeve in ("front_sleeve_0", "front_sleeve_1", "front_sleeve_2"):
        ctx.allow_overlap(
            base,
            platform,
            elem_a="front_pin",
            elem_b=sleeve,
            reason="The stainless front hinge pin is intentionally captured inside the platform hinge sleeve.",
        )
        ctx.expect_overlap(
            base,
            platform,
            axes="y",
            elem_a="front_pin",
            elem_b=sleeve,
            min_overlap=0.035,
            name=f"{sleeve} is retained on the front hinge pin",
        )

    ctx.allow_overlap(
        base,
        support_leg,
        elem_a="rear_pin",
        elem_b="lower_bushing",
        reason="The rear support-leg pivot pin is intentionally nested through the lower bushing.",
    )
    ctx.expect_overlap(
        base,
        support_leg,
        axes="y",
        elem_a="rear_pin",
        elem_b="lower_bushing",
        min_overlap=0.040,
        name="lower support-leg bushing surrounds the base pivot pin",
    )

    ctx.allow_overlap(
        platform,
        support_leg,
        elem_a="rear_socket",
        elem_b="upper_bushing",
        reason="The upper bushing is intentionally seated in the platform socket to show the propping pivot.",
    )
    ctx.expect_overlap(
        platform,
        support_leg,
        axes="y",
        elem_a="rear_socket",
        elem_b="upper_bushing",
        min_overlap=0.045,
        name="upper support-leg bushing is captured in the platform socket",
    )

    ctx.expect_gap(
        platform,
        base,
        axis="z",
        positive_elem="platform_panel",
        negative_elem="base_panel",
        min_gap=0.004,
        name="working platform clears the thin base panel",
    )

    with ctx.pose({front_hinge: -OPEN_ANGLE}):
        folded_aabb = ctx.part_element_world_aabb(platform, elem="platform_panel")
    with ctx.pose({front_hinge: 0.20}):
        raised_aabb = ctx.part_element_world_aabb(platform, elem="platform_panel")
    ctx.check(
        "front hinge raises the rear of the platform",
        folded_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > folded_aabb[1][2] + 0.045,
        details=f"folded={folded_aabb}, raised={raised_aabb}",
    )

    rest_leg_aabb = ctx.part_element_world_aabb(support_leg, elem="upper_bushing")
    with ctx.pose({rear_leg_pivot: 0.35}):
        swung_leg_aabb = ctx.part_element_world_aabb(support_leg, elem="upper_bushing")
    ctx.check(
        "rear leg pivot swings the upper support bushing",
        rest_leg_aabb is not None
        and swung_leg_aabb is not None
        and abs(swung_leg_aabb[1][0] - rest_leg_aabb[1][0]) > 0.020,
        details=f"rest={rest_leg_aabb}, swung={swung_leg_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
