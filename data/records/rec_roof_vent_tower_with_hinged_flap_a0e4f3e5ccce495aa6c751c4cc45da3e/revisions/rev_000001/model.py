from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    flashing = model.material("flashing", rgba=(0.44, 0.46, 0.49, 1.0))

    base_x = 0.36
    base_y = 0.52
    base_h = 0.12

    tower_x = 0.32
    tower_y = 0.42
    tower_h = 0.84
    wall = 0.025

    tower_bottom = base_h
    tower_top = tower_bottom + tower_h
    front_face_x = tower_x * 0.5 - wall * 0.5
    face_outer_x = tower_x * 0.5

    opening_width = 0.30
    opening_bottom = 0.30
    opening_top = 0.84
    opening_height = opening_top - opening_bottom

    trim_thickness = 0.025
    trim_proud = 0.011
    trim_center_x = front_face_x + trim_proud
    trim_front_x = trim_center_x + trim_thickness * 0.5

    housing = model.part("housing")
    housing.visual(
        Box((base_x, base_y, base_h)),
        origin=Origin(xyz=(0.0, 0.0, base_h * 0.5)),
        material=flashing,
        name="roof_curb",
    )
    housing.visual(
        Box((tower_x, tower_y, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, base_h + 0.015)),
        material=dark_steel,
        name="curb_cap",
    )
    housing.visual(
        Box((wall, tower_y, tower_h + 0.01)),
        origin=Origin(
            xyz=(-(tower_x * 0.5 - wall * 0.5), 0.0, tower_bottom + tower_h * 0.5 - 0.005)
        ),
        material=galvanized,
        name="rear_wall",
    )
    housing.visual(
        Box((tower_x, wall, tower_h + 0.01)),
        origin=Origin(
            xyz=(0.0, tower_y * 0.5 - wall * 0.5, tower_bottom + tower_h * 0.5 - 0.005)
        ),
        material=galvanized,
        name="left_wall",
    )
    housing.visual(
        Box((tower_x, wall, tower_h + 0.01)),
        origin=Origin(
            xyz=(0.0, -(tower_y * 0.5 - wall * 0.5), tower_bottom + tower_h * 0.5 - 0.005)
        ),
        material=galvanized,
        name="right_wall",
    )
    housing.visual(
        Box((wall, tower_y, 0.20)),
        origin=Origin(xyz=(front_face_x, 0.0, 0.21)),
        material=galvanized,
        name="lower_front_wall",
    )
    housing.visual(
        Box((wall, tower_y, 0.14)),
        origin=Origin(xyz=(front_face_x, 0.0, 0.89)),
        material=galvanized,
        name="upper_front_wall",
    )
    housing.visual(
        Box((wall, 0.058, opening_height + 0.06)),
        origin=Origin(xyz=(front_face_x, 0.181, 0.57)),
        material=galvanized,
        name="left_jamb_wall",
    )
    housing.visual(
        Box((wall, 0.058, opening_height + 0.06)),
        origin=Origin(xyz=(front_face_x, -0.181, 0.57)),
        material=galvanized,
        name="right_jamb_wall",
    )
    housing.visual(
        Box((0.36, 0.46, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, tower_top - 0.004)),
        material=dark_steel,
        name="rain_cap",
    )
    housing.visual(
        Box((trim_thickness, 0.35, 0.04)),
        origin=Origin(xyz=(trim_center_x, 0.0, 0.86)),
        material=dark_steel,
        name="outlet_top_trim",
    )
    housing.visual(
        Box((trim_thickness, 0.35, 0.04)),
        origin=Origin(xyz=(trim_center_x, 0.0, 0.28)),
        material=dark_steel,
        name="outlet_sill_trim",
    )
    housing.visual(
        Box((trim_thickness, 0.028, 0.58)),
        origin=Origin(xyz=(trim_center_x, 0.161, 0.57)),
        material=dark_steel,
        name="left_outlet_trim",
    )
    housing.visual(
        Box((trim_thickness, 0.028, 0.58)),
        origin=Origin(xyz=(trim_center_x, -0.161, 0.57)),
        material=dark_steel,
        name="right_outlet_trim",
    )
    housing.visual(
        Box((0.12, 0.24, 0.02)),
        origin=Origin(xyz=(-(tower_x * 0.5) + 0.06, 0.0, tower_top - 0.018)),
        material=dark_steel,
        name="rear_cap_plate",
    )
    housing.inertial = Inertial.from_geometry(
        Box((base_x, base_y, tower_top)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, tower_top * 0.5)),
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.010, 0.335, 0.625)),
        origin=Origin(xyz=(0.005, 0.0, -0.3125)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.0075, length=0.315),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.018, 0.295, 0.045)),
        origin=Origin(xyz=(0.012, 0.0, -0.11)),
        material=dark_steel,
        name="mid_stiffener",
    )
    flap.visual(
        Box((0.014, 0.315, 0.022)),
        origin=Origin(xyz=(0.009, 0.0, -0.614)),
        material=dark_steel,
        name="drip_rail",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.030, 0.34, 0.63)),
        mass=4.5,
        origin=Origin(xyz=(0.012, 0.0, -0.30)),
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(trim_front_x, 0.0, 0.88)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("housing_to_flap")

    ctx.expect_gap(
        flap,
        housing,
        axis="x",
        positive_elem="flap_panel",
        negative_elem="outlet_top_trim",
        max_gap=0.003,
        max_penetration=0.0,
        name="flap closes against the outlet frame",
    )
    ctx.expect_overlap(
        flap,
        housing,
        axes="yz",
        elem_a="flap_panel",
        elem_b="outlet_top_trim",
        min_overlap=0.02,
        name="flap spans the framed outlet width at the hinge edge",
    )

    closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    closed_center = None
    open_center = None
    if closed_aabb is not None:
        closed_center = tuple((lo + hi) * 0.5 for lo, hi in zip(closed_aabb[0], closed_aabb[1]))
    if open_aabb is not None:
        open_center = tuple((lo + hi) * 0.5 for lo, hi in zip(open_aabb[0], open_aabb[1]))

    ctx.check(
        "flap swings outward and upward",
        closed_center is not None
        and open_center is not None
        and open_center[0] > closed_center[0] + 0.10
        and open_center[2] > closed_center[2] + 0.10,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
