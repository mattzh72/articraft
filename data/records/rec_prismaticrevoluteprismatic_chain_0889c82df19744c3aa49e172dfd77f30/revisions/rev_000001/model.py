from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


RAIL_LENGTH = 0.62
RAIL_BASE_WIDTH = 0.104
RAIL_BASE_THICK = 0.016
GUIDE_LENGTH = 0.56
GUIDE_WIDTH = 0.014
GUIDE_HEIGHT = 0.032
GUIDE_BAR_OFFSET = 0.021

CARRIAGE_LENGTH = 0.150
CARRIAGE_WIDTH = 0.124
CARRIAGE_HEIGHT = 0.076

PIVOT_Y = 0.030
PIVOT_Z = 0.138

NOSE_JOINT_Y = 0.024
NOSE_JOINT_Z = 0.000

SLIDE_TRAVEL = 0.175
BRACKET_SWIVEL = 0.55
NOSE_TRAVEL = 0.060


def _box(size: tuple[float, float, float], center: tuple[float, float, float] = (0.0, 0.0, 0.0)):
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, True)).translate(center)


def _cylinder(
    radius: float,
    length: float,
    axis: str,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    shape = cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))
    if axis == "x":
        shape = shape.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    elif axis == "y":
        shape = shape.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    elif axis != "z":
        raise ValueError(f"Unsupported axis: {axis}")
    return shape.translate(center)


def _make_base_rail_shape():
    rail = _box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_BASE_THICK), (0.0, 0.0, -0.022))
    rail = rail.union(_box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT), (0.0, GUIDE_BAR_OFFSET, 0.002)))
    rail = rail.union(_box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT), (0.0, -GUIDE_BAR_OFFSET, 0.002)))

    rail = rail.union(_box((0.024, 0.068, 0.018), (-0.287, 0.0, 0.019)))
    rail = rail.union(_box((0.024, 0.068, 0.018), (0.287, 0.0, 0.019)))
    rail = rail.union(_box((0.050, 0.020, 0.012), (-0.287, 0.0, -0.004)))
    rail = rail.union(_box((0.050, 0.020, 0.012), (0.287, 0.0, -0.004)))
    return rail


def _make_root_carriage_shape():
    carriage = _box((CARRIAGE_LENGTH, 0.118, 0.052), (0.0, 0.0, 0.054))
    carriage = carriage.cut(_box((0.126, 0.030, 0.022), (0.0, 0.0, 0.034)))
    carriage = carriage.cut(_box((0.124, 0.018, 0.034), (0.0, GUIDE_BAR_OFFSET, 0.028)))
    carriage = carriage.cut(_box((0.124, 0.018, 0.034), (0.0, -GUIDE_BAR_OFFSET, 0.028)))
    carriage = carriage.cut(_box((0.092, 0.042, 0.014), (0.0, -0.018, 0.077)))

    for x_pos in (-0.046, 0.046):
        for y_pos in (-GUIDE_BAR_OFFSET, GUIDE_BAR_OFFSET):
            carriage = carriage.union(_box((0.020, 0.016, 0.010), (x_pos, y_pos, 0.023)))

    carriage = carriage.union(_box((0.092, 0.010, 0.010), (0.0, 0.033, 0.043)))
    carriage = carriage.union(_box((0.100, 0.022, 0.014), (0.0, -0.028, 0.079)))
    carriage = carriage.union(_box((0.020, 0.026, 0.030), (-0.026, PIVOT_Y, 0.079)))
    carriage = carriage.union(_box((0.020, 0.026, 0.030), (0.026, PIVOT_Y, 0.079)))
    carriage = carriage.union(_box((0.072, 0.014, 0.008), (0.0, PIVOT_Y, 0.090)))
    carriage = carriage.union(_cylinder(0.016, 0.008, "z", (0.0, PIVOT_Y, 0.090)))

    return carriage


def _make_middle_bracket_shape():
    bracket = _cylinder(0.010, 0.030, "x", (0.0, 0.0, 0.0))
    bracket = bracket.union(_box((0.018, 0.022, 0.020), (0.0, 0.011, 0.0)))
    bracket = bracket.union(_box((0.024, 0.020, 0.026), (0.0, 0.028, 0.0)))

    bracket = bracket.union(_box((0.006, 0.090, 0.032), (-0.012, 0.072, 0.0)))
    bracket = bracket.union(_box((0.006, 0.090, 0.032), (0.012, 0.072, 0.0)))
    bracket = bracket.union(_box((0.018, 0.090, 0.006), (0.0, 0.072, 0.013)))
    bracket = bracket.union(_box((0.018, 0.090, 0.006), (0.0, 0.072, -0.013)))

    bracket = bracket.union(_box((0.026, 0.024, 0.032), (0.0, 0.104, 0.0)))
    bracket = bracket.cut(_box((0.020, 0.020, 0.026), (0.0, 0.104, 0.0)))
    bracket = bracket.union(_box((0.020, 0.008, 0.008), (0.0, 0.126, 0.013)))
    bracket = bracket.union(_box((0.020, 0.008, 0.008), (0.0, 0.126, -0.013)))

    bracket = bracket.cut(_cylinder(0.0096, 0.148, "y", (0.0, 0.074, 0.0)))

    return bracket


def _make_tool_nose_shape():
    nose = _cylinder(0.0118, 0.172, "y", (0.0, 0.086, 0.0))
    nose = nose.union(_cylinder(0.0165, 0.006, "y", (0.0, -0.003, 0.0)))
    nose = nose.union(_cylinder(0.0128, 0.012, "y", (0.0, 0.028, 0.0)))
    nose = nose.union(_cylinder(0.0160, 0.012, "y", (0.0, 0.126, 0.0)))
    nose = nose.union(_box((0.024, 0.036, 0.024), (0.0, 0.188, 0.0)))
    nose = nose.union(_cylinder(0.0085, 0.022, "y", (0.0, 0.217, 0.0)))
    return nose


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_positioning_module")

    model.material("rail_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("carriage_alloy", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("bracket_graphite", rgba=(0.19, 0.22, 0.26, 1.0))
    model.material("nose_steel", rgba=(0.68, 0.70, 0.73, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material="rail_steel",
        name="base_plate",
    )
    base_rail.visual(
        Box((GUIDE_LENGTH, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, GUIDE_BAR_OFFSET, -0.001)),
        material="rail_steel",
        name="left_guide",
    )
    base_rail.visual(
        Box((GUIDE_LENGTH, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -GUIDE_BAR_OFFSET, -0.001)),
        material="rail_steel",
        name="right_guide",
    )
    base_rail.visual(
        Box((0.018, 0.060, 0.034)),
        origin=Origin(xyz=(-0.284, 0.0, 0.001)),
        material="rail_steel",
        name="left_hard_stop",
    )
    base_rail.visual(
        Box((0.018, 0.060, 0.034)),
        origin=Origin(xyz=(0.284, 0.0, 0.001)),
        material="rail_steel",
        name="right_hard_stop",
    )
    base_rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, 0.040)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    root_carriage = model.part("root_carriage")
    root_carriage.visual(
        Box((CARRIAGE_LENGTH, 0.094, 0.046)),
        origin=Origin(xyz=(0.0, -0.010, 0.036)),
        material="carriage_alloy",
        name="carriage_body",
    )
    root_carriage.visual(
        Box((0.100, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, -0.032, 0.066)),
        material="carriage_alloy",
        name="rear_block",
    )
    root_carriage.visual(
        Box((0.060, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.012, 0.067)),
        material="carriage_alloy",
        name="front_bridge",
    )
    root_carriage.visual(
        Box((0.016, 0.024, 0.108)),
        origin=Origin(xyz=(-0.024, PIVOT_Y, 0.104)),
        material="carriage_alloy",
        name="left_cheek",
    )
    root_carriage.visual(
        Box((0.016, 0.024, 0.108)),
        origin=Origin(xyz=(0.024, PIVOT_Y, 0.104)),
        material="carriage_alloy",
        name="right_cheek",
    )
    root_carriage.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(-0.036, PIVOT_Y, PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="carriage_alloy",
        name="left_pivot_cap",
    )
    root_carriage.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.036, PIVOT_Y, PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="carriage_alloy",
        name="right_pivot_cap",
    )
    root_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, 0.094, 0.108)),
        mass=1.8,
        origin=Origin(xyz=(0.0, -0.004, 0.072)),
    )

    middle_bracket = model.part("middle_bracket")
    middle_bracket.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="bracket_graphite",
        name="pivot_barrel",
    )
    middle_bracket.visual(
        Box((0.018, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material="bracket_graphite",
        name="pivot_neck",
    )
    middle_bracket.visual(
        Box((0.006, 0.104, 0.030)),
        origin=Origin(xyz=(-0.013, 0.064, 0.0)),
        material="bracket_graphite",
        name="left_guide_wall",
    )
    middle_bracket.visual(
        Box((0.006, 0.104, 0.030)),
        origin=Origin(xyz=(0.013, 0.064, 0.0)),
        material="bracket_graphite",
        name="right_guide_wall",
    )
    middle_bracket.visual(
        Box((0.020, 0.104, 0.006)),
        origin=Origin(xyz=(0.0, 0.064, 0.013)),
        material="bracket_graphite",
        name="top_guide_plate",
    )
    middle_bracket.visual(
        Box((0.020, 0.104, 0.006)),
        origin=Origin(xyz=(0.0, 0.064, -0.013)),
        material="bracket_graphite",
        name="bottom_guide_plate",
    )
    middle_bracket.visual(
        Box((0.024, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.118, 0.013)),
        material="bracket_graphite",
        name="front_stop_top",
    )
    middle_bracket.visual(
        Box((0.024, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.118, -0.013)),
        material="bracket_graphite",
        name="front_stop_bottom",
    )
    middle_bracket.inertial = Inertial.from_geometry(
        Box((0.040, 0.130, 0.054)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.068, 0.008)),
    )

    tool_nose = model.part("tool_nose")
    tool_nose.visual(
        Cylinder(radius=0.0085, length=0.174),
        origin=Origin(xyz=(0.0, 0.087, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="nose_steel",
        name="nose_rod",
    )
    tool_nose.visual(
        Cylinder(radius=0.0100, length=0.006),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="nose_steel",
        name="rear_collar",
    )
    tool_nose.visual(
        Cylinder(radius=0.0092, length=0.010),
        origin=Origin(xyz=(0.0, 0.110, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="nose_steel",
        name="guide_ring",
    )
    tool_nose.visual(
        Box((0.022, 0.032, 0.022)),
        origin=Origin(xyz=(0.0, 0.176, 0.0)),
        material="nose_steel",
        name="tool_head",
    )
    tool_nose.visual(
        Cylinder(radius=0.0075, length=0.020),
        origin=Origin(xyz=(0.0, 0.202, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="nose_steel",
        name="tool_tip",
    )
    tool_nose.inertial = Inertial.from_geometry(
        Box((0.026, 0.222, 0.026)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.100, 0.0)),
    )

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=root_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=650.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "bracket_yaw",
        ArticulationType.REVOLUTE,
        parent=root_carriage,
        child=middle_bracket,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BRACKET_SWIVEL,
            upper=BRACKET_SWIVEL,
            effort=65.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=middle_bracket,
        child=tool_nose,
        origin=Origin(xyz=(0.0, NOSE_JOINT_Y, NOSE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=NOSE_TRAVEL,
            effort=220.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_rail = object_model.get_part("base_rail")
    root_carriage = object_model.get_part("root_carriage")
    middle_bracket = object_model.get_part("middle_bracket")
    tool_nose = object_model.get_part("tool_nose")
    rail_slide = object_model.get_articulation("rail_slide")
    bracket_yaw = object_model.get_articulation("bracket_yaw")
    nose_slide = object_model.get_articulation("nose_slide")

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

    ctx.expect_contact(root_carriage, base_rail, name="carriage_supported_on_rail")
    ctx.expect_contact(middle_bracket, root_carriage, name="bracket_supported_on_carriage")
    ctx.expect_contact(tool_nose, middle_bracket, name="nose_seated_in_rear_sleeve")
    bracket_box = ctx.part_world_aabb(middle_bracket)
    nose_box = ctx.part_world_aabb(tool_nose)
    projection_margin = None
    if bracket_box is not None and nose_box is not None:
        projection_margin = nose_box[1][1] - bracket_box[1][1]
    ctx.check(
        "tool_projects_from_far_side_of_bracket",
        projection_margin is not None and projection_margin >= 0.070,
        details=f"projection_margin_y={projection_margin}",
    )

    with ctx.pose({rail_slide: SLIDE_TRAVEL, bracket_yaw: BRACKET_SWIVEL, nose_slide: NOSE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_at_forward_extended_pose")

    with ctx.pose({rail_slide: -SLIDE_TRAVEL, bracket_yaw: -BRACKET_SWIVEL, nose_slide: NOSE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_at_reverse_extended_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
