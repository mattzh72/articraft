from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FLOOR_LENGTH = 0.48
FLOOR_DEPTH = 0.27
FLOOR_THICKNESS = 0.016
FOOT_LENGTH = 0.074
FOOT_DEPTH = 0.055
FOOT_HEIGHT = 0.014

SUPPORT_Y = 0.11
RAIL_DEPTH = 0.032
RAIL_HEIGHT = 0.032
BLOCK_WIDTH = 0.058
BLOCK_DEPTH = 0.034
PEDESTAL_HEIGHT = 0.06
SHAFT_CENTER_Z = 0.096
BEARING_BOSS_RADIUS = 0.028
HOUSING_BORE_RADIUS = 0.015

SHAFT_RADIUS = 0.011
JOURNAL_RADIUS = 0.0156
COLLAR_RADIUS = 0.023
COLLAR_THICKNESS = 0.006
SHAFT_BODY_LENGTH = 2.0 * (SUPPORT_Y + BLOCK_DEPTH / 2.0 + COLLAR_THICKNESS) + 0.008
JOURNAL_LENGTH = BLOCK_DEPTH + 0.008

STAGE1_Y = -0.042
STAGE2_Y = 0.042

INPUT_GEAR_RADIUS = 0.034
COUNTER_LARGE_RADIUS = 0.072
COUNTER_SMALL_RADIUS = 0.031
OUTPUT_GEAR_RADIUS = 0.061
GEAR_CENTER_GAP = 0.004

INPUT_X = -(INPUT_GEAR_RADIUS + COUNTER_LARGE_RADIUS + GEAR_CENTER_GAP)
COUNTER_X = 0.0
OUTPUT_X = COUNTER_SMALL_RADIUS + OUTPUT_GEAR_RADIUS + GEAR_CENTER_GAP

INPUT_GEAR_WIDTH = 0.018
COUNTER_LARGE_WIDTH = 0.022
COUNTER_SMALL_WIDTH = 0.017
OUTPUT_GEAR_WIDTH = 0.02


def cylinder_y(
    radius: float,
    length: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((center_x, center_y, center_z))
    )


def box_solid(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    base_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, False))
        .translate((center_x, center_y, base_z))
    )


def make_gear(
    *,
    outer_radius: float,
    width: float,
    teeth: int,
    hub_radius: float,
    hub_length: float,
    phase_deg: float = 0.0,
) -> cq.Workplane:
    root_radius = outer_radius * 0.82
    tooth_depth = outer_radius - root_radius + 0.004
    tooth_width = min((2.0 * pi * root_radius / teeth) * 0.55, outer_radius * 0.34)

    gear = (
        cq.Workplane("XY")
        .circle(root_radius)
        .extrude(width)
        .translate((0.0, 0.0, -width / 2.0))
    )
    gear = gear.union(
        cq.Workplane("XY")
        .circle(hub_radius)
        .extrude(hub_length)
        .translate((0.0, 0.0, -hub_length / 2.0))
    )

    tooth = (
        cq.Workplane("XY")
        .box(tooth_width, tooth_depth, width, centered=(True, False, True))
        .translate((0.0, root_radius - 0.002, 0.0))
    )
    for tooth_index in range(teeth):
        gear = gear.union(
            tooth.rotate(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                tooth_index * 360.0 / teeth + phase_deg,
            )
        )

    return gear.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)


def make_housing() -> cq.Workplane:
    housing = box_solid(FLOOR_LENGTH, FLOOR_DEPTH, FLOOR_THICKNESS)

    for foot_x in (-0.16, 0.16):
        for foot_y in (-0.085, 0.085):
            housing = housing.union(
                box_solid(
                    FOOT_LENGTH,
                    FOOT_DEPTH,
                    FOOT_HEIGHT,
                    center_x=foot_x,
                    center_y=foot_y,
                    base_z=-FOOT_HEIGHT,
                )
            )

    rail_length = OUTPUT_X - INPUT_X + 0.11
    for y_sign in (-1.0, 1.0):
        rail_y = y_sign * SUPPORT_Y
        housing = housing.union(
            box_solid(
                rail_length,
                RAIL_DEPTH,
                RAIL_HEIGHT,
                center_y=rail_y,
                base_z=FLOOR_THICKNESS,
            )
        )

        for x_pos in (INPUT_X, COUNTER_X, OUTPUT_X):
            housing = housing.union(
                box_solid(
                    BLOCK_WIDTH,
                    BLOCK_DEPTH,
                    PEDESTAL_HEIGHT,
                    center_x=x_pos,
                    center_y=rail_y,
                    base_z=FLOOR_THICKNESS,
                )
            )
            housing = housing.union(
                cylinder_y(
                    BEARING_BOSS_RADIUS,
                    BLOCK_DEPTH + 0.004,
                    center_x=x_pos,
                    center_y=rail_y,
                    center_z=SHAFT_CENTER_Z,
                )
            )

    for y_sign in (-1.0, 1.0):
        for x_pos in (INPUT_X, COUNTER_X, OUTPUT_X):
            housing = housing.cut(
                cylinder_y(
                    HOUSING_BORE_RADIUS,
                    BLOCK_DEPTH + 0.012,
                    center_x=x_pos,
                    center_y=y_sign * SUPPORT_Y,
                    center_z=SHAFT_CENTER_Z,
                )
            )

    return housing


def make_shaft_core() -> cq.Workplane:
    shaft = cylinder_y(SHAFT_RADIUS, SHAFT_BODY_LENGTH)
    for y_sign in (-1.0, 1.0):
        shaft = shaft.union(
            cylinder_y(
                JOURNAL_RADIUS,
                JOURNAL_LENGTH,
                center_y=y_sign * SUPPORT_Y,
            )
        )
        shaft = shaft.union(
            cylinder_y(
                COLLAR_RADIUS,
                COLLAR_THICKNESS,
                center_y=y_sign * (SUPPORT_Y + BLOCK_DEPTH / 2.0 + COLLAR_THICKNESS / 2.0),
            )
        )
    return shaft


def make_input_shaft() -> cq.Workplane:
    shaft = make_shaft_core()
    shaft = shaft.union(
        cylinder_y(
            0.013,
            0.032,
            center_y=SUPPORT_Y + BLOCK_DEPTH / 2.0 + COLLAR_THICKNESS + 0.016,
        )
    )
    shaft = shaft.union(
        make_gear(
            outer_radius=INPUT_GEAR_RADIUS,
            width=INPUT_GEAR_WIDTH,
            teeth=14,
            hub_radius=0.021,
            hub_length=0.028,
            phase_deg=8.0,
        ).translate((0.0, STAGE1_Y, 0.0))
    )
    return shaft


def make_countershaft() -> cq.Workplane:
    shaft = make_shaft_core()
    shaft = shaft.union(
        make_gear(
            outer_radius=COUNTER_LARGE_RADIUS,
            width=COUNTER_LARGE_WIDTH,
            teeth=26,
            hub_radius=0.027,
            hub_length=0.032,
            phase_deg=5.0,
        ).translate((0.0, STAGE1_Y, 0.0))
    )
    shaft = shaft.union(
        make_gear(
            outer_radius=COUNTER_SMALL_RADIUS,
            width=COUNTER_SMALL_WIDTH,
            teeth=12,
            hub_radius=0.02,
            hub_length=0.026,
            phase_deg=18.0,
        ).translate((0.0, STAGE2_Y, 0.0))
    )
    return shaft


def make_output_shaft() -> cq.Workplane:
    shaft = make_shaft_core()
    shaft = shaft.union(
        cylinder_y(
            0.016,
            0.044,
            center_y=-(SUPPORT_Y + BLOCK_DEPTH / 2.0 + COLLAR_THICKNESS + 0.022),
        )
    )
    shaft = shaft.union(
        make_gear(
            outer_radius=OUTPUT_GEAR_RADIUS,
            width=OUTPUT_GEAR_WIDTH,
            teeth=22,
            hub_radius=0.025,
            hub_length=0.032,
            phase_deg=11.0,
        ).translate((0.0, STAGE2_Y, 0.0))
    )
    return shaft


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_reduction_gearbox")

    model.material("housing_cast", rgba=(0.38, 0.42, 0.46, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(make_housing().val(), "gearbox_housing"),
        material="housing_cast",
        name="housing_body",
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        mesh_from_cadquery(make_input_shaft().val(), "input_shaft"),
        material="machined_steel",
        name="input_shaft_body",
    )

    countershaft = model.part("countershaft")
    countershaft.visual(
        mesh_from_cadquery(make_countershaft().val(), "countershaft"),
        material="machined_steel",
        name="countershaft_body",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        mesh_from_cadquery(make_output_shaft().val(), "output_shaft"),
        material="machined_steel",
        name="output_shaft_body",
    )

    shaft_limits = MotionLimits(effort=8.0, velocity=12.0, lower=-2.0 * pi, upper=2.0 * pi)

    model.articulation(
        "housing_to_input_shaft",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=input_shaft,
        origin=Origin(xyz=(INPUT_X, 0.0, SHAFT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=shaft_limits,
    )
    model.articulation(
        "housing_to_countershaft",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=countershaft,
        origin=Origin(xyz=(COUNTER_X, 0.0, SHAFT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=12.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )
    model.articulation(
        "housing_to_output_shaft",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_X, 0.0, SHAFT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=12.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    input_shaft = object_model.get_part("input_shaft")
    countershaft = object_model.get_part("countershaft")
    output_shaft = object_model.get_part("output_shaft")

    input_joint = object_model.get_articulation("housing_to_input_shaft")
    counter_joint = object_model.get_articulation("housing_to_countershaft")
    output_joint = object_model.get_articulation("housing_to_output_shaft")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    for shaft in (input_shaft, countershaft, output_shaft):
        ctx.allow_overlap(
            housing,
            shaft,
            reason="Hidden journal interference represents a supported bearing fit in the open gearbox housing.",
        )

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

    expected_parts = {"housing", "input_shaft", "countershaft", "output_shaft"}
    actual_parts = {part.name for part in object_model.parts}
    ctx.check(
        "gearbox_part_set",
        actual_parts == expected_parts,
        details=f"expected {sorted(expected_parts)}, got {sorted(actual_parts)}",
    )

    ctx.expect_origin_distance(
        input_shaft,
        countershaft,
        axes="yz",
        max_dist=1e-6,
        name="input_and_countershaft_are_parallel_and_level",
    )
    ctx.expect_origin_distance(
        countershaft,
        output_shaft,
        axes="yz",
        max_dist=1e-6,
        name="counter_and_output_are_parallel_and_level",
    )
    ctx.expect_origin_gap(
        countershaft,
        input_shaft,
        axis="x",
        min_gap=0.09,
        max_gap=0.13,
        name="stage1_shaft_spacing",
    )
    ctx.expect_origin_gap(
        output_shaft,
        countershaft,
        axis="x",
        min_gap=0.08,
        max_gap=0.11,
        name="stage2_shaft_spacing",
    )

    for joint in (input_joint, counter_joint, output_joint):
        ctx.check(
            f"{joint.name}_axis_parallel_to_shafts",
            tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0),
            details=f"joint axis was {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_is_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"joint type was {joint.articulation_type}",
        )

    with ctx.pose(
        {
            input_joint: 0.75,
            counter_joint: -0.55,
            output_joint: 1.1,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_offset_pose")
        ctx.expect_origin_distance(
            input_shaft,
            countershaft,
            axes="yz",
            max_dist=1e-6,
            name="shafts_remain_parallel_in_offset_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
