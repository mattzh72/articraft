from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_HEIGHT = 0.024
SUPPORT_TOP_Z = 0.104
MAIN_AXIS_X = 0.0
SECONDARY_AXIS_X = 0.230


def _frame_shape() -> cq.Workplane:
    base_radius = 0.200
    central_pedestal_radius = 0.056
    central_collar_radius = 0.074
    support_column_x = 0.142
    support_column_radius = 0.032
    support_foot_radius = 0.052
    arm_width = 0.052
    arm_bottom_z = 0.082
    arm_height = SUPPORT_TOP_Z - arm_bottom_z
    end_housing_radius = 0.038
    pedestal_height = SUPPORT_TOP_Z - BASE_HEIGHT

    frame = cq.Workplane("XY").circle(base_radius).extrude(BASE_HEIGHT)
    frame = frame.union(
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(central_pedestal_radius)
        .extrude(pedestal_height)
    )
    frame = frame.union(
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(central_collar_radius)
        .extrude(0.018)
    )
    frame = frame.union(
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .center(support_column_x, 0.0)
        .circle(support_column_radius)
        .extrude(pedestal_height)
    )
    frame = frame.union(
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .center(support_column_x, 0.0)
        .circle(support_foot_radius)
        .extrude(0.018)
    )
    frame = frame.union(
        cq.Workplane("XY")
        .workplane(offset=arm_bottom_z)
        .center((support_column_x + SECONDARY_AXIS_X) / 2.0, 0.0)
        .box(
            SECONDARY_AXIS_X - support_column_x,
            arm_width,
            arm_height,
            centered=(True, True, False),
        )
    )
    frame = frame.union(
        cq.Workplane("XY")
        .workplane(offset=arm_bottom_z - 0.018)
        .center(SECONDARY_AXIS_X, 0.0)
        .circle(end_housing_radius)
        .extrude(SUPPORT_TOP_Z - (arm_bottom_z - 0.018))
    )
    frame = frame.union(
        cq.Workplane("XZ")
        .polyline(
            [
                (support_column_x - 0.018, BASE_HEIGHT),
                (support_column_x + 0.010, BASE_HEIGHT),
                (support_column_x + 0.010, arm_bottom_z),
                (SECONDARY_AXIS_X - 0.020, arm_bottom_z),
            ]
        )
        .close()
        .extrude(0.018, both=True)
    )
    return frame


def _base_table_shape() -> cq.Workplane:
    hub_height = 0.014
    hub_radius = 0.044
    plate_height = 0.018
    plate_radius = 0.125
    top_surface_z = hub_height + plate_height

    table = cq.Workplane("XY").circle(hub_radius).extrude(hub_height)
    table = table.union(
        cq.Workplane("XY").workplane(offset=hub_height).circle(plate_radius).extrude(plate_height)
    )
    table = table.cut(
        cq.Workplane("XY")
        .workplane(offset=top_surface_z - 0.004)
        .circle(0.090)
        .extrude(0.004)
    )
    table = table.cut(
        cq.Workplane("XY").circle(0.016).extrude(top_surface_z + 0.002)
    )
    table = table.cut(
        cq.Workplane("XY")
        .workplane(offset=hub_height)
        .rect(0.060, 0.012)
        .extrude(0.004)
    )
    table = table.cut(
        cq.Workplane("XY")
        .workplane(offset=hub_height)
        .transformed(rotate=(0.0, 0.0, 60.0))
        .rect(0.060, 0.012)
        .extrude(0.004)
    )
    table = table.cut(
        cq.Workplane("XY")
        .workplane(offset=hub_height)
        .transformed(rotate=(0.0, 0.0, -60.0))
        .rect(0.060, 0.012)
        .extrude(0.004)
    )
    table = table.union(
        cq.Workplane("XY")
        .workplane(offset=top_surface_z)
        .center(0.108, 0.0)
        .box(0.034, 0.014, 0.010, centered=(True, True, False))
    )
    table = table.union(
        cq.Workplane("XY")
        .workplane(offset=top_surface_z + 0.010)
        .center(0.130, 0.0)
        .circle(0.008)
        .extrude(0.018)
    )
    return table


def _secondary_plate_shape() -> cq.Workplane:
    hub_height = 0.012
    hub_radius = 0.026
    plate_height = 0.014
    plate_radius = 0.065
    top_surface_z = hub_height + plate_height

    plate = cq.Workplane("XY").circle(hub_radius).extrude(hub_height)
    plate = plate.union(
        cq.Workplane("XY").workplane(offset=hub_height).circle(plate_radius).extrude(plate_height)
    )
    plate = plate.cut(
        cq.Workplane("XY")
        .workplane(offset=top_surface_z - 0.003)
        .circle(0.044)
        .extrude(0.003)
    )
    plate = plate.cut(
        cq.Workplane("XY").circle(0.010).extrude(top_surface_z + 0.002)
    )
    plate = plate.cut(
        cq.Workplane("XY")
        .workplane(offset=hub_height)
        .transformed(rotate=(0.0, 0.0, 45.0))
        .rect(0.038, 0.010)
        .extrude(0.0035)
    )
    plate = plate.cut(
        cq.Workplane("XY")
        .workplane(offset=hub_height)
        .transformed(rotate=(0.0, 0.0, -45.0))
        .rect(0.038, 0.010)
        .extrude(0.0035)
    )
    plate = plate.union(
        cq.Workplane("XY")
        .workplane(offset=top_surface_z)
        .center(0.056, 0.0)
        .box(0.026, 0.010, 0.008, centered=(True, True, False))
    )
    plate = plate.union(
        cq.Workplane("XY")
        .workplane(offset=top_surface_z + 0.008)
        .center(0.074, 0.0)
        .circle(0.0065)
        .extrude(0.014)
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilevered_offset_rotary_fixture")

    model.material("frame_paint", rgba=(0.22, 0.24, 0.26, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("anodized_plate", rgba=(0.58, 0.62, 0.67, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "fixture_frame"),
        material="frame_paint",
        name="frame_shell",
    )

    base_table = model.part("base_table")
    base_table.visual(
        mesh_from_cadquery(_base_table_shape(), "base_rotary_plate"),
        material="machined_steel",
        name="base_table_shell",
    )

    secondary_plate = model.part("secondary_plate")
    secondary_plate.visual(
        mesh_from_cadquery(_secondary_plate_shape(), "secondary_rotary_plate"),
        material="anodized_plate",
        name="secondary_plate_shell",
    )

    model.articulation(
        "frame_to_base_table",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=base_table,
        origin=Origin(xyz=(MAIN_AXIS_X, 0.0, SUPPORT_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=-pi,
            upper=pi,
        ),
    )

    model.articulation(
        "frame_to_secondary_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=secondary_plate,
        origin=Origin(xyz=(SECONDARY_AXIS_X, 0.0, SUPPORT_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    base_table = object_model.get_part("base_table")
    secondary_plate = object_model.get_part("secondary_plate")
    main_joint = object_model.get_articulation("frame_to_base_table")
    secondary_joint = object_model.get_articulation("frame_to_secondary_plate")

    ctx.expect_gap(
        base_table,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="base table seats on the central bearing support",
    )
    ctx.expect_gap(
        secondary_plate,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="secondary plate seats on the cantilevered bearing support",
    )
    ctx.expect_origin_distance(
        secondary_plate,
        base_table,
        axes="x",
        min_dist=0.20,
        max_dist=0.24,
        name="secondary axis stays offset from the main axis",
    )
    ctx.check(
        "both rotary joints use parallel supported vertical axes",
        (
            main_joint.axis == (0.0, 0.0, 1.0)
            and secondary_joint.axis == (0.0, 0.0, 1.0)
            and abs(main_joint.origin.xyz[1] - secondary_joint.origin.xyz[1]) < 1e-9
            and abs(main_joint.origin.xyz[2] - secondary_joint.origin.xyz[2]) < 1e-9
            and secondary_joint.origin.xyz[0] - main_joint.origin.xyz[0] > 0.20
        ),
        details=(
            f"main_origin={main_joint.origin.xyz}, secondary_origin={secondary_joint.origin.xyz}, "
            f"main_axis={main_joint.axis}, secondary_axis={secondary_joint.axis}"
        ),
    )

    rest_table_aabb = ctx.part_world_aabb(base_table)
    with ctx.pose({main_joint: pi / 2.0}):
        turned_table_aabb = ctx.part_world_aabb(base_table)
    ctx.check(
        "base table rotation is visually legible",
        (
            rest_table_aabb is not None
            and turned_table_aabb is not None
            and turned_table_aabb[1][0] < rest_table_aabb[1][0] - 0.007
            and turned_table_aabb[1][1] > rest_table_aabb[1][1] + 0.007
        ),
        details=f"rest={rest_table_aabb}, turned={turned_table_aabb}",
    )

    rest_secondary_aabb = ctx.part_world_aabb(secondary_plate)
    with ctx.pose({secondary_joint: pi / 2.0}):
        turned_secondary_aabb = ctx.part_world_aabb(secondary_plate)
    ctx.check(
        "secondary plate rotation is visually legible",
        (
            rest_secondary_aabb is not None
            and turned_secondary_aabb is not None
            and turned_secondary_aabb[1][0] < rest_secondary_aabb[1][0] - 0.006
            and turned_secondary_aabb[1][1] > rest_secondary_aabb[1][1] + 0.006
        ),
        details=f"rest={rest_secondary_aabb}, turned={turned_secondary_aabb}",
    )

    with ctx.pose({main_joint: 0.0, secondary_joint: pi}):
        ctx.expect_gap(
            secondary_plate,
            base_table,
            axis="x",
            min_gap=0.002,
            name="the two rotary plates keep a real lateral clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
