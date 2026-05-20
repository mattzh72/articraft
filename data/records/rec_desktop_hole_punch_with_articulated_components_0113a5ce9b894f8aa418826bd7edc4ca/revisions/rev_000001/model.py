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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_hole_paper_punch")

    body_gray = model.material("body_gray", rgba=(0.42, 0.45, 0.49, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.81, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.12, 0.13, 0.14, 1.0))
    ruler_gray = model.material("ruler_gray", rgba=(0.70, 0.72, 0.74, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.31, 0.30, 0.09)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    base.visual(
        Box((0.23, 0.30, 0.004)),
        origin=Origin(xyz=(-0.035, 0.0, 0.002)),
        material=dark_gray,
        name="rear_floor",
    )
    base.visual(
        Box((0.30, 0.010, 0.052)),
        origin=Origin(xyz=(-0.005, -0.145, 0.028)),
        material=body_gray,
        name="left_wall",
    )
    base.visual(
        Box((0.30, 0.010, 0.052)),
        origin=Origin(xyz=(-0.005, 0.145, 0.028)),
        material=body_gray,
        name="right_wall",
    )
    base.visual(
        Box((0.040, 0.30, 0.052)),
        origin=Origin(xyz=(-0.135, 0.0, 0.028)),
        material=body_gray,
        name="rear_wall",
    )
    base.visual(
        Box((0.20, 0.30, 0.008)),
        origin=Origin(xyz=(-0.030, 0.0, 0.050)),
        material=body_gray,
        name="top_deck",
    )
    base.visual(
        Box((0.11, 0.30, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, 0.045)),
        material=body_gray,
        name="punch_block",
    )
    base.visual(
        Box((0.020, 0.30, 0.012)),
        origin=Origin(xyz=(0.145, 0.0, 0.030)),
        material=dark_gray,
        name="front_track",
    )
    base.visual(
        Box((0.16, 0.018, 0.004)),
        origin=Origin(xyz=(0.030, -0.095, 0.003)),
        material=dark_gray,
        name="left_guide",
    )
    base.visual(
        Box((0.16, 0.018, 0.004)),
        origin=Origin(xyz=(0.030, 0.095, 0.003)),
        material=dark_gray,
        name="right_guide",
    )
    base.visual(
        Box((0.030, 0.040, 0.024)),
        origin=Origin(xyz=(-0.105, -0.112, 0.066)),
        material=dark_gray,
        name="left_hinge_pedestal",
    )
    base.visual(
        Box((0.030, 0.040, 0.024)),
        origin=Origin(xyz=(-0.105, 0.112, 0.066)),
        material=dark_gray,
        name="right_hinge_pedestal",
    )

    punch_y = (-0.120, -0.040, 0.040, 0.120)
    for index, y in enumerate(punch_y):
        base.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(0.050, y, 0.056)),
            material=steel,
            name=f"die_{index}",
        )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.24, 0.30, 0.05)),
        mass=0.9,
        origin=Origin(xyz=(0.12, 0.0, 0.020)),
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.224),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_rod",
    )
    handle.visual(
        Box((0.18, 0.014, 0.020)),
        origin=Origin(xyz=(0.100, -0.112, 0.008)),
        material=handle_black,
        name="left_arm",
    )
    handle.visual(
        Box((0.18, 0.014, 0.020)),
        origin=Origin(xyz=(0.100, 0.112, 0.008)),
        material=handle_black,
        name="right_arm",
    )
    handle.visual(
        Box((0.12, 0.25, 0.012)),
        origin=Origin(xyz=(0.120, 0.0, 0.016)),
        material=handle_black,
        name="top_panel",
    )
    handle.visual(
        Box((0.060, 0.27, 0.018)),
        origin=Origin(xyz=(0.195, 0.0, 0.018)),
        material=handle_black,
        name="front_beam",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.29),
        origin=Origin(xyz=(0.205, 0.0, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="grip",
    )
    handle.visual(
        Box((0.050, 0.27, 0.012)),
        origin=Origin(xyz=(0.160, 0.0, 0.004)),
        material=dark_gray,
        name="carrier",
    )
    for index, y in enumerate(punch_y):
        handle.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(0.155, y, -0.007)),
            material=steel,
            name=f"head_{index}",
        )
        handle.visual(
            Cylinder(radius=0.005, length=0.016),
            origin=Origin(xyz=(0.155, y, -0.020)),
            material=steel,
            name=f"plunger_{index}",
        )

    gauge = model.part("gauge")
    gauge.inertial = Inertial.from_geometry(
        Box((0.030, 0.19, 0.030)),
        mass=0.08,
    )
    gauge.visual(
        Box((0.010, 0.19, 0.006)),
        origin=Origin(),
        material=ruler_gray,
        name="bar",
    )
    gauge.visual(
        Box((0.014, 0.032, 0.008)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=dark_gray,
        name="carriage",
    )
    gauge.visual(
        Box((0.020, 0.004, 0.022)),
        origin=Origin(xyz=(-0.010, -0.072, 0.009)),
        material=ruler_gray,
        name="fence",
    )

    drawer = model.part("drawer")
    drawer.inertial = Inertial.from_geometry(
        Box((0.16, 0.23, 0.03)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    drawer.visual(
        Box((0.155, 0.215, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=drawer_black,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.145, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, -0.106, 0.008)),
        material=drawer_black,
        name="left_side",
    )
    drawer.visual(
        Box((0.145, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.106, 0.008)),
        material=drawer_black,
        name="right_side",
    )
    drawer.visual(
        Box((0.003, 0.209, 0.014)),
        origin=Origin(xyz=(-0.076, 0.0, 0.008)),
        material=drawer_black,
        name="back_wall",
    )
    drawer.visual(
        Box((0.012, 0.223, 0.012)),
        origin=Origin(xyz=(0.072, 0.0, 0.009)),
        material=drawer_black,
        name="front_lip",
    )
    drawer.visual(
        Box((0.018, 0.060, 0.010)),
        origin=Origin(xyz=(0.086, 0.0, 0.012)),
        material=steel,
        name="pull",
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.105, 0.0, 0.088)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "base_to_gauge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gauge,
        origin=Origin(xyz=(0.148, 0.0, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.10,
            lower=-0.050,
            upper=0.050,
        ),
    )
    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.015, 0.0, 0.005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.12,
            lower=0.0,
            upper=0.070,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    gauge = object_model.get_part("gauge")
    drawer = object_model.get_part("drawer")

    handle_hinge = object_model.get_articulation("base_to_handle")
    gauge_slide = object_model.get_articulation("base_to_gauge")
    drawer_slide = object_model.get_articulation("base_to_drawer")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="plunger_1",
        negative_elem="die_1",
        min_gap=0.001,
        max_gap=0.004,
        name="closed handle leaves a small plunger gap above the die line",
    )

    ctx.expect_within(
        gauge,
        base,
        axes="y",
        inner_elem="bar",
        outer_elem="front_track",
        margin=0.0,
        name="gauge bar stays within the front track span at rest",
    )
    ctx.expect_gap(
        gauge,
        base,
        axis="z",
        positive_elem="carriage",
        negative_elem="front_track",
        min_gap=0.0,
        max_gap=0.0005,
        name="gauge carriage sits on the front track at rest",
    )

    ctx.expect_within(
        drawer,
        base,
        axes="y",
        inner_elem="front_lip",
        outer_elem="front_track",
        margin=0.04,
        name="drawer stays laterally centered under the punch line",
    )

    grip_rest_aabb = ctx.part_element_world_aabb(handle, elem="grip")
    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        grip_open_aabb = ctx.part_element_world_aabb(handle, elem="grip")
    rest_grip_z = None if grip_rest_aabb is None else 0.5 * (grip_rest_aabb[0][2] + grip_rest_aabb[1][2])
    open_grip_z = None if grip_open_aabb is None else 0.5 * (grip_open_aabb[0][2] + grip_open_aabb[1][2])
    ctx.check(
        "handle opens upward from the rear hinge",
        rest_grip_z is not None and open_grip_z is not None and open_grip_z > rest_grip_z + 0.05,
        details=f"rest_z={rest_grip_z}, open_z={open_grip_z}",
    )

    rest_gauge_pos = ctx.part_world_position(gauge)
    with ctx.pose({gauge_slide: gauge_slide.motion_limits.upper}):
        ctx.expect_within(
            gauge,
            base,
            axes="y",
            inner_elem="bar",
            outer_elem="front_track",
            margin=0.0,
            name="gauge bar stays within the front track span at max travel",
        )
        gauge_max_pos = ctx.part_world_position(gauge)
    ctx.check(
        "gauge bar slides along the transverse front edge",
        rest_gauge_pos is not None and gauge_max_pos is not None and gauge_max_pos[1] > rest_gauge_pos[1] + 0.03,
        details=f"rest={rest_gauge_pos}, max={gauge_max_pos}",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            min_overlap=0.08,
            name="drawer retains insertion at full extension",
        )
        ctx.expect_within(
            drawer,
            base,
            axes="y",
            inner_elem="front_lip",
            outer_elem="front_track",
            margin=0.04,
            name="drawer stays aligned with the base guides when extended",
        )
        drawer_max_pos = ctx.part_world_position(drawer)
    ctx.check(
        "chip drawer slides forward out of the base",
        rest_drawer_pos is not None and drawer_max_pos is not None and drawer_max_pos[0] > rest_drawer_pos[0] + 0.05,
        details=f"rest={rest_drawer_pos}, max={drawer_max_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
