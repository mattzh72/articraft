from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_laptop_stand")

    black = Material("matte_black_powdercoat", rgba=(0.02, 0.022, 0.025, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = Material("dark_anodized_tray", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = Material("soft_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    steel = Material("polished_hinge_pin", rgba=(0.58, 0.60, 0.62, 1.0))

    model.materials.extend([black, aluminum, dark, rubber, steel])

    base = model.part("base_frame")
    # Low rectangular desktop frame.
    base.visual(Box((0.026, 0.330, 0.018)), origin=Origin(xyz=(-0.170, 0.015, 0.009)), material=black, name="side_rail_0")
    base.visual(Box((0.026, 0.330, 0.018)), origin=Origin(xyz=(0.170, 0.015, 0.009)), material=black, name="side_rail_1")
    base.visual(Box((0.366, 0.026, 0.018)), origin=Origin(xyz=(0.0, -0.145, 0.009)), material=black, name="rear_rail")
    base.visual(Box((0.326, 0.020, 0.014)), origin=Origin(xyz=(0.0, 0.175, 0.007)), material=black, name="front_rail")

    for i, x in enumerate((-0.150, 0.150)):
        # Four-wall square sleeve, left open as a real telescoping socket.
        base.visual(Box((0.006, 0.034, 0.180)), origin=Origin(xyz=(x - 0.014, -0.120, 0.108)), material=black, name=f"outer_wall_{i}_inner")
        base.visual(Box((0.006, 0.034, 0.180)), origin=Origin(xyz=(x + 0.014, -0.120, 0.108)), material=black, name=f"outer_wall_{i}_outer")
        base.visual(Box((0.034, 0.006, 0.180)), origin=Origin(xyz=(x, -0.134, 0.108)), material=black, name=f"outer_wall_{i}_rear")
        base.visual(Box((0.034, 0.006, 0.180)), origin=Origin(xyz=(x, -0.106, 0.108)), material=black, name=f"outer_wall_{i}_front")
        base.visual(Box((0.052, 0.046, 0.012)), origin=Origin(xyz=(x, -0.120, 0.024)), material=black, name=f"sleeve_foot_{i}")

    for i, (x, y) in enumerate(((-0.170, -0.145), (0.170, -0.145), (-0.170, 0.175), (0.170, 0.175))):
        base.visual(Box((0.040, 0.030, 0.006)), origin=Origin(xyz=(x, y, -0.003)), material=rubber, name=f"desk_pad_{i}")

    inner_columns = []
    for i, x in enumerate((-0.150, 0.150)):
        column = model.part(f"inner_column_{i}")
        column.visual(Box((0.022, 0.022, 0.185)), origin=Origin(xyz=(0.0, 0.0, 0.0925)), material=aluminum, name="inner_tube")
        column.visual(Box((0.032, 0.032, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.193)), material=black, name="top_cap")
        inner_columns.append(column)

        model.articulation(
            f"height_column_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=column,
            origin=Origin(xyz=(x, -0.120, 0.060)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.100),
            mimic=Mimic(joint="height_column_0") if i == 1 else None,
        )

    hinge_beam = model.part("hinge_beam")
    hinge_beam.visual(Box((0.398, 0.024, 0.020)), origin=Origin(xyz=(0.150, 0.0, 0.010)), material=black, name="top_crossbar")
    hinge_beam.visual(
        Cylinder(radius=0.006, length=0.390),
        origin=Origin(xyz=(0.150, 0.020, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    for i, x in enumerate((-0.030, 0.330)):
        hinge_beam.visual(Box((0.036, 0.018, 0.030)), origin=Origin(xyz=(x, 0.010, 0.020)), material=black, name=f"pin_block_{i}")

    model.articulation(
        "column_to_beam",
        ArticulationType.FIXED,
        parent=inner_columns[0],
        child=hinge_beam,
        origin=Origin(xyz=(0.0, 0.0, 0.201)),
    )

    tray = model.part("top_tray")
    tray_panel = SlotPatternPanelGeometry(
        (0.340, 0.250),
        0.006,
        slot_size=(0.056, 0.008),
        pitch=(0.076, 0.040),
        frame=0.020,
        corner_radius=0.008,
        stagger=True,
    )
    tray.visual(mesh_from_geometry(tray_panel, "slotted_tray_panel"), origin=Origin(xyz=(0.0, 0.135, -0.012)), material=dark, name="slotted_panel")
    tray.visual(
        Cylinder(radius=0.010, length=0.312),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    tray.visual(Box((0.330, 0.026, 0.004)), origin=Origin(xyz=(0.0, 0.012, -0.011)), material=dark, name="rear_leaf")
    tray.visual(Box((0.012, 0.248, 0.030)), origin=Origin(xyz=(-0.172, 0.137, 0.002)), material=dark, name="side_lip_0")
    tray.visual(Box((0.012, 0.248, 0.030)), origin=Origin(xyz=(0.172, 0.137, 0.002)), material=dark, name="side_lip_1")
    tray.visual(Box((0.310, 0.016, 0.035)), origin=Origin(xyz=(0.0, 0.264, 0.005)), material=dark, name="front_stop")
    tray.visual(Box((0.112, 0.016, 0.003)), origin=Origin(xyz=(-0.082, 0.145, -0.0075)), material=rubber, name="rubber_strip_0")
    tray.visual(Box((0.112, 0.016, 0.003)), origin=Origin(xyz=(0.082, 0.145, -0.0075)), material=rubber, name="rubber_strip_1")

    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=hinge_beam,
        child=tray,
        origin=Origin(xyz=(0.150, 0.020, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    column_0 = object_model.get_part("inner_column_0")
    column_1 = object_model.get_part("inner_column_1")
    beam = object_model.get_part("hinge_beam")
    tray = object_model.get_part("top_tray")

    height_0 = object_model.get_articulation("height_column_0")
    height_1 = object_model.get_articulation("height_column_1")
    tray_tilt = object_model.get_articulation("tray_tilt")

    ctx.allow_overlap(
        beam,
        tray,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The visible hinge pin is intentionally captured inside the tray barrel.",
    )

    ctx.check(
        "second column mimics first height joint",
        getattr(height_1, "mimic", None) is not None and height_1.mimic.joint == "height_column_0",
        details=f"height_column_1 mimic={getattr(height_1, 'mimic', None)}",
    )

    for i, column in enumerate((column_0, column_1)):
        ctx.expect_gap(base, column, axis="x", min_gap=0.0, max_gap=0.001, positive_elem=f"outer_wall_{i}_outer", negative_elem="inner_tube", name=f"column_{i} outer wall sliding contact")
        ctx.expect_gap(column, base, axis="x", min_gap=0.0, max_gap=0.001, positive_elem="inner_tube", negative_elem=f"outer_wall_{i}_inner", name=f"column_{i} inner wall sliding contact")
        ctx.expect_gap(base, column, axis="y", min_gap=0.0, max_gap=0.001, positive_elem=f"outer_wall_{i}_front", negative_elem="inner_tube", name=f"column_{i} front wall sliding contact")
        ctx.expect_gap(column, base, axis="y", min_gap=0.0, max_gap=0.001, positive_elem="inner_tube", negative_elem=f"outer_wall_{i}_rear", name=f"column_{i} rear wall sliding contact")
        ctx.expect_overlap(column, base, axes="z", min_overlap=0.080, elem_a="inner_tube", elem_b=f"outer_wall_{i}_front", name=f"column_{i} retained insertion at low height")

    ctx.expect_overlap(beam, tray, axes="x", min_overlap=0.280, elem_a="hinge_pin", elem_b="hinge_barrel", name="tray hinge spans both supports")
    ctx.expect_within(beam, tray, axes="yz", inner_elem="hinge_pin", outer_elem="hinge_barrel", margin=0.001, name="hinge pin stays centered in barrel")

    rest_pos_0 = ctx.part_world_position(column_0)
    rest_pos_1 = ctx.part_world_position(column_1)
    rest_front = ctx.part_element_world_aabb(tray, elem="front_stop")
    with ctx.pose({height_0: 0.100}):
        high_pos_0 = ctx.part_world_position(column_0)
        high_pos_1 = ctx.part_world_position(column_1)
        ctx.expect_overlap(column_0, base, axes="z", min_overlap=0.030, elem_a="inner_tube", elem_b="outer_wall_0_front", name="column_0 remains inserted at full height")
        ctx.expect_overlap(column_1, base, axes="z", min_overlap=0.030, elem_a="inner_tube", elem_b="outer_wall_1_front", name="column_1 remains inserted at full height")

    ctx.check(
        "matched columns rise together",
        rest_pos_0 is not None
        and rest_pos_1 is not None
        and high_pos_0 is not None
        and high_pos_1 is not None
        and high_pos_0[2] > rest_pos_0[2] + 0.095
        and high_pos_1[2] > rest_pos_1[2] + 0.095
        and abs((high_pos_0[2] - rest_pos_0[2]) - (high_pos_1[2] - rest_pos_1[2])) < 0.001,
        details=f"rest0={rest_pos_0}, rest1={rest_pos_1}, high0={high_pos_0}, high1={high_pos_1}",
    )

    with ctx.pose({tray_tilt: 0.60}):
        tilted_front = ctx.part_element_world_aabb(tray, elem="front_stop")
        ctx.expect_overlap(beam, tray, axes="x", min_overlap=0.280, elem_a="hinge_pin", elem_b="hinge_barrel", name="hinge remains aligned while tilted")

    ctx.check(
        "tray front rises when tilted",
        rest_front is not None and tilted_front is not None and tilted_front[0][2] > rest_front[0][2] + 0.10,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    return ctx.report()


object_model = build_object_model()
