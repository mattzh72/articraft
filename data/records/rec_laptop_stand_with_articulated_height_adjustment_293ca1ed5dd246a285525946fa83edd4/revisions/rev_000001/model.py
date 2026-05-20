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
    model = ArticulatedObject(name="desktop_laptop_stand")

    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.05, 0.055, 0.06, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.42, 0.43, 0.43, 1.0))

    # Object frame: +X points toward the front lip, +Y spans left/right, +Z is up.
    rear_x = -0.14
    column_y = 0.18
    sleeve_top_z = 0.14

    base = model.part("base")
    base.visual(
        Box((0.085, 0.44, 0.012)),
        origin=Origin(xyz=(rear_x, 0.0, 0.006)),
        material=dark_anodized,
        name="rear_foot_bar",
    )
    base.visual(
        Box((0.36, 0.035, 0.018)),
        origin=Origin(xyz=(0.02, -column_y, 0.009)),
        material=dark_anodized,
        name="left_desk_foot",
    )
    base.visual(
        Box((0.36, 0.035, 0.018)),
        origin=Origin(xyz=(0.02, column_y, 0.009)),
        material=dark_anodized,
        name="right_desk_foot",
    )
    base.visual(
        Cylinder(radius=0.023, length=0.1224),
        origin=Origin(xyz=(rear_x, -column_y, 0.0788)),
        material=satin_aluminum,
        name="left_sleeve",
    )
    base.visual(
        Cylinder(radius=0.023, length=0.1224),
        origin=Origin(xyz=(rear_x, column_y, 0.0788)),
        material=satin_aluminum,
        name="right_sleeve",
    )
    base.visual(
        Box((0.070, 0.026, 0.004)),
        origin=Origin(xyz=(0.165, -column_y, 0.002)),
        material=black_rubber,
        name="left_front_foot_pad",
    )
    base.visual(
        Box((0.070, 0.026, 0.004)),
        origin=Origin(xyz=(0.165, column_y, 0.002)),
        material=black_rubber,
        name="right_front_foot_pad",
    )

    column_radius = 0.014
    column_length = 0.280
    column_center_z = 0.015
    column_top_local_z = column_center_z + column_length / 2.0
    bar_center_above_column_top = 0.035
    bar_joint_z = column_top_local_z + bar_center_above_column_top

    left_column = model.part("left_column")
    left_column.visual(
        Cylinder(radius=column_radius, length=column_length),
        origin=Origin(xyz=(0.0, 0.0, column_center_z)),
        material=brushed_steel,
        name="inner_post",
    )

    right_column = model.part("right_column")
    right_column.visual(
        Cylinder(radius=column_radius, length=column_length),
        origin=Origin(xyz=(0.0, 0.0, column_center_z)),
        material=brushed_steel,
        name="inner_post",
    )

    tilt_bar = model.part("tilt_bar")
    tilt_bar.visual(
        Cylinder(radius=0.007, length=0.430),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_pin",
    )
    for side, y in (("left", -column_y), ("right", column_y)):
        tilt_bar.visual(
            Box((0.040, 0.046, 0.030)),
            # The saddle bottom sits on top of the matching vertical post; its
            # top slightly intersects the pin so the bar reads as one rigid weldment.
            origin=Origin(xyz=(0.0, y, -0.020)),
            material=dark_anodized,
            name=f"{side}_top_saddle",
        )

    tray_depth = 0.320
    tray_width = 0.365
    tray = model.part("tray")
    deck_geometry = SlotPatternPanelGeometry(
        (tray_depth - 0.020, tray_width),
        0.006,
        slot_size=(0.055, 0.006),
        pitch=(0.075, 0.034),
        frame=0.020,
        corner_radius=0.012,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(deck_geometry, "tray_slotted_deck"),
        origin=Origin(xyz=(tray_depth / 2.0 + 0.010, 0.0, 0.0)),
        material=satin_aluminum,
        name="slotted_deck",
    )
    tray.visual(
        Cylinder(radius=0.011, length=0.300),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="rear_hinge_sleeve",
    )
    tray.visual(
        Box((0.016, 0.320, 0.0065)),
        origin=Origin(xyz=(0.016, 0.0, 0.00575)),
        material=satin_aluminum,
        name="rear_hinge_leaf",
    )
    tray.visual(
        Box((0.020, 0.330, 0.030)),
        origin=Origin(xyz=(tray_depth - 0.010, 0.0, 0.012)),
        material=satin_aluminum,
        name="front_lip",
    )
    for side, y in (("left", -tray_width / 2.0 + 0.006), ("right", tray_width / 2.0 - 0.006)):
        tray.visual(
            Box((0.270, 0.012, 0.012)),
            origin=Origin(xyz=(0.170, y, 0.007)),
            material=satin_aluminum,
            name=f"{side}_side_rail",
        )
    for idx, (x, y) in enumerate(((0.105, -0.095), (0.105, 0.095), (0.245, -0.095), (0.245, 0.095))):
        tray.visual(
            Box((0.060, 0.020, 0.003)),
            origin=Origin(xyz=(x, y, 0.0043)),
            material=black_rubber,
            name=f"rubber_pad_{idx}",
        )

    left_slide = model.articulation(
        "base_to_left_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_column,
        origin=Origin(xyz=(rear_x, -column_y, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.10),
    )
    model.articulation(
        "base_to_right_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_column,
        origin=Origin(xyz=(rear_x, column_y, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.10),
        mimic=Mimic(joint=left_slide.name, multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "left_column_to_tilt_bar",
        ArticulationType.FIXED,
        parent=left_column,
        child=tilt_bar,
        origin=Origin(xyz=(0.0, column_y, bar_joint_z)),
    )
    model.articulation(
        "tilt_bar_to_tray",
        ArticulationType.REVOLUTE,
        parent=tilt_bar,
        child=tray,
        origin=Origin(),
        # With the tray extending along local +X, positive motion lifts the front lip.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=0.75),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    left_column = object_model.get_part("left_column")
    right_column = object_model.get_part("right_column")
    tilt_bar = object_model.get_part("tilt_bar")
    tray = object_model.get_part("tray")
    left_slide = object_model.get_articulation("base_to_left_column")
    right_slide = object_model.get_articulation("base_to_right_column")
    tray_hinge = object_model.get_articulation("tilt_bar_to_tray")

    ctx.allow_overlap(
        base,
        left_column,
        elem_a="left_sleeve",
        elem_b="inner_post",
        reason="The polished inner post is intentionally represented sliding inside the left outer sleeve proxy.",
    )
    ctx.allow_overlap(
        base,
        right_column,
        elem_a="right_sleeve",
        elem_b="inner_post",
        reason="The polished inner post is intentionally represented sliding inside the right outer sleeve proxy.",
    )
    ctx.allow_overlap(
        tilt_bar,
        tray,
        elem_a="hinge_pin",
        elem_b="rear_hinge_sleeve",
        reason="The rear tray sleeve captures the common tilt bar pin; the solid proxy overlap represents the pin inside the hinge tube.",
    )

    ctx.expect_within(
        left_column,
        base,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="left_sleeve",
        margin=0.002,
        name="left post stays centered in sleeve",
    )
    ctx.expect_within(
        right_column,
        base,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="right_sleeve",
        margin=0.002,
        name="right post stays centered in sleeve",
    )
    ctx.expect_overlap(
        left_column,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="left_sleeve",
        min_overlap=0.08,
        name="left post retained in collapsed sleeve",
    )
    ctx.expect_overlap(
        right_column,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="right_sleeve",
        min_overlap=0.08,
        name="right post retained in collapsed sleeve",
    )
    ctx.expect_within(
        tilt_bar,
        tray,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="rear_hinge_sleeve",
        margin=0.001,
        name="common pin is coaxial with tray hinge sleeve",
    )
    ctx.expect_overlap(
        tilt_bar,
        tray,
        axes="y",
        elem_a="hinge_pin",
        elem_b="rear_hinge_sleeve",
        min_overlap=0.30,
        name="tray uses one shared rear hinge line",
    )

    ctx.check(
        "right column mimics left column",
        getattr(right_slide, "mimic", None) is not None
        and right_slide.mimic.joint == left_slide.name
        and abs(right_slide.mimic.multiplier - 1.0) < 1e-9,
        details=f"right mimic={getattr(right_slide, 'mimic', None)}",
    )

    rest_left = ctx.part_world_position(left_column)
    rest_right = ctx.part_world_position(right_column)
    rest_bar = ctx.part_world_position(tilt_bar)
    with ctx.pose({left_slide: 0.10}):
        ctx.expect_overlap(
            left_column,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="left_sleeve",
            min_overlap=0.020,
            name="left post retained at full height",
        )
        ctx.expect_overlap(
            right_column,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="right_sleeve",
            min_overlap=0.020,
            name="right post retained at full height",
        )
        raised_left = ctx.part_world_position(left_column)
        raised_right = ctx.part_world_position(right_column)
        raised_bar = ctx.part_world_position(tilt_bar)

    ctx.check(
        "matched columns extend together",
        rest_left is not None
        and rest_right is not None
        and raised_left is not None
        and raised_right is not None
        and abs((raised_left[2] - rest_left[2]) - 0.10) < 1e-6
        and abs((raised_right[2] - rest_right[2]) - 0.10) < 1e-6,
        details=f"rest_left={rest_left}, raised_left={raised_left}, rest_right={rest_right}, raised_right={raised_right}",
    )
    ctx.check(
        "common tilt bar rides with columns",
        rest_bar is not None
        and raised_bar is not None
        and abs((raised_bar[2] - rest_bar[2]) - 0.10) < 1e-6,
        details=f"rest_bar={rest_bar}, raised_bar={raised_bar}",
    )

    rest_lip = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tray_hinge: 0.55}):
        tilted_lip = ctx.part_element_world_aabb(tray, elem="front_lip")
    ctx.check(
        "tray hinge lifts the front lip",
        rest_lip is not None
        and tilted_lip is not None
        and tilted_lip[0][2] > rest_lip[0][2] + 0.09,
        details=f"rest_lip={rest_lip}, tilted_lip={tilted_lip}",
    )

    return ctx.report()


object_model = build_object_model()
