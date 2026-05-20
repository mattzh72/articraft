from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LEAF_WIDTH = 0.72
LEAF_HEIGHT = 1.12
HINGE_AXIS_X = -0.36
LOUVER_CENTER_X = 0.37
LOUVER_COUNT = 6
LOUVER_PITCH = 0.155
LOUVER_Z0 = -0.3875
BAR_TRAVEL = 0.035
DRIVE_PIN_NAMES = (
    "drive_pin_0",
    "drive_pin_1",
    "drive_pin_2",
    "drive_pin_3",
    "drive_pin_4",
    "drive_pin_5",
)


def _louver_z(index: int) -> float:
    return LOUVER_Z0 + index * LOUVER_PITCH


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exterior_storm_shutter")
    model.material("painted_storm_blue", rgba=(0.12, 0.24, 0.32, 1.0))
    model.material("worn_louver_blue", rgba=(0.18, 0.34, 0.42, 1.0))
    model.material("dark_jamb", rgba=(0.075, 0.065, 0.055, 1.0))
    model.material("galvanized_metal", rgba=(0.62, 0.66, 0.65, 1.0))
    model.material("blackened_zinc", rgba=(0.035, 0.035, 0.032, 1.0))

    opening_frame = model.part("opening_frame")
    # A shallow exterior casing behind the movable shutter leaf.
    opening_frame.visual(
        Box((0.09, 0.04, 1.32)),
        origin=Origin(xyz=(-0.425, -0.055, 0.0)),
        material="dark_jamb",
        name="hinge_jamb",
    )
    opening_frame.visual(
        Box((0.09, 0.04, 1.32)),
        origin=Origin(xyz=(0.425, -0.055, 0.0)),
        material="dark_jamb",
        name="free_jamb",
    )
    opening_frame.visual(
        Box((0.90, 0.04, 0.09)),
        origin=Origin(xyz=(0.0, -0.055, 0.615)),
        material="dark_jamb",
        name="head_jamb",
    )
    opening_frame.visual(
        Box((0.90, 0.04, 0.09)),
        origin=Origin(xyz=(0.0, -0.055, -0.615)),
        material="dark_jamb",
        name="sill_jamb",
    )
    # Fixed hinge knuckles and their straps are attached to the jamb.
    for i, z in enumerate((-0.45, 0.0, 0.45)):
        opening_frame.visual(
            Box((0.045, 0.060, 0.18)),
            origin=Origin(xyz=(-0.392, -0.025, z)),
            material="galvanized_metal",
            name=f"jamb_hinge_strap_{i}",
        )
        opening_frame.visual(
            Cylinder(radius=0.018, length=0.18),
            origin=Origin(xyz=(HINGE_AXIS_X, 0.0, z)),
            material="galvanized_metal",
            name=f"jamb_hinge_barrel_{i}",
        )
    opening_frame.visual(
        Cylinder(radius=0.010, length=1.10),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, 0.0)),
        material="blackened_zinc",
        name="hinge_pin",
    )

    leaf = model.part("leaf")
    leaf.visual(
        Box((0.075, 0.060, LEAF_HEIGHT)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material="painted_storm_blue",
        name="hinge_stile",
    )
    leaf.visual(
        Box((0.075, 0.060, LEAF_HEIGHT)),
        origin=Origin(xyz=(0.680, 0.0, 0.0)),
        material="painted_storm_blue",
        name="free_stile",
    )
    leaf.visual(
        Box((0.695, 0.060, 0.075)),
        origin=Origin(xyz=(0.370, 0.0, 0.5225)),
        material="painted_storm_blue",
        name="top_rail",
    )
    leaf.visual(
        Box((0.695, 0.060, 0.075)),
        origin=Origin(xyz=(0.370, 0.0, -0.5225)),
        material="painted_storm_blue",
        name="bottom_rail",
    )
    # Moving hinge knuckles alternate between the fixed jamb barrels.
    for i, z in enumerate((-0.225, 0.225)):
        leaf.visual(
            Box((0.050, 0.028, 0.15)),
            origin=Origin(xyz=(0.035, -0.025, z)),
            material="galvanized_metal",
            name=f"leaf_hinge_leaf_{i}",
        )
        leaf.visual(
            Cylinder(radius=0.018, length=0.18),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="galvanized_metal",
            name=f"leaf_hinge_barrel_{i}",
        )

    # Small screw heads and many clipped pivot saddles make the frame read stout
    # and keep each louver seated in a real-looking outer-frame pivot.
    screw_positions = (
        (0.060, 0.034, -0.47),
        (0.060, 0.034, 0.47),
        (0.680, 0.034, -0.47),
        (0.680, 0.034, 0.47),
        (0.370, 0.034, -0.5225),
        (0.370, 0.034, 0.5225),
    )
    for i, (x, y, z) in enumerate(screw_positions):
        leaf.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, 0.031, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="galvanized_metal",
            name=f"frame_screw_{i}",
        )

    for i in range(LOUVER_COUNT):
        z = _louver_z(i)
        for side_name, x in (("hinge", 0.100), ("free", 0.640)):
            for tab_name, y in (("rear", -0.014), ("front", 0.014)):
                leaf.visual(
                    Box((0.024, 0.012, 0.045)),
                    origin=Origin(xyz=(x, y, z)),
                    material="galvanized_metal",
                    name=f"{side_name}_pivot_clip_{i}_{tab_name}",
                )

    # Two U-shaped guide brackets support the vertical tilt rod while leaving
    # running clearance around it.
    for i, z in enumerate((-0.455, 0.455)):
        leaf.visual(
            Box((0.095, 0.040, 0.030)),
            origin=Origin(xyz=(0.625, 0.045, z)),
            material="galvanized_metal",
            name=f"linkage_guide_base_{i}",
        )
        leaf.visual(
            Box((0.010, 0.065, 0.030)),
            origin=Origin(xyz=(0.5825, 0.096, z)),
            material="galvanized_metal",
            name=f"linkage_guide_side_{i}_0",
        )
        leaf.visual(
            Box((0.010, 0.065, 0.030)),
            origin=Origin(xyz=(0.6175, 0.096, z)),
            material="galvanized_metal",
            name=f"linkage_guide_side_{i}_1",
        )
        leaf.visual(
            Box((0.046, 0.010, 0.030)),
            origin=Origin(xyz=(0.600, 0.127, z)),
            material="galvanized_metal",
            name=f"linkage_guide_front_{i}",
        )

    model.articulation(
        "opening_to_leaf",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child=leaf,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=1.65),
    )

    linkage_bar = model.part("linkage_bar")
    linkage_bar.visual(
        Box((0.025, 0.014, 0.93)),
        origin=Origin(),
        material="blackened_zinc",
        name="vertical_bar",
    )
    for i in range(LOUVER_COUNT):
        linkage_bar.visual(
            Box((0.028, 0.022, 0.018)),
            origin=Origin(xyz=(0.0, -0.012, _louver_z(i))),
            material="galvanized_metal",
            name=DRIVE_PIN_NAMES[i],
        )

    model.articulation(
        "leaf_to_linkage_bar",
        ArticulationType.PRISMATIC,
        parent=leaf,
        child=linkage_bar,
        origin=Origin(xyz=(0.600, 0.110, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=-BAR_TRAVEL, upper=BAR_TRAVEL),
    )

    for i in range(LOUVER_COUNT):
        louver = model.part(f"louver_{i}")
        louver.visual(
            Cylinder(radius=0.008, length=0.536),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="galvanized_metal",
            name="pivot_pin",
        )
        louver.visual(
            Box((0.500, 0.145, 0.024)),
            origin=Origin(rpy=(0.25, 0.0, 0.0)),
            material="worn_louver_blue",
            name="wide_slat",
        )
        louver.visual(
            Box((0.035, 0.030, 0.012)),
            origin=Origin(xyz=(0.230, 0.070, 0.0)),
            material="galvanized_metal",
            name="control_ear",
        )
        model.articulation(
            f"leaf_to_louver_{i}",
            ArticulationType.REVOLUTE,
            parent=leaf,
            child=louver,
            origin=Origin(xyz=(LOUVER_CENTER_X, 0.0, _louver_z(i))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.45, upper=0.45),
            mimic=None if i == 0 else Mimic("leaf_to_louver_0", multiplier=1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    leaf_hinge = object_model.get_articulation("opening_to_leaf")
    linkage = object_model.get_articulation("leaf_to_linkage_bar")
    opening_frame = object_model.get_part("opening_frame")
    leaf = object_model.get_part("leaf")
    linkage_bar = object_model.get_part("linkage_bar")

    for i in range(2):
        ctx.allow_overlap(
            opening_frame,
            leaf,
            elem_a="hinge_pin",
            elem_b=f"leaf_hinge_barrel_{i}",
            reason="The jamb-side hinge pin is intentionally captured inside the moving leaf hinge barrel.",
        )
        ctx.expect_within(
            opening_frame,
            leaf,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=f"leaf_hinge_barrel_{i}",
            margin=0.001,
            name=f"hinge pin is seated in leaf barrel {i}",
        )

    for i in range(LOUVER_COUNT):
        louver = object_model.get_part(f"louver_{i}")
        louver_joint = object_model.get_articulation(f"leaf_to_louver_{i}")
        if i == 0:
            ctx.check(
                "master louver has direct horizontal pivot",
                louver_joint.mimic is None and tuple(louver_joint.axis) == (1.0, 0.0, 0.0),
                details=f"axis={louver_joint.axis}, mimic={louver_joint.mimic}",
            )
        else:
            ctx.check(
                f"louver_{i} follows the master louver",
                louver_joint.mimic is not None
                and louver_joint.mimic.joint == "leaf_to_louver_0"
                and abs(louver_joint.mimic.multiplier - 1.0) < 1e-9,
                details=f"mimic={louver_joint.mimic}",
            )
        ctx.expect_within(
            louver,
            leaf,
            axes="x",
            inner_elem="pivot_pin",
            margin=0.006,
            name=f"louver_{i} pivot pin stays clipped in frame width",
        )

    lower_aabb = ctx.part_element_world_aabb(object_model.get_part("louver_2"), elem="wide_slat")
    rest_bar_pos = ctx.part_world_position(linkage_bar)
    master_louver = object_model.get_articulation("leaf_to_louver_0")
    ctx.expect_gap(
        linkage_bar,
        object_model.get_part("louver_2"),
        axis="y",
        positive_elem="drive_pin_2",
        negative_elem="control_ear",
        min_gap=0.0,
        max_gap=0.008,
        name="linkage drive pin sits just outside louver ear",
    )
    ctx.expect_overlap(
        linkage_bar,
        object_model.get_part("louver_2"),
        axes="xz",
        elem_a="drive_pin_2",
        elem_b="control_ear",
        min_overlap=0.008,
        name="linkage drive pin aligns with louver ear",
    )
    with ctx.pose({linkage: BAR_TRAVEL, master_louver: 0.35}):
        upper_aabb = ctx.part_element_world_aabb(object_model.get_part("louver_2"), elem="wide_slat")
        raised_bar_pos = ctx.part_world_position(linkage_bar)
        ctx.expect_within(
            object_model.get_part("louver_2"),
            leaf,
            axes="x",
            inner_elem="pivot_pin",
            margin=0.006,
            name="tilted louver pivot remains seated",
        )
        ctx.expect_within(
            linkage_bar,
            leaf,
            axes="x",
            inner_elem="vertical_bar",
            margin=0.010,
            name="raised linkage bar remains in guide width",
        )
        ctx.expect_gap(
            linkage_bar,
            object_model.get_part("louver_2"),
            axis="y",
            positive_elem="drive_pin_2",
            negative_elem="control_ear",
            min_gap=0.0,
            max_gap=0.012,
            name="raised linkage drive pin remains near louver ear",
        )

    ctx.check(
        "linkage bar translates vertically",
        rest_bar_pos is not None and raised_bar_pos is not None and raised_bar_pos[2] > rest_bar_pos[2] + 0.030,
        details=f"rest={rest_bar_pos}, raised={raised_bar_pos}",
    )
    if lower_aabb is not None and upper_aabb is not None:
        lower_height = lower_aabb[1][2] - lower_aabb[0][2]
        upper_height = upper_aabb[1][2] - upper_aabb[0][2]
        ctx.check(
            "coupled raised linkage pose tilts the wide slats",
            upper_height > lower_height + 0.015,
            details=f"rest slat height={lower_height:.4f}, raised height={upper_height:.4f}",
        )
    else:
        ctx.fail("coupled raised linkage pose tilts the wide slats", "could not measure louver blade AABBs")

    closed_aabb = ctx.part_world_aabb(leaf)
    with ctx.pose({leaf_hinge: 1.25}):
        opened_aabb = ctx.part_world_aabb(leaf)
    if closed_aabb is not None and opened_aabb is not None:
        ctx.check(
            "leaf swings outward on side hinge",
            opened_aabb[1][1] > closed_aabb[1][1] + 0.35,
            details=f"closed y-max={closed_aabb[1][1]:.3f}, opened y-max={opened_aabb[1][1]:.3f}",
        )
    else:
        ctx.fail("leaf swings outward on side hinge", "could not measure leaf AABBs")

    return ctx.report()


object_model = build_object_model()
