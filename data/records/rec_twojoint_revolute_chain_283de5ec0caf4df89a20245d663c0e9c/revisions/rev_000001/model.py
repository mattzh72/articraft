from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


METAL = Material("satin_anodized_aluminum", rgba=(0.62, 0.66, 0.68, 1.0))
BRACKET = Material("dark_powder_coated_steel", rgba=(0.09, 0.10, 0.11, 1.0))
PIN = Material("dark_burnished_pin", rgba=(0.015, 0.015, 0.014, 1.0))
TAB = Material("matte_end_tab", rgba=(0.10, 0.17, 0.23, 1.0))


Y_CYLINDER = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _y_cyl_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=Y_CYLINDER.rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_two_joint_revolute_chain")

    root_link = model.part("root_link")
    root_link.visual(
        Box((0.220, 0.160, 0.025)),
        origin=Origin(xyz=(-0.105, 0.0, -0.177)),
        material=BRACKET,
        name="ground_foot",
    )
    root_link.visual(
        Box((0.052, 0.118, 0.178)),
        origin=Origin(xyz=(-0.105, 0.0, -0.082)),
        material=BRACKET,
        name="upright_web",
    )
    root_link.visual(
        Box((0.170, 0.035, 0.035)),
        origin=Origin(xyz=(-0.115, 0.0, 0.0)),
        material=METAL,
        name="fixed_link",
    )
    root_link.visual(
        Box((0.030, 0.130, 0.045)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        material=BRACKET,
        name="root_yoke_bridge",
    )
    root_link.visual(
        Box((0.060, 0.035, 0.080)),
        origin=Origin(xyz=(0.0, 0.0475, 0.0)),
        material=BRACKET,
        name="root_lug_upper",
    )
    root_link.visual(
        Box((0.060, 0.035, 0.080)),
        origin=Origin(xyz=(0.0, -0.0475, 0.0)),
        material=BRACKET,
        name="root_lug_lower",
    )
    root_link.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=_y_cyl_origin(0.0, 0.068, 0.0),
        material=PIN,
        name="root_cap_upper",
    )
    root_link.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=_y_cyl_origin(0.0, -0.068, 0.0),
        material=PIN,
        name="root_cap_lower",
    )

    middle_link = model.part("middle_link")
    middle_link.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Y_CYLINDER,
        material=METAL,
        name="first_barrel",
    )
    middle_link.visual(
        Box((0.260, 0.030, 0.030)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=METAL,
        name="middle_beam",
    )
    middle_link.visual(
        Box((0.030, 0.128, 0.044)),
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        material=METAL,
        name="middle_yoke_bridge",
    )
    middle_link.visual(
        Box((0.060, 0.035, 0.076)),
        origin=Origin(xyz=(0.320, 0.0475, 0.0)),
        material=METAL,
        name="middle_lug_upper",
    )
    middle_link.visual(
        Box((0.060, 0.035, 0.076)),
        origin=Origin(xyz=(0.320, -0.0475, 0.0)),
        material=METAL,
        name="middle_lug_lower",
    )
    middle_link.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=_y_cyl_origin(0.320, 0.068, 0.0),
        material=PIN,
        name="middle_cap_upper",
    )
    middle_link.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=_y_cyl_origin(0.320, -0.068, 0.0),
        material=PIN,
        name="middle_cap_lower",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Y_CYLINDER,
        material=METAL,
        name="second_barrel",
    )
    end_tab.visual(
        Box((0.118, 0.032, 0.026)),
        origin=Origin(xyz=(0.081, 0.0, 0.0)),
        material=TAB,
        name="tab_plate",
    )
    end_tab.visual(
        Box((0.024, 0.050, 0.030)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=TAB,
        name="end_pad",
    )
    end_tab.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=_y_cyl_origin(0.123, 0.0, 0.0),
        material=PIN,
        name="tab_hole_liner",
    )

    limits = MotionLimits(effort=3.0, velocity=2.5, lower=-0.6, upper=1.35)
    model.articulation(
        "root_to_middle",
        ArticulationType.REVOLUTE,
        parent=root_link,
        child=middle_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "middle_to_tab",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=end_tab,
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_link = object_model.get_part("root_link")
    middle_link = object_model.get_part("middle_link")
    end_tab = object_model.get_part("end_tab")
    root_joint = object_model.get_articulation("root_to_middle")
    tab_joint = object_model.get_articulation("middle_to_tab")

    revolute_count = sum(
        1
        for articulation in object_model.articulations
        if articulation.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check(
        "chain has exactly two revolute joints",
        revolute_count == 2 and len(object_model.articulations) == 2,
        details=f"revolute_count={revolute_count}, total={len(object_model.articulations)}",
    )
    ctx.check(
        "joint axes are parallel",
        tuple(root_joint.axis) == tuple(tab_joint.axis) == (0.0, -1.0, 0.0),
        details=f"root_axis={root_joint.axis}, tab_axis={tab_joint.axis}",
    )

    ctx.expect_gap(
        root_link,
        middle_link,
        axis="y",
        positive_elem="root_lug_upper",
        negative_elem="first_barrel",
        min_gap=0.0,
        max_gap=0.001,
        name="root upper hinge cheek seats on barrel",
    )
    ctx.expect_gap(
        middle_link,
        root_link,
        axis="y",
        positive_elem="first_barrel",
        negative_elem="root_lug_lower",
        min_gap=0.0,
        max_gap=0.001,
        name="root lower hinge cheek seats on barrel",
    )
    ctx.expect_gap(
        middle_link,
        end_tab,
        axis="y",
        positive_elem="middle_lug_upper",
        negative_elem="second_barrel",
        min_gap=0.0,
        max_gap=0.001,
        name="middle upper hinge cheek seats on barrel",
    )
    ctx.expect_gap(
        end_tab,
        middle_link,
        axis="y",
        positive_elem="second_barrel",
        negative_elem="middle_lug_lower",
        min_gap=0.0,
        max_gap=0.001,
        name="middle lower hinge cheek seats on barrel",
    )
    ctx.expect_overlap(
        root_link,
        middle_link,
        axes="xz",
        elem_a="root_lug_upper",
        elem_b="first_barrel",
        min_overlap=0.040,
        name="first hinge blocks share pivot projection",
    )
    ctx.expect_overlap(
        middle_link,
        end_tab,
        axes="xz",
        elem_a="middle_lug_upper",
        elem_b="second_barrel",
        min_overlap=0.038,
        name="second hinge blocks share pivot projection",
    )

    closed_tab_aabb = ctx.part_element_world_aabb(end_tab, elem="tab_plate")
    with ctx.pose({root_joint: 0.75, tab_joint: 0.60}):
        lifted_tab_aabb = ctx.part_element_world_aabb(end_tab, elem="tab_plate")
    closed_tab_max_z = closed_tab_aabb[1][2] if closed_tab_aabb else None
    lifted_tab_max_z = lifted_tab_aabb[1][2] if lifted_tab_aabb else None
    ctx.check(
        "serial joints lift the end tab in plane",
        closed_tab_max_z is not None
        and lifted_tab_max_z is not None
        and lifted_tab_max_z > closed_tab_max_z + 0.10,
        details=f"closed_max_z={closed_tab_max_z}, lifted_max_z={lifted_tab_max_z}",
    )

    return ctx.report()


object_model = build_object_model()
