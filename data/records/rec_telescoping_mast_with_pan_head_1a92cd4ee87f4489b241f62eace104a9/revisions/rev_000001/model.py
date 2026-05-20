from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_tube(outer_radius: float, inner_radius: float, z_min: float, z_max: float):
    """Thin-walled cylindrical sleeve with open through bore."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _sleeve_with_collar(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    collar_radius: float,
    collar_height: float,
):
    """Main telescoping sleeve with a thicker lower bearing collar."""
    sleeve = _annular_tube(outer_radius, inner_radius, z_min, z_max)
    sleeve.merge(
        _annular_tube(collar_radius, inner_radius, z_min, z_min + collar_height)
    )
    return sleeve


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_telescoping_pan_pole")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.04, 0.045, 0.05, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    satin_plate = model.material("satin_plate", rgba=(0.34, 0.36, 0.38, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.30, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=satin_plate,
        name="mount_plate",
    )
    top_support.visual(
        mesh_from_geometry(
            _annular_tube(0.044, 0.035, -0.200, 0.0), "top_hanging_collar"
        ),
        material=brushed_steel,
        name="hanging_collar",
    )
    for ix, x in enumerate((-0.105, 0.105)):
        for iy, y in enumerate((-0.060, 0.060)):
            top_support.visual(
                Cylinder(radius=0.010, length=0.005),
                origin=Origin(xyz=(x, y, 0.0275)),
                material=brushed_steel,
                name=f"screw_head_{ix}_{iy}",
            )

    outer_member = model.part("outer_member")
    outer_member.visual(
        mesh_from_geometry(
            _sleeve_with_collar(
                outer_radius=0.030,
                inner_radius=0.024,
                z_min=-0.420,
                z_max=0.160,
                collar_radius=0.034,
                collar_height=0.050,
            ),
            "outer_member_sleeve",
        ),
        material=dark_anodized,
        name="outer_sleeve",
    )
    for idx, (x, y) in enumerate(
        ((0.032, 0.0), (0.0, 0.032), (-0.032, 0.0), (0.0, -0.032))
    ):
        outer_member.visual(
            Sphere(radius=0.003),
            origin=Origin(xyz=(x, y, 0.140)),
            material=brushed_steel,
            name=f"outer_guide_{idx}",
        )

    middle_member = model.part("middle_member")
    middle_sleeve = _sleeve_with_collar(
        outer_radius=0.021,
        inner_radius=0.016,
        z_min=-0.240,
        z_max=0.220,
        collar_radius=0.024,
        collar_height=0.040,
    )
    middle_member.visual(
        mesh_from_geometry(middle_sleeve, "middle_member_sleeve"),
        material=matte_black,
        name="middle_sleeve",
    )
    for idx, (x, y) in enumerate(
        ((0.021, 0.0), (0.0, 0.021), (-0.021, 0.0), (0.0, -0.021))
    ):
        middle_member.visual(
            Sphere(radius=0.003),
            origin=Origin(xyz=(x, y, 0.200)),
            material=brushed_steel,
            name=f"middle_guide_{idx}",
        )

    inner_member = model.part("inner_member")
    inner_member.visual(
        Cylinder(radius=0.013, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=brushed_steel,
        name="inner_rod",
    )
    for idx, (x, y) in enumerate(
        ((0.013, 0.0), (0.0, 0.013), (-0.013, 0.0), (0.0, -0.013))
    ):
        inner_member.visual(
            Sphere(radius=0.003),
            origin=Origin(xyz=(x, y, 0.200)),
            material=brushed_steel,
            name=f"inner_guide_{idx}",
        )

    pan_plate = model.part("pan_plate")
    pan_plate.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=brushed_steel,
        name="bearing_hub",
    )
    pan_plate.visual(
        Cylinder(radius=0.085, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        material=matte_black,
        name="round_plate",
    )
    pan_plate.visual(
        Box((0.045, 0.024, 0.009)),
        origin=Origin(xyz=(0.088, 0.0, -0.034)),
        material=matte_black,
        name="index_tab",
    )

    model.articulation(
        "top_to_outer",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=outer_member,
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.10),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_member,
        child=middle_member,
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.20, lower=0.0, upper=0.16),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_member,
        child=inner_member,
        origin=Origin(xyz=(0.0, 0.0, -0.240)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.22, lower=0.0, upper=0.14),
    )
    model.articulation(
        "inner_to_pan_plate",
        ArticulationType.REVOLUTE,
        parent=inner_member,
        child=pan_plate,
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=1.5, lower=-math.pi, upper=math.pi
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    top_to_outer = object_model.get_articulation("top_to_outer")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_pan = object_model.get_articulation("inner_to_pan_plate")

    top_support = object_model.get_part("top_support")
    outer_member = object_model.get_part("outer_member")
    middle_member = object_model.get_part("middle_member")
    inner_member = object_model.get_part("inner_member")
    pan_plate = object_model.get_part("pan_plate")

    ctx.check(
        "serial vertical prismatic joints",
        top_to_outer.articulation_type == ArticulationType.PRISMATIC
        and outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and top_to_outer.axis == (0.0, 0.0, -1.0)
        and outer_to_middle.axis == (0.0, 0.0, -1.0)
        and middle_to_inner.axis == (0.0, 0.0, -1.0),
        details="The telescoping members should slide serially downward from the top support.",
    )
    ctx.check(
        "pan plate revolute joint",
        inner_to_pan.articulation_type == ArticulationType.REVOLUTE
        and inner_to_pan.axis == (0.0, 0.0, 1.0),
        details="The compact plate should rotate about the pole axis.",
    )

    ctx.expect_within(
        outer_member,
        top_support,
        axes="xy",
        inner_elem="outer_sleeve",
        outer_elem="hanging_collar",
        margin=0.002,
        name="outer member nests in top collar",
    )
    ctx.expect_overlap(
        outer_member,
        top_support,
        axes="z",
        elem_a="outer_sleeve",
        elem_b="hanging_collar",
        min_overlap=0.10,
        name="outer member retained by top support",
    )
    ctx.expect_within(
        middle_member,
        outer_member,
        axes="xy",
        inner_elem="middle_sleeve",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="middle member nests in outer member",
    )
    ctx.expect_overlap(
        middle_member,
        outer_member,
        axes="z",
        elem_a="middle_sleeve",
        elem_b="outer_sleeve",
        min_overlap=0.12,
        name="middle member retained in outer member",
    )
    ctx.expect_within(
        inner_member,
        middle_member,
        axes="xy",
        inner_elem="inner_rod",
        outer_elem="middle_sleeve",
        margin=0.002,
        name="inner member nests in middle member",
    )
    ctx.expect_overlap(
        inner_member,
        middle_member,
        axes="z",
        elem_a="inner_rod",
        elem_b="middle_sleeve",
        min_overlap=0.18,
        name="inner member retained in middle member",
    )
    ctx.expect_contact(
        pan_plate,
        inner_member,
        elem_a="bearing_hub",
        elem_b="inner_rod",
        contact_tol=0.001,
        name="pan bearing hub is seated on inner member",
    )

    rest_pan_pos = ctx.part_world_position(pan_plate)
    rest_outer_pos = ctx.part_world_position(outer_member)
    rest_middle_pos = ctx.part_world_position(middle_member)
    rest_inner_pos = ctx.part_world_position(inner_member)
    rest_tab_box = ctx.part_element_world_aabb(pan_plate, elem="index_tab")

    with ctx.pose(
        {
            top_to_outer: 0.10,
            outer_to_middle: 0.16,
            middle_to_inner: 0.14,
            inner_to_pan: math.pi / 2.0,
        }
    ):
        ctx.expect_overlap(
            outer_member,
            top_support,
            axes="z",
            elem_a="outer_sleeve",
            elem_b="hanging_collar",
            min_overlap=0.050,
            name="extended outer member remains captured",
        )
        ctx.expect_overlap(
            middle_member,
            outer_member,
            axes="z",
            elem_a="middle_sleeve",
            elem_b="outer_sleeve",
            min_overlap=0.050,
            name="extended middle member remains captured",
        )
        ctx.expect_overlap(
            inner_member,
            middle_member,
            axes="z",
            elem_a="inner_rod",
            elem_b="middle_sleeve",
            min_overlap=0.080,
            name="extended inner member remains captured",
        )
        extended_pan_pos = ctx.part_world_position(pan_plate)
        extended_outer_pos = ctx.part_world_position(outer_member)
        extended_middle_pos = ctx.part_world_position(middle_member)
        extended_inner_pos = ctx.part_world_position(inner_member)
        turned_tab_box = ctx.part_element_world_aabb(pan_plate, elem="index_tab")

    ctx.check(
        "nested stages extend downward",
        rest_outer_pos is not None
        and rest_middle_pos is not None
        and rest_inner_pos is not None
        and extended_outer_pos is not None
        and extended_middle_pos is not None
        and extended_inner_pos is not None
        and extended_outer_pos[2] < rest_outer_pos[2] - 0.09
        and extended_middle_pos[2] < rest_middle_pos[2] - 0.24
        and extended_inner_pos[2] < rest_inner_pos[2] - 0.38,
        details=(
            f"rest={(rest_outer_pos, rest_middle_pos, rest_inner_pos)}, "
            f"extended={(extended_outer_pos, extended_middle_pos, extended_inner_pos)}"
        ),
    )
    ctx.check(
        "pan plate stays on pole axis while rotating",
        rest_pan_pos is not None
        and extended_pan_pos is not None
        and abs(rest_pan_pos[0] - extended_pan_pos[0]) < 0.001
        and abs(rest_pan_pos[1] - extended_pan_pos[1]) < 0.001,
        details=f"rest={rest_pan_pos}, extended={extended_pan_pos}",
    )
    ctx.check(
        "asymmetric pan tab visibly rotates",
        rest_tab_box is not None
        and turned_tab_box is not None
        and (rest_tab_box[1][0] - rest_tab_box[0][0]) > (
            rest_tab_box[1][1] - rest_tab_box[0][1]
        )
        and (turned_tab_box[1][1] - turned_tab_box[0][1]) > (
            turned_tab_box[1][0] - turned_tab_box[0][0]
        ),
        details=f"rest_tab={rest_tab_box}, turned_tab={turned_tab_box}",
    )

    return ctx.report()


object_model = build_object_model()
