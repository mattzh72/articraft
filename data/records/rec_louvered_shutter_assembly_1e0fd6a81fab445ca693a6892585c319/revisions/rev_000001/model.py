from __future__ import annotations

from math import pi

import cadquery as cq
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
    mesh_from_cadquery,
)


PANEL_WIDTH = 1.36
PANEL_HEIGHT = 1.50
PANEL_DEPTH = 0.045
HINGE_Y = -0.040
PANEL_CENTER_Y = 0.040


def _louver_blade_shape(length: float) -> cq.Workplane:
    """A plantation-shutter slat: long, rounded, and slightly airfoil-like."""
    return cq.Workplane("YZ").ellipse(0.018, 0.052).extrude(length * 0.5, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_plantation_shutter")

    painted_white = model.material("painted_white", rgba=(0.88, 0.86, 0.80, 1.0))
    warm_white = model.material("warm_white", rgba=(0.95, 0.93, 0.87, 1.0))
    shadow = model.material("shadow", rgba=(0.18, 0.17, 0.15, 1.0))
    hinge_metal = model.material("satin_hinge_metal", rgba=(0.58, 0.54, 0.46, 1.0))

    opening = model.part("opening_frame")
    opening.visual(
        Box((0.11, 0.080, PANEL_HEIGHT + 0.24)),
        origin=Origin(xyz=(-0.065, 0.0, 0.0)),
        material=warm_white,
        name="hinge_jamb",
    )
    opening.visual(
        Box((0.11, 0.080, PANEL_HEIGHT + 0.24)),
        origin=Origin(xyz=(PANEL_WIDTH + 0.065, 0.0, 0.0)),
        material=warm_white,
        name="latch_jamb",
    )
    opening.visual(
        Box((PANEL_WIDTH + 0.24, 0.080, 0.11)),
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, 0.0, PANEL_HEIGHT * 0.5 + 0.065)),
        material=warm_white,
        name="head_jamb",
    )
    opening.visual(
        Box((PANEL_WIDTH + 0.24, 0.080, 0.11)),
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, 0.0, -PANEL_HEIGHT * 0.5 - 0.065)),
        material=warm_white,
        name="sill_jamb",
    )
    # Dark return surfaces inside the wall opening make the single shutter panel
    # read as hanging in a real aperture instead of floating in space.
    opening.visual(
        Box((0.018, 0.022, PANEL_HEIGHT + 0.02)),
        origin=Origin(xyz=(-0.004, 0.044, 0.0)),
        material=shadow,
        name="hinge_reveal",
    )
    opening.visual(
        Box((0.018, 0.022, PANEL_HEIGHT + 0.02)),
        origin=Origin(xyz=(PANEL_WIDTH + 0.004, 0.044, 0.0)),
        material=shadow,
        name="latch_reveal",
    )
    for index, z in enumerate((0.50, 0.0, -0.50)):
        opening.visual(
            Box((0.032, 0.010, 0.22)),
            origin=Origin(xyz=(-0.014, HINGE_Y + 0.014, z)),
            material=hinge_metal,
            name=f"frame_hinge_leaf_{index}",
        )
        opening.visual(
            Cylinder(radius=0.014, length=0.16),
            origin=Origin(xyz=(0.0, HINGE_Y, z)),
            material=hinge_metal,
            name=f"frame_hinge_knuckle_{index}",
        )

    panel = model.part("panel_frame")
    stile_w = 0.10
    rail_h = 0.105
    mullion_w = 0.070
    panel.visual(
        Box((stile_w, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.055, PANEL_CENTER_Y, 0.0)),
        material=painted_white,
        name="hinge_stile",
    )
    panel.visual(
        Box((stile_w, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(PANEL_WIDTH - 0.050, PANEL_CENTER_Y, 0.0)),
        material=painted_white,
        name="latch_stile",
    )
    panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, rail_h)),
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, PANEL_CENTER_Y, PANEL_HEIGHT * 0.5 - rail_h * 0.5)),
        material=painted_white,
        name="top_rail",
    )
    panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, rail_h)),
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, PANEL_CENTER_Y, -PANEL_HEIGHT * 0.5 + rail_h * 0.5)),
        material=painted_white,
        name="bottom_rail",
    )
    panel.visual(
        Box((mullion_w, PANEL_DEPTH, PANEL_HEIGHT - 2.0 * rail_h + 0.016)),
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, PANEL_CENTER_Y, 0.0)),
        material=painted_white,
        name="center_mullion",
    )

    # Shallow shadowed stops around the two louver banks emphasize that there is
    # one broad framed door, not two separate shutters.
    left_min, left_max = 0.105, PANEL_WIDTH * 0.5 - mullion_w * 0.5
    right_min, right_max = PANEL_WIDTH * 0.5 + mullion_w * 0.5, PANEL_WIDTH - 0.105
    open_z_min, open_z_max = -PANEL_HEIGHT * 0.5 + rail_h + 0.012, PANEL_HEIGHT * 0.5 - rail_h - 0.012
    for bank_name, x_min, x_max in (("left", left_min, left_max), ("right", right_min, right_max)):
        bank_center = (x_min + x_max) * 0.5
        bank_width = x_max - x_min
        panel.visual(
            Box((bank_width, 0.007, 0.012)),
            origin=Origin(xyz=(bank_center, PANEL_CENTER_Y - PANEL_DEPTH * 0.5 - 0.002, open_z_max)),
            material=shadow,
            name=f"{bank_name}_top_shadow",
        )
        panel.visual(
            Box((bank_width, 0.007, 0.012)),
            origin=Origin(xyz=(bank_center, PANEL_CENTER_Y - PANEL_DEPTH * 0.5 - 0.002, open_z_min)),
            material=shadow,
            name=f"{bank_name}_bottom_shadow",
        )
        panel.visual(
            Box((0.012, 0.007, open_z_max - open_z_min)),
            origin=Origin(xyz=(x_min, PANEL_CENTER_Y - PANEL_DEPTH * 0.5 - 0.002, 0.0)),
            material=shadow,
            name=f"{bank_name}_side_shadow_0",
        )
        panel.visual(
            Box((0.012, 0.007, open_z_max - open_z_min)),
            origin=Origin(xyz=(x_max, PANEL_CENTER_Y - PANEL_DEPTH * 0.5 - 0.002, 0.0)),
            material=shadow,
            name=f"{bank_name}_side_shadow_1",
        )

    for index, z in enumerate((0.25, -0.25)):
        panel.visual(
            Box((0.050, 0.028, 0.24)),
            origin=Origin(xyz=(0.020, 0.012, z)),
            material=hinge_metal,
            name=f"panel_hinge_leaf_{index}",
        )
        panel.visual(
            Cylinder(radius=0.014, length=0.20),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_metal,
            name=f"panel_hinge_knuckle_{index}",
        )

    louver_length = (left_max - left_min) - 0.055
    pin_length = left_max - left_min
    louver_mesh = mesh_from_cadquery(_louver_blade_shape(louver_length), "rounded_louver_blade")
    pivot_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    louver_positions = [-0.525, -0.375, -0.225, -0.075, 0.075, 0.225, 0.375, 0.525]
    bank_specs = (
        ("left", (left_min + left_max) * 0.5),
        ("right", (right_min + right_max) * 0.5),
    )
    bank_master_joint: dict[str, str] = {}

    for bank_name, x_center in bank_specs:
        for i, z in enumerate(louver_positions):
            louver = model.part(f"{bank_name}_louver_{i}")
            louver.visual(
                louver_mesh,
                origin=Origin(),
                material=painted_white,
                name="blade",
            )
            louver.visual(
                Cylinder(radius=0.007, length=pin_length),
                origin=pivot_origin,
                material=hinge_metal,
                name="pivot_pin",
            )
            joint_name = f"{bank_name}_louver_{i}_pivot"
            mimic = None
            if i == 0:
                bank_master_joint[bank_name] = joint_name
            else:
                mimic = Mimic(joint=bank_master_joint[bank_name], multiplier=1.0, offset=0.0)
            model.articulation(
                joint_name,
                ArticulationType.REVOLUTE,
                parent=panel,
                child=louver,
                origin=Origin(xyz=(x_center, PANEL_CENTER_Y, z)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-1.05, upper=1.05),
                mimic=mimic,
            )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=opening,
        child=panel,
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    opening = object_model.get_part("opening_frame")
    panel = object_model.get_part("panel_frame")
    left_louver = object_model.get_part("left_louver_0")
    left_follower = object_model.get_part("left_louver_4")
    right_louver = object_model.get_part("right_louver_0")
    right_follower = object_model.get_part("right_louver_4")
    panel_hinge = object_model.get_articulation("panel_hinge")
    left_pivot = object_model.get_articulation("left_louver_0_pivot")
    right_pivot = object_model.get_articulation("right_louver_0_pivot")

    ctx.expect_within(
        left_louver,
        panel,
        axes="xz",
        margin=0.02,
        inner_elem="blade",
        name="left louver bank is framed by the panel",
    )
    ctx.expect_within(
        right_louver,
        panel,
        axes="xz",
        margin=0.02,
        inner_elem="blade",
        name="right louver bank is framed by the panel",
    )
    ctx.expect_origin_gap(
        right_louver,
        left_louver,
        axis="x",
        min_gap=0.50,
        max_gap=0.80,
        name="two louver banks are side by side",
    )

    rest_stile = ctx.part_element_world_aabb(panel, elem="latch_stile")
    with ctx.pose({panel_hinge: 1.05}):
        open_stile = ctx.part_element_world_aabb(panel, elem="latch_stile")
    ctx.check(
        "single shutter panel swings outward on its side hinge",
        rest_stile is not None
        and open_stile is not None
        and open_stile[1][1] > rest_stile[1][1] + 0.75,
        details=f"rest={rest_stile}, opened={open_stile}",
    )

    def _elem_y_size(part_name: str, elem: str = "blade") -> float | None:
        aabb = ctx.part_element_world_aabb(object_model.get_part(part_name), elem=elem)
        if aabb is None:
            return None
        return aabb[1][1] - aabb[0][1]

    left_rest = _elem_y_size("left_louver_0")
    left_follow_rest = _elem_y_size("left_louver_4")
    with ctx.pose({left_pivot: 0.80}):
        left_tilted = _elem_y_size("left_louver_0")
        left_follow_tilted = _elem_y_size("left_louver_4")
    ctx.check(
        "left bank louvers rotate together around horizontal pivots",
        left_rest is not None
        and left_tilted is not None
        and left_follow_rest is not None
        and left_follow_tilted is not None
        and left_tilted > left_rest + 0.035
        and left_follow_tilted > left_follow_rest + 0.035,
        details=(
            f"master rest/tilted={left_rest}/{left_tilted}, "
            f"follower rest/tilted={left_follow_rest}/{left_follow_tilted}"
        ),
    )

    right_rest = _elem_y_size("right_louver_0")
    right_follow_rest = _elem_y_size("right_louver_4")
    with ctx.pose({right_pivot: 0.80}):
        right_tilted = _elem_y_size("right_louver_0")
        right_follow_tilted = _elem_y_size("right_louver_4")
    ctx.check(
        "right bank louvers rotate together around horizontal pivots",
        right_rest is not None
        and right_tilted is not None
        and right_follow_rest is not None
        and right_follow_tilted is not None
        and right_tilted > right_rest + 0.035
        and right_follow_tilted > right_follow_rest + 0.035,
        details=(
            f"master rest/tilted={right_rest}/{right_tilted}, "
            f"follower rest/tilted={right_follow_rest}/{right_follow_tilted}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
