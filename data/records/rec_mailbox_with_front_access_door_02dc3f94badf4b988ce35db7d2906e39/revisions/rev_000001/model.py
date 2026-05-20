from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rural_parcel_box")

    body_green = model.material("body_green", rgba=(0.23, 0.34, 0.24, 1.0))
    trim_green = model.material("trim_green", rgba=(0.17, 0.25, 0.18, 1.0))
    stand_black = model.material("stand_black", rgba=(0.12, 0.12, 0.12, 1.0))
    latch_steel = model.material("latch_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    seal_black = model.material("seal_black", rgba=(0.07, 0.07, 0.07, 1.0))

    body_width = 0.52
    body_depth = 0.45
    body_height = 0.84
    wall_thickness = 0.028
    roof_thickness = 0.024
    front_y = body_depth * 0.5

    sill_height = 0.055
    lower_opening_height = 0.54
    rail_height = 0.05
    flap_opening_height = 0.145

    lower_opening_top = sill_height + lower_opening_height
    flap_opening_bottom = lower_opening_top + rail_height
    flap_opening_top = flap_opening_bottom + flap_opening_height

    inner_width = body_width - 2.0 * wall_thickness
    inner_depth = body_depth - 2.0 * wall_thickness

    lower_door_width = inner_width - 0.012
    lower_door_height = lower_opening_height - 0.006
    lower_door_thickness = 0.024

    flap_width = inner_width - 0.012
    flap_height = flap_opening_height - 0.004
    flap_thickness = 0.020

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height + roof_thickness)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, (body_height + roof_thickness) * 0.5)),
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height)),
        origin=Origin(
            xyz=(-body_width * 0.5 + wall_thickness * 0.5, 0.0, body_height * 0.5)
        ),
        material=body_green,
        name="body_left_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height)),
        origin=Origin(
            xyz=(body_width * 0.5 - wall_thickness * 0.5, 0.0, body_height * 0.5)
        ),
        material=body_green,
        name="body_right_wall",
    )
    body.visual(
        Box((inner_width, wall_thickness, body_height)),
        origin=Origin(
            xyz=(0.0, -body_depth * 0.5 + wall_thickness * 0.5, body_height * 0.5)
        ),
        material=body_green,
        name="body_back_wall",
    )
    body.visual(
        Box((inner_width, inner_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, wall_thickness * 0.5)),
        material=trim_green,
        name="body_floor_panel",
    )
    body.visual(
        Box((inner_width, wall_thickness, sill_height)),
        origin=Origin(
            xyz=(0.0, front_y - wall_thickness * 0.5, sill_height * 0.5)
        ),
        material=trim_green,
        name="body_lower_sill",
    )
    body.visual(
        Box((inner_width, wall_thickness, rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - wall_thickness * 0.5,
                lower_opening_top + rail_height * 0.5,
            )
        ),
        material=trim_green,
        name="body_mid_rail",
    )
    body.visual(
        Box((inner_width, wall_thickness, body_height - flap_opening_top)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - wall_thickness * 0.5,
                flap_opening_top + (body_height - flap_opening_top) * 0.5,
            )
        ),
        material=trim_green,
        name="body_top_rail",
    )
    body.visual(
        Box((body_width + 0.02, body_depth + 0.01, roof_thickness)),
        origin=Origin(
            xyz=(0.0, -0.005, body_height + roof_thickness * 0.5),
        ),
        material=body_green,
        name="body_roof_cap",
    )
    body.visual(
        Box((inner_width * 0.96, 0.020, body_height - roof_thickness)),
        origin=Origin(
            xyz=(0.0, -body_depth * 0.5 + wall_thickness + 0.010, body_height * 0.5)
        ),
        material=seal_black,
        name="body_back_liner",
    )

    stand = model.part("stand")
    stand_height = 0.24
    stand_top_width = 0.40
    stand_top_depth = 0.31
    stand_top_thickness = 0.020
    leg_size = 0.045
    lower_brace_height = 0.040
    stand.inertial = Inertial.from_geometry(
        Box((stand_top_width, stand_top_depth, stand_height)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -stand_height * 0.5)),
    )
    stand.visual(
        Box((stand_top_width, stand_top_depth, stand_top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -stand_top_thickness * 0.5)),
        material=stand_black,
        name="stand_top_plate",
    )
    leg_x = stand_top_width * 0.5 - leg_size * 0.5
    leg_y = stand_top_depth * 0.5 - leg_size * 0.5
    leg_center_z = -(stand_height + stand_top_thickness) * 0.5
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            stand.visual(
                Box((leg_size, leg_size, stand_height - stand_top_thickness)),
                origin=Origin(xyz=(sx * leg_x, sy * leg_y, leg_center_z)),
                material=stand_black,
                name=f"stand_leg_{'l' if sx < 0 else 'r'}_{'r' if sy > 0 else 'f'}",
            )
    brace_z = -stand_height + lower_brace_height * 0.5 + 0.018
    stand.visual(
        Box((stand_top_width - 2.0 * leg_size, 0.030, lower_brace_height)),
        origin=Origin(xyz=(0.0, leg_y, brace_z)),
        material=stand_black,
        name="stand_rear_brace",
    )
    stand.visual(
        Box((stand_top_width - 2.0 * leg_size, 0.030, lower_brace_height)),
        origin=Origin(xyz=(0.0, -leg_y, brace_z)),
        material=stand_black,
        name="stand_front_brace",
    )
    stand.visual(
        Box((0.030, stand_top_depth - 2.0 * leg_size, lower_brace_height)),
        origin=Origin(xyz=(-leg_x, 0.0, brace_z)),
        material=stand_black,
        name="stand_left_brace",
    )
    stand.visual(
        Box((0.030, stand_top_depth - 2.0 * leg_size, lower_brace_height)),
        origin=Origin(xyz=(leg_x, 0.0, brace_z)),
        material=stand_black,
        name="stand_right_brace",
    )

    lower_door = model.part("lower_door")
    lower_door.inertial = Inertial.from_geometry(
        Box((lower_door_width, lower_door_thickness + 0.030, lower_door_height)),
        mass=7.0,
        origin=Origin(
            xyz=(0.0, (lower_door_thickness + 0.030) * 0.5, lower_door_height * 0.5)
        ),
    )
    lower_door.visual(
        Box((lower_door_width, lower_door_thickness, lower_door_height)),
        origin=Origin(
            xyz=(0.0, lower_door_thickness * 0.5, lower_door_height * 0.5)
        ),
        material=body_green,
        name="lower_door_panel",
    )
    lower_door.visual(
        Box((lower_door_width - 0.080, 0.010, lower_door_height - 0.090)),
        origin=Origin(
            xyz=(
                0.0,
                lower_door_thickness + 0.005,
                lower_door_height * 0.5 + 0.005,
            )
        ),
        material=trim_green,
        name="lower_door_outer_stiffener",
    )
    lower_door.visual(
        Box((0.130, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, lower_door_thickness + 0.024, lower_door_height * 0.62)),
        material=latch_steel,
        name="lower_door_handle",
    )

    deposit_flap = model.part("deposit_flap")
    deposit_flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_thickness + 0.022, flap_height)),
        mass=2.0,
        origin=Origin(xyz=(0.0, (flap_thickness + 0.022) * 0.5, -flap_height * 0.5)),
    )
    deposit_flap.visual(
        Box((flap_width, flap_thickness, flap_height)),
        origin=Origin(xyz=(0.0, flap_thickness * 0.5, -flap_height * 0.5)),
        material=body_green,
        name="deposit_flap_panel",
    )
    deposit_flap.visual(
        Box((flap_width - 0.060, 0.008, flap_height - 0.045)),
        origin=Origin(
            xyz=(0.0, flap_thickness + 0.004, -flap_height * 0.5 - 0.004)
        ),
        material=trim_green,
        name="deposit_flap_outer_stiffener",
    )
    deposit_flap.visual(
        Box((0.120, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, flap_thickness + 0.018, -flap_height * 0.72)),
        material=latch_steel,
        name="deposit_flap_pull",
    )

    model.articulation(
        "body_to_stand",
        ArticulationType.FIXED,
        parent=body,
        child=stand,
        origin=Origin(),
    )
    model.articulation(
        "body_to_lower_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_door,
        origin=Origin(xyz=(0.0, front_y, sill_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_deposit_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=deposit_flap,
        origin=Origin(xyz=(0.0, front_y, flap_opening_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.25,
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

    body = object_model.get_part("body")
    stand = object_model.get_part("stand")
    lower_door = object_model.get_part("lower_door")
    deposit_flap = object_model.get_part("deposit_flap")

    stand_mount = object_model.get_articulation("body_to_stand")
    lower_hinge = object_model.get_articulation("body_to_lower_door")
    flap_hinge = object_model.get_articulation("body_to_deposit_flap")

    ctx.check(
        "stand mount is fixed",
        stand_mount.articulation_type == ArticulationType.FIXED,
        details=f"type={stand_mount.articulation_type}",
    )
    ctx.check(
        "door hinges are horizontal",
        tuple(lower_hinge.axis) == (-1.0, 0.0, 0.0)
        and tuple(flap_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"lower={lower_hinge.axis}, upper={flap_hinge.axis}",
    )

    ctx.expect_contact(
        body,
        stand,
        elem_b="stand_top_plate",
        name="stand supports insulated body",
    )

    with ctx.pose({lower_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            lower_door,
            body,
            axis="y",
            positive_elem="lower_door_panel",
            negative_elem="body_left_wall",
            max_gap=0.002,
            max_penetration=0.0,
            name="lower retrieval door closes flush to front plane",
        )
        ctx.expect_overlap(
            lower_door,
            body,
            axes="x",
            elem_a="lower_door_panel",
            elem_b="body_mid_rail",
            min_overlap=0.25,
            name="lower retrieval door aligns with body face",
        )
        ctx.expect_gap(
            deposit_flap,
            body,
            axis="y",
            positive_elem="deposit_flap_panel",
            negative_elem="body_right_wall",
            max_gap=0.002,
            max_penetration=0.0,
            name="upper deposit flap closes flush to front plane",
        )
        ctx.expect_overlap(
            deposit_flap,
            body,
            axes="x",
            elem_a="deposit_flap_panel",
            elem_b="body_top_rail",
            min_overlap=0.25,
            name="upper deposit flap aligns with body face",
        )

        closed_lower_aabb = ctx.part_element_world_aabb(lower_door, elem="lower_door_panel")
        closed_flap_aabb = ctx.part_element_world_aabb(
            deposit_flap, elem="deposit_flap_panel"
        )

    with ctx.pose({lower_hinge: lower_hinge.motion_limits.upper}):
        open_lower_aabb = ctx.part_element_world_aabb(lower_door, elem="lower_door_panel")

    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_flap_aabb = ctx.part_element_world_aabb(
            deposit_flap, elem="deposit_flap_panel"
        )

    ctx.check(
        "lower retrieval door drops downward when opened",
        closed_lower_aabb is not None
        and open_lower_aabb is not None
        and open_lower_aabb[1][2] < closed_lower_aabb[1][2] - 0.30
        and open_lower_aabb[1][1] > closed_lower_aabb[1][1] + 0.10,
        details=f"closed={closed_lower_aabb}, open={open_lower_aabb}",
    )
    ctx.check(
        "upper deposit flap swings outward from its top hinge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.08
        and open_flap_aabb[0][2] > closed_flap_aabb[0][2] + 0.04,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
