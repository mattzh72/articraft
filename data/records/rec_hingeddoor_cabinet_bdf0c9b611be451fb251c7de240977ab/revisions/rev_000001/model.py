from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def _add_offset_door(
    part,
    *,
    lower_width: float,
    upper_width: float,
    lower_height: float,
    upper_height: float,
    total_thickness: float,
    core_thickness: float,
    mirror: bool,
    door_material,
    panel_material,
) -> None:
    sign = -1.0 if mirror else 1.0
    overlay = total_thickness - core_thickness
    stile = 0.058
    rail = 0.070
    front_y = -(core_thickness + overlay * 0.5)
    core_y = -(core_thickness * 0.5)
    total_height = lower_height + upper_height

    part.visual(
        Box((lower_width, core_thickness, lower_height)),
        origin=Origin(
            xyz=(sign * lower_width * 0.5, core_y, lower_height * 0.5),
        ),
        material=door_material,
        name="lower_leaf",
    )
    part.visual(
        Box((upper_width, core_thickness, upper_height)),
        origin=Origin(
            xyz=(sign * upper_width * 0.5, core_y, lower_height + upper_height * 0.5),
        ),
        material=door_material,
        name="upper_leaf",
    )

    part.visual(
        Box((stile, overlay, total_height)),
        origin=Origin(
            xyz=(sign * stile * 0.5, front_y, total_height * 0.5),
        ),
        material=door_material,
        name="outer_stile",
    )
    part.visual(
        Box((lower_width, overlay, rail)),
        origin=Origin(
            xyz=(sign * lower_width * 0.5, front_y, rail * 0.5),
        ),
        material=door_material,
        name="bottom_rail",
    )
    part.visual(
        Box((lower_width, overlay, rail)),
        origin=Origin(
            xyz=(sign * lower_width * 0.5, front_y, lower_height - rail * 0.5),
        ),
        material=door_material,
        name="mid_rail",
    )
    part.visual(
        Box((upper_width, overlay, rail)),
        origin=Origin(
            xyz=(sign * upper_width * 0.5, front_y, total_height - rail * 0.5),
        ),
        material=door_material,
        name="top_rail",
    )
    part.visual(
        Box((stile, overlay, lower_height)),
        origin=Origin(
            xyz=(sign * (lower_width - stile * 0.5), front_y, lower_height * 0.5),
        ),
        material=door_material,
        name="lower_inner_stile",
    )
    part.visual(
        Box((stile, overlay, upper_height)),
        origin=Origin(
            xyz=(sign * (upper_width - stile * 0.5), front_y, lower_height + upper_height * 0.5),
        ),
        material=door_material,
        name="upper_inner_stile",
    )

    lower_panel_w = max(lower_width - 2.0 * stile, 0.06)
    lower_panel_h = max(lower_height - 2.0 * rail, 0.06)
    upper_panel_w = max(upper_width - 2.0 * stile, 0.05)
    upper_panel_h = max(upper_height - rail - 0.040, 0.05)
    recess_center_y = -(core_thickness * 0.45)

    part.visual(
        Box((lower_panel_w, core_thickness * 0.55, lower_panel_h)),
        origin=Origin(
            xyz=(sign * lower_width * 0.5, recess_center_y, lower_height * 0.5),
        ),
        material=panel_material,
        name="lower_recess_panel",
    )
    part.visual(
        Box((upper_panel_w, core_thickness * 0.55, upper_panel_h)),
        origin=Origin(
            xyz=(
                sign * upper_width * 0.5,
                recess_center_y,
                lower_height + upper_height * 0.5 - rail * 0.18,
            ),
        ),
        material=panel_material,
        name="upper_recess_panel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_sink_cabinet")

    painted_white = model.material("painted_white", rgba=(0.93, 0.92, 0.89, 1.0))
    warm_panel = model.material("warm_panel", rgba=(0.87, 0.84, 0.79, 1.0))
    interior_melamine = model.material("interior_melamine", rgba=(0.95, 0.94, 0.90, 1.0))
    toe_kick_gray = model.material("toe_kick_gray", rgba=(0.39, 0.39, 0.37, 1.0))

    cabinet_width = 0.91
    cabinet_depth = 0.58
    cabinet_height = 0.86
    panel_t = 0.019
    back_t = 0.006
    toe_kick_h = 0.10
    toe_recess = 0.075
    false_rail_z = 0.74
    false_rail_h = 0.085
    door_t = 0.019
    door_core_t = 0.014
    door_bottom_z = 0.105
    door_height = false_rail_z - door_bottom_z - 0.006
    upper_cut_segment_h = 0.182
    lower_segment_h = door_height - upper_cut_segment_h
    lower_leaf_w = 0.440
    upper_leaf_w = 0.305
    hinge_x = cabinet_width * 0.5 - panel_t * 0.5

    carcass = model.part("carcass")
    carcass.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=38.0,
        origin=Origin(xyz=(0.0, cabinet_depth * 0.5, cabinet_height * 0.5)),
    )

    side_upper_h = cabinet_height - toe_kick_h
    plinth_depth = cabinet_depth - toe_recess
    inner_width = cabinet_width - 2.0 * panel_t

    carcass.visual(
        Box((panel_t, cabinet_depth, side_upper_h)),
        origin=Origin(
            xyz=(-cabinet_width * 0.5 + panel_t * 0.5, cabinet_depth * 0.5, toe_kick_h + side_upper_h * 0.5),
        ),
        material=painted_white,
        name="left_side_upper",
    )
    carcass.visual(
        Box((panel_t, cabinet_depth, side_upper_h)),
        origin=Origin(
            xyz=(cabinet_width * 0.5 - panel_t * 0.5, cabinet_depth * 0.5, toe_kick_h + side_upper_h * 0.5),
        ),
        material=painted_white,
        name="right_side_upper",
    )
    carcass.visual(
        Box((panel_t, plinth_depth, toe_kick_h)),
        origin=Origin(
            xyz=(-cabinet_width * 0.5 + panel_t * 0.5, toe_recess + plinth_depth * 0.5, toe_kick_h * 0.5),
        ),
        material=interior_melamine,
        name="left_side_plinth",
    )
    carcass.visual(
        Box((panel_t, plinth_depth, toe_kick_h)),
        origin=Origin(
            xyz=(cabinet_width * 0.5 - panel_t * 0.5, toe_recess + plinth_depth * 0.5, toe_kick_h * 0.5),
        ),
        material=interior_melamine,
        name="right_side_plinth",
    )
    carcass.visual(
        Box((inner_width, cabinet_depth - toe_recess, panel_t)),
        origin=Origin(
            xyz=(0.0, toe_recess + (cabinet_depth - toe_recess) * 0.5, toe_kick_h + panel_t * 0.5),
        ),
        material=interior_melamine,
        name="cabinet_floor",
    )
    carcass.visual(
        Box((inner_width, back_t, side_upper_h)),
        origin=Origin(
            xyz=(0.0, cabinet_depth - back_t * 0.5, toe_kick_h + side_upper_h * 0.5),
        ),
        material=interior_melamine,
        name="back_panel",
    )
    carcass.visual(
        Box((inner_width, panel_t, toe_kick_h)),
        origin=Origin(
            xyz=(0.0, toe_recess + panel_t * 0.5, toe_kick_h * 0.5),
        ),
        material=toe_kick_gray,
        name="toe_kick",
    )
    carcass.visual(
        Box((inner_width, panel_t, false_rail_h)),
        origin=Origin(
            xyz=(0.0, panel_t * 0.5, false_rail_z + false_rail_h * 0.5),
        ),
        material=painted_white,
        name="false_rail",
    )
    carcass.visual(
        Box((inner_width, panel_t, panel_t)),
        origin=Origin(
            xyz=(0.0, cabinet_depth - panel_t * 0.5, cabinet_height - panel_t * 0.5),
        ),
        material=painted_white,
        name="rear_top_rail",
    )
    carcass.visual(
        Box((0.14, cabinet_depth - 0.16, panel_t)),
        origin=Origin(
            xyz=(
                -cabinet_width * 0.5 + panel_t + 0.07,
                0.08 + (cabinet_depth - 0.16) * 0.5,
                cabinet_height - panel_t * 0.5,
            ),
        ),
        material=painted_white,
        name="left_top_stretcher",
    )
    carcass.visual(
        Box((0.14, cabinet_depth - 0.16, panel_t)),
        origin=Origin(
            xyz=(
                cabinet_width * 0.5 - panel_t - 0.07,
                0.08 + (cabinet_depth - 0.16) * 0.5,
                cabinet_height - panel_t * 0.5,
            ),
        ),
        material=painted_white,
        name="right_top_stretcher",
    )

    left_door = model.part("left_door")
    left_door.inertial = Inertial.from_geometry(
        Box((lower_leaf_w, door_t, door_height)),
        mass=5.8,
        origin=Origin(xyz=(lower_leaf_w * 0.5, -door_t * 0.5, door_height * 0.5)),
    )
    _add_offset_door(
        left_door,
        lower_width=lower_leaf_w,
        upper_width=upper_leaf_w,
        lower_height=lower_segment_h,
        upper_height=upper_cut_segment_h,
        total_thickness=door_t,
        core_thickness=door_core_t,
        mirror=False,
        door_material=painted_white,
        panel_material=warm_panel,
    )

    right_door = model.part("right_door")
    right_door.inertial = Inertial.from_geometry(
        Box((lower_leaf_w, door_t, door_height)),
        mass=5.8,
        origin=Origin(xyz=(-lower_leaf_w * 0.5, -door_t * 0.5, door_height * 0.5)),
    )
    _add_offset_door(
        right_door,
        lower_width=lower_leaf_w,
        upper_width=upper_leaf_w,
        lower_height=lower_segment_h,
        upper_height=upper_cut_segment_h,
        total_thickness=door_t,
        core_thickness=door_core_t,
        mirror=True,
        door_material=painted_white,
        panel_material=warm_panel,
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(-hinge_x, 0.0, door_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-math.radians(110.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(hinge_x, 0.0, door_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.check(
        "door hinges are vertical side pivots",
        left_hinge.axis == (0.0, 0.0, 1.0)
        and right_hinge.axis == (0.0, 0.0, 1.0)
        and left_hinge.origin.xyz[0] < -0.40
        and right_hinge.origin.xyz[0] > 0.40,
        details=(
            f"left origin={left_hinge.origin.xyz}, axis={left_hinge.axis}; "
            f"right origin={right_hinge.origin.xyz}, axis={right_hinge.axis}"
        ),
    )

    ctx.expect_contact(left_door, carcass, contact_tol=0.0005, name="left door mounted to carcass")
    ctx.expect_contact(right_door, carcass, contact_tol=0.0005, name="right door mounted to carcass")

    ctx.expect_gap(
        carcass,
        left_door,
        axis="z",
        positive_elem="false_rail",
        negative_elem="upper_leaf",
        min_gap=0.004,
        max_gap=0.010,
        name="false rail sits just above left offset leaf",
    )
    ctx.expect_gap(
        carcass,
        right_door,
        axis="z",
        positive_elem="false_rail",
        negative_elem="upper_leaf",
        min_gap=0.004,
        max_gap=0.010,
        name="false rail sits just above right offset leaf",
    )

    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        positive_elem="lower_leaf",
        negative_elem="lower_leaf",
        min_gap=0.008,
        max_gap=0.016,
        name="lower door leaves nearly meet at center",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        positive_elem="upper_leaf",
        negative_elem="upper_leaf",
        min_gap=0.26,
        max_gap=0.29,
        name="upper leaves preserve center sink cutout",
    )

    left_closed = ctx.part_element_world_aabb(left_door, elem="lower_leaf")
    right_closed = ctx.part_element_world_aabb(right_door, elem="lower_leaf")
    if left_closed is None or right_closed is None:
        ctx.fail("door visual bounds available", "Could not resolve lower leaf AABBs.")
    else:
        with ctx.pose({left_hinge: -math.radians(75.0)}):
            left_open = ctx.part_element_world_aabb(left_door, elem="lower_leaf")
            if left_open is None:
                ctx.fail("left door open pose bounds available", "Missing left door open-pose AABB.")
            else:
                ctx.check(
                    "left door swings outward",
                    left_open[0][1] < left_closed[0][1] - 0.22,
                    details=f"closed min y={left_closed[0][1]:.4f}, open min y={left_open[0][1]:.4f}",
                )
            ctx.expect_contact(
                left_door,
                carcass,
                contact_tol=0.0005,
                name="left door stays captured on outer hinge when open",
            )

        with ctx.pose({right_hinge: math.radians(75.0)}):
            right_open = ctx.part_element_world_aabb(right_door, elem="lower_leaf")
            if right_open is None:
                ctx.fail("right door open pose bounds available", "Missing right door open-pose AABB.")
            else:
                ctx.check(
                    "right door swings outward",
                    right_open[0][1] < right_closed[0][1] - 0.22,
                    details=f"closed min y={right_closed[0][1]:.4f}, open min y={right_open[0][1]:.4f}",
                )
            ctx.expect_contact(
                right_door,
                carcass,
                contact_tol=0.0005,
                name="right door stays captured on outer hinge when open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
