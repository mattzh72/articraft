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
    model = ArticulatedObject(name="watch_winder_presentation_box")

    piano_black = model.material("piano_black", rgba=(0.08, 0.08, 0.09, 1.0))
    graphite_trim = model.material("graphite_trim", rgba=(0.20, 0.21, 0.23, 1.0))
    warm_suede = model.material("warm_suede", rgba=(0.63, 0.56, 0.46, 1.0))
    charcoal_fabric = model.material("charcoal_fabric", rgba=(0.18, 0.18, 0.19, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.68, 0.70, 0.72, 1.0))

    outer_w = 0.190
    outer_d = 0.150
    body_h = 0.095
    wall = 0.012
    floor_th = 0.010

    lid_w = 0.194
    lid_d = 0.154
    lid_top_th = 0.008
    lid_skirt_th = 0.009
    lid_skirt_drop = 0.029

    box_body = model.part("box_body")
    box_body.visual(
        Box((outer_w, outer_d, floor_th)),
        origin=Origin(xyz=(0.0, 0.0, floor_th * 0.5)),
        material=piano_black,
        name="outer_floor",
    )
    side_wall_h = body_h - floor_th
    wall_center_z = floor_th + side_wall_h * 0.5
    box_body.visual(
        Box((wall, outer_d, side_wall_h)),
        origin=Origin(xyz=(-(outer_w - wall) * 0.5, 0.0, wall_center_z)),
        material=piano_black,
        name="left_wall",
    )
    box_body.visual(
        Box((wall, outer_d, side_wall_h)),
        origin=Origin(xyz=((outer_w - wall) * 0.5, 0.0, wall_center_z)),
        material=piano_black,
        name="right_wall",
    )
    box_body.visual(
        Box((outer_w - 2.0 * wall, wall, side_wall_h)),
        origin=Origin(xyz=(0.0, (outer_d - wall) * 0.5, wall_center_z)),
        material=piano_black,
        name="front_wall",
    )
    box_body.visual(
        Box((outer_w - 2.0 * wall, wall, side_wall_h)),
        origin=Origin(xyz=(0.0, -(outer_d - wall) * 0.5, wall_center_z)),
        material=piano_black,
        name="rear_wall",
    )
    box_body.visual(
        Box((outer_w - 0.030, outer_d - 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.002, floor_th + 0.002)),
        material=warm_suede,
        name="inner_base_liner",
    )
    box_body.visual(
        Box((outer_w - 2.0 * wall - 0.010, 0.016, 0.044)),
        origin=Origin(xyz=(0.0, -(outer_d * 0.5) + wall + 0.008, 0.048)),
        material=graphite_trim,
        name="rear_motor_housing",
    )
    box_body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_top_th)),
        origin=Origin(xyz=(0.0, lid_d * 0.5, 0.0)),
        material=piano_black,
        name="outer_panel",
    )
    lid.visual(
        Box((lid_skirt_th, lid_d, lid_skirt_drop)),
        origin=Origin(xyz=(-(outer_w * 0.5 + lid_skirt_th * 0.5 + 0.001), lid_d * 0.5, -(lid_skirt_drop * 0.5 + lid_top_th * 0.5))),
        material=piano_black,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_skirt_th, lid_d, lid_skirt_drop)),
        origin=Origin(xyz=((outer_w * 0.5 + lid_skirt_th * 0.5 + 0.001), lid_d * 0.5, -(lid_skirt_drop * 0.5 + lid_top_th * 0.5))),
        material=piano_black,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_w, lid_skirt_th, lid_skirt_drop)),
        origin=Origin(xyz=(0.0, outer_d + lid_skirt_th * 0.5 + 0.001, -(lid_skirt_drop * 0.5 + lid_top_th * 0.5))),
        material=piano_black,
        name="front_skirt",
    )
    lid.visual(
        Box((outer_w - 0.020, outer_d - 0.028, 0.004)),
        origin=Origin(xyz=(0.0, lid_d * 0.5 - 0.002, -0.006)),
        material=warm_suede,
        name="inner_liner",
    )
    lid.visual(
        Box((0.050, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, lid_d + 0.004, -0.010)),
        material=satin_metal,
        name="front_pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_top_th + lid_skirt_drop)),
        mass=1.0,
        origin=Origin(xyz=(0.0, lid_d * 0.5, -0.010)),
    )

    cradle = model.part("watch_cradle")
    cradle.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=satin_metal,
        name="spindle_hub",
    )
    cradle.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=graphite_trim,
        name="carrier_disk",
    )
    cradle.visual(
        Box((0.074, 0.010, 0.066)),
        origin=Origin(xyz=(0.0, 0.037, 0.006), rpy=(0.28, 0.0, 0.0)),
        material=graphite_trim,
        name="carrier_plate",
    )
    cradle.visual(
        Box((0.082, 0.050, 0.064)),
        origin=Origin(xyz=(0.0, 0.055, 0.006), rpy=(0.28, 0.0, 0.0)),
        material=charcoal_fabric,
        name="cushion_body",
    )
    cradle.visual(
        Box((0.012, 0.046, 0.060)),
        origin=Origin(xyz=(-0.035, 0.057, 0.006), rpy=(0.28, 0.0, 0.0)),
        material=warm_suede,
        name="left_bolster",
    )
    cradle.visual(
        Box((0.012, 0.046, 0.060)),
        origin=Origin(xyz=(0.035, 0.057, 0.006), rpy=(0.28, 0.0, 0.0)),
        material=warm_suede,
        name="right_bolster",
    )
    cradle.visual(
        Box((0.016, 0.020, 0.014)),
        origin=Origin(xyz=(0.034, 0.070, 0.008), rpy=(0.28, 0.0, 0.0)),
        material=satin_metal,
        name="retention_tab",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.098, 0.086, 0.080)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.046, 0.006)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=box_body,
        child=lid,
        origin=Origin(xyz=(0.0, -(outer_d * 0.5), body_h + lid_top_th * 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=box_body,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.047, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
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

    box_body = object_model.get_part("box_body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle = object_model.get_part("watch_cradle")
    cradle_spin = object_model.get_articulation("cradle_spin")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            box_body,
            axes="xy",
            min_overlap=0.140,
            name="lid covers the body footprint when closed",
        )
        ctx.expect_within(
            cradle,
            box_body,
            axes="xyz",
            margin=0.004,
            name="cradle rests within the presentation box body",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_hinge: math.radians(95.0)}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_skirt")

    ctx.check(
        "lid front edge lifts upward when opened",
        closed_front is not None
        and opened_front is not None
        and opened_front[0][2] > closed_front[0][2] + 0.080,
        details=f"closed={closed_front}, opened={opened_front}",
    )

    rest_tab = ctx.part_element_world_aabb(cradle, elem="retention_tab")
    with ctx.pose({cradle_spin: math.pi * 0.5}):
        quarter_turn_tab = ctx.part_element_world_aabb(cradle, elem="retention_tab")
        ctx.expect_within(
            cradle,
            box_body,
            axes="xyz",
            margin=0.004,
            name="cradle stays inside the box while rotating",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_tab_center = _aabb_center(rest_tab)
    quarter_turn_tab_center = _aabb_center(quarter_turn_tab)
    ctx.check(
        "cradle rotation moves the mounted watch support around the spindle",
        rest_tab_center is not None
        and quarter_turn_tab_center is not None
        and abs(quarter_turn_tab_center[0] - rest_tab_center[0]) > 0.020
        and abs(quarter_turn_tab_center[2] - rest_tab_center[2]) > 0.020,
        details=f"rest={rest_tab_center}, quarter_turn={quarter_turn_tab_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
