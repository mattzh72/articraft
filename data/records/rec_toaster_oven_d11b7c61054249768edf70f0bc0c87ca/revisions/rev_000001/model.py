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
    model = ArticulatedObject(name="countertop_toaster_oven")

    stainless = model.material("stainless", rgba=(0.72, 0.73, 0.75, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.18, 0.23, 0.27, 0.45))
    handle_metal = model.material("handle_metal", rgba=(0.82, 0.83, 0.84, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.72, 0.74, 0.77, 1.0))

    outer_w = 0.52
    outer_d = 0.40
    outer_h = 0.32
    wall_t = 0.012
    top_t = 0.012
    bottom_t = 0.020

    panel_w = 0.104
    panel_center_x = outer_w * 0.5 - panel_w * 0.5 - 0.006
    opening_w = 0.356
    opening_h = 0.195
    opening_center_x = -0.046
    opening_bottom_z = 0.058
    opening_center_z = opening_bottom_z + opening_h * 0.5
    lip_t = 0.008
    lip_d = 0.008
    lip_center_y = -outer_d * 0.5 + wall_t + lip_d * 0.5
    opening_left_x = opening_center_x - opening_w * 0.5
    opening_right_x = opening_center_x + opening_w * 0.5
    lip_left_outer_x = -outer_w * 0.5 + wall_t
    lip_right_outer_x = panel_center_x - panel_w * 0.5
    lip_frame_w = lip_right_outer_x - lip_left_outer_x
    lip_frame_center_x = (lip_left_outer_x + lip_right_outer_x) * 0.5

    body = model.part("oven_body")
    body.visual(
        Box((wall_t, outer_d, outer_h)),
        origin=Origin(xyz=(-outer_w * 0.5 + wall_t * 0.5, 0.0, outer_h * 0.5)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, outer_h)),
        origin=Origin(xyz=(outer_w * 0.5 - wall_t * 0.5, 0.0, outer_h * 0.5)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, outer_d, top_t)),
        origin=Origin(xyz=(0.0, 0.0, outer_h - top_t * 0.5)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t * 0.5)),
        material=trim_dark,
        name="bottom_shell",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, outer_h - top_t - bottom_t)),
        origin=Origin(
            xyz=(
                0.0,
                outer_d * 0.5 - wall_t * 0.5,
                bottom_t + (outer_h - top_t - bottom_t) * 0.5,
            )
        ),
        material=stainless,
        name="rear_shell",
    )
    body.visual(
        Box((wall_t, outer_d - 2.0 * wall_t, outer_h - 0.042)),
        origin=Origin(
            xyz=(
                panel_center_x - panel_w * 0.5 + wall_t * 0.5,
                0.0,
                bottom_t + (outer_h - 0.042) * 0.5,
            )
        ),
        material=trim_dark,
        name="control_partition",
    )

    panel_front_y = -outer_d * 0.5 + wall_t * 0.5
    panel_height = 0.268
    panel_center_z = 0.172
    panel_opening_h = 0.048
    panel_opening_gap = 0.022
    panel_top_z = panel_center_z + panel_height * 0.5
    panel_bottom_z = panel_center_z - panel_height * 0.5
    knob_zs = (0.244, 0.174, 0.104)
    rail_w = 0.018
    fascia_inner_w = panel_w - 2.0 * rail_w

    body.visual(
        Box((rail_w, wall_t, panel_height)),
        origin=Origin(
            xyz=(panel_center_x - panel_w * 0.5 + rail_w * 0.5, panel_front_y, panel_center_z)
        ),
        material=trim_dark,
        name="panel_left_rail",
    )
    body.visual(
        Box((rail_w, wall_t, panel_height)),
        origin=Origin(
            xyz=(panel_center_x + panel_w * 0.5 - rail_w * 0.5, panel_front_y, panel_center_z)
        ),
        material=trim_dark,
        name="panel_right_rail",
    )
    body.visual(
        Box((fascia_inner_w, wall_t, panel_top_z - (knob_zs[0] + panel_opening_h * 0.5))),
        origin=Origin(
            xyz=(
                panel_center_x,
                panel_front_y,
                (panel_top_z + knob_zs[0] + panel_opening_h * 0.5) * 0.5,
            )
        ),
        material=trim_dark,
        name="panel_top_cap",
    )
    body.visual(
        Box((fascia_inner_w, wall_t, (knob_zs[2] - panel_opening_h * 0.5) - panel_bottom_z)),
        origin=Origin(
            xyz=(
                panel_center_x,
                panel_front_y,
                ((knob_zs[2] - panel_opening_h * 0.5) + panel_bottom_z) * 0.5,
            )
        ),
        material=trim_dark,
        name="panel_bottom_cap",
    )
    body.visual(
        Box(
            (
                fascia_inner_w,
                wall_t,
                (knob_zs[0] - panel_opening_h * 0.5)
                - (knob_zs[1] + panel_opening_h * 0.5),
            )
        ),
        origin=Origin(
            xyz=(
                panel_center_x,
                panel_front_y,
                (
                    (knob_zs[0] - panel_opening_h * 0.5)
                    + (knob_zs[1] + panel_opening_h * 0.5)
                )
                * 0.5,
            )
        ),
        material=trim_dark,
        name="panel_mid_bridge_upper",
    )
    body.visual(
        Box(
            (
                fascia_inner_w,
                wall_t,
                (knob_zs[1] - panel_opening_h * 0.5)
                - (knob_zs[2] + panel_opening_h * 0.5),
            )
        ),
        origin=Origin(
            xyz=(
                panel_center_x,
                panel_front_y,
                (
                    (knob_zs[1] - panel_opening_h * 0.5)
                    + (knob_zs[2] + panel_opening_h * 0.5)
                )
                * 0.5,
            )
        ),
        material=trim_dark,
        name="panel_mid_bridge_lower",
    )
    body.visual(
        Box((panel_w - 2.0 * wall_t, 0.072, wall_t)),
        origin=Origin(xyz=(panel_center_x, -0.158, panel_top_z - wall_t * 0.5)),
        material=trim_dark,
        name="control_bay_top",
    )
    body.visual(
        Box((panel_w - 2.0 * wall_t, 0.072, wall_t)),
        origin=Origin(xyz=(panel_center_x, -0.158, panel_bottom_z + wall_t * 0.5)),
        material=trim_dark,
        name="control_bay_bottom",
    )
    body.visual(
        Box((panel_w - 2.0 * wall_t, wall_t, panel_height - 2.0 * wall_t)),
        origin=Origin(xyz=(panel_center_x, -0.122, panel_center_z,)),
        material=trim_dark,
        name="control_bay_back",
    )
    knob_hole_w = 0.024
    knob_hole_h = 0.024
    panel_inner_left_x = panel_center_x - fascia_inner_w * 0.5
    panel_inner_right_x = panel_center_x + fascia_inner_w * 0.5
    side_fill_w = (fascia_inner_w - knob_hole_w) * 0.5
    top_fill_h = (panel_opening_h - knob_hole_h) * 0.5
    for index, knob_z in enumerate(knob_zs):
        body.visual(
            Box((side_fill_w, wall_t, knob_hole_h)),
            origin=Origin(
                xyz=(
                    (panel_inner_left_x + (panel_center_x - knob_hole_w * 0.5)) * 0.5,
                    panel_front_y,
                    knob_z,
                )
            ),
            material=trim_dark,
            name=f"knob_opening_left_fill_{index}",
        )
        body.visual(
            Box((side_fill_w, wall_t, knob_hole_h)),
            origin=Origin(
                xyz=(
                    ((panel_center_x + knob_hole_w * 0.5) + panel_inner_right_x) * 0.5,
                    panel_front_y,
                    knob_z,
                )
            ),
            material=trim_dark,
            name=f"knob_opening_right_fill_{index}",
        )
        body.visual(
            Box((knob_hole_w, wall_t, top_fill_h)),
            origin=Origin(
                xyz=(
                    panel_center_x,
                    panel_front_y,
                    knob_z + knob_hole_h * 0.5 + top_fill_h * 0.5,
                )
            ),
            material=trim_dark,
            name=f"knob_opening_top_fill_{index}",
        )
        body.visual(
            Box((knob_hole_w, wall_t, top_fill_h)),
            origin=Origin(
                xyz=(
                    panel_center_x,
                    panel_front_y,
                    knob_z - knob_hole_h * 0.5 - top_fill_h * 0.5,
                )
            ),
            material=trim_dark,
            name=f"knob_opening_bottom_fill_{index}",
        )

    body.visual(
        Box((opening_left_x - lip_left_outer_x, lip_d, opening_h + 2.0 * lip_t)),
        origin=Origin(
            xyz=(
                (lip_left_outer_x + opening_left_x) * 0.5,
                lip_center_y,
                opening_center_z,
            )
        ),
        material=trim_dark,
        name="stop_lip_left",
    )
    body.visual(
        Box((lip_right_outer_x - opening_right_x, lip_d, opening_h + 2.0 * lip_t)),
        origin=Origin(
            xyz=(
                (opening_right_x + lip_right_outer_x) * 0.5,
                lip_center_y,
                opening_center_z,
            )
        ),
        material=trim_dark,
        name="stop_lip_right",
    )
    body.visual(
        Box((lip_frame_w, lip_d, lip_t)),
        origin=Origin(
            xyz=(
                lip_frame_center_x,
                lip_center_y,
                opening_bottom_z + opening_h + lip_t * 0.5,
            )
        ),
        material=trim_dark,
        name="stop_lip_top",
    )
    body.visual(
        Box((lip_frame_w, lip_d, lip_t)),
        origin=Origin(xyz=(lip_frame_center_x, lip_center_y, opening_bottom_z - lip_t * 0.5)),
        material=trim_dark,
        name="stop_lip_bottom",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, outer_h * 0.5)),
    )

    door = model.part("door")
    door_w = opening_w + 0.018
    door_h = opening_h + 0.014
    door_t = 0.018
    side_frame_w = 0.024
    top_frame_h = 0.026
    bottom_frame_h = 0.038
    glass_w = door_w - 2.0 * side_frame_w + 0.004
    glass_h = door_h - top_frame_h - bottom_frame_h + 0.004

    door.visual(
        Box((side_frame_w, door_t, door_h)),
        origin=Origin(xyz=(-door_w * 0.5 + side_frame_w * 0.5, door_t * 0.5, door_h * 0.5)),
        material=trim_dark,
        name="door_frame_left",
    )
    door.visual(
        Box((side_frame_w, door_t, door_h)),
        origin=Origin(xyz=(door_w * 0.5 - side_frame_w * 0.5, door_t * 0.5, door_h * 0.5)),
        material=trim_dark,
        name="door_frame_right",
    )
    door.visual(
        Box((door_w, door_t, top_frame_h)),
        origin=Origin(xyz=(0.0, door_t * 0.5, door_h - top_frame_h * 0.5)),
        material=trim_dark,
        name="door_frame_top",
    )
    door.visual(
        Box((door_w, door_t, bottom_frame_h)),
        origin=Origin(xyz=(0.0, door_t * 0.5, bottom_frame_h * 0.5)),
        material=trim_dark,
        name="door_frame_bottom",
    )
    door.visual(
        Box((glass_w, 0.004, glass_h)),
        origin=Origin(
            xyz=(
                0.0,
                door_t - 0.002,
                bottom_frame_h + glass_h * 0.5 + 0.004,
            )
        ),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.010, length=door_w * 0.68),
        origin=Origin(
            xyz=(0.0, -0.030, door_h - 0.020),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=handle_metal,
        name="door_handle",
    )
    for index, offset_x in enumerate((-door_w * 0.25, door_w * 0.25)):
        door.visual(
            Cylinder(radius=0.005, length=0.034),
            origin=Origin(
                xyz=(offset_x, -0.014, door_h - 0.020),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=handle_metal,
            name=f"handle_post_{index}",
        )
    for index, offset_x in enumerate((-door_w * 0.32, door_w * 0.32)):
        door.visual(
            Cylinder(radius=0.007, length=0.040),
            origin=Origin(
                xyz=(offset_x, 0.010, 0.006),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=trim_dark,
            name=f"hinge_barrel_{index}",
        )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t + 0.050, door_h)),
        mass=1.5,
        origin=Origin(xyz=(0.0, -0.008, door_h * 0.5)),
    )

    door_hinge_y = -outer_d * 0.5 - 0.006
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(opening_center_x, door_hinge_y, opening_bottom_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    knob_names = ("upper", "middle", "lower")
    for knob_name, knob_z in zip(knob_names, knob_zs):
        knob = model.part(f"knob_{knob_name}")
        knob.visual(
            Cylinder(radius=0.0045, length=0.020),
            origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=handle_metal,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_cap,
            name="shaft_collar",
        )
        knob.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_black,
            name="knob_base",
        )
        knob.visual(
            Cylinder(radius=0.021, length=0.022),
            origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_black,
            name="knob_grip",
        )
        knob.visual(
            Cylinder(radius=0.015, length=0.004),
            origin=Origin(xyz=(0.0, -0.038, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_cap,
            name="front_cap",
        )
        knob.visual(
            Box((0.006, 0.003, 0.014)),
            origin=Origin(xyz=(0.013, -0.039, 0.0)),
            material=knob_cap,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.050, 0.042, 0.050)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.014, 0.0)),
        )
        model.articulation(
            f"body_to_knob_{knob_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(panel_center_x, -outer_d * 0.5, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=4.0,
                lower=-2.5,
                upper=2.5,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="stop_lip_bottom",
            max_gap=0.002,
            max_penetration=0.0,
            name="door seats against opening stop lip",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            elem_b="stop_lip_top",
            min_overlap=0.30,
            name="door spans the oven opening width",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="z",
            elem_b="stop_lip_left",
            min_overlap=0.18,
            name="door spans the oven opening height",
        )

    closed_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    with ctx.pose({hinge: 1.15}):
        open_handle = ctx.part_element_world_aabb(door, elem="door_handle")

    ctx.check(
        "door opens downward and outward",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][1] < closed_handle[0][1] - 0.05
        and open_handle[1][2] < closed_handle[1][2] - 0.10,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    for knob_name in ("upper", "middle", "lower"):
        knob = object_model.get_part(f"knob_{knob_name}")
        joint = object_model.get_articulation(f"body_to_knob_{knob_name}")
        rest_indicator = ctx.part_element_world_aabb(knob, elem="indicator")
        with ctx.pose({joint: 1.2}):
            turned_indicator = ctx.part_element_world_aabb(knob, elem="indicator")

        rest_center = _aabb_center(rest_indicator)
        turned_center = _aabb_center(turned_indicator)
        radial_motion = None
        if rest_center is not None and turned_center is not None:
            radial_motion = math.hypot(
                turned_center[0] - rest_center[0],
                turned_center[2] - rest_center[2],
            )

        ctx.check(
            f"{knob_name} knob rotates around its shaft",
            radial_motion is not None and radial_motion > 0.010,
            details=(
                f"rest_indicator={rest_indicator}, "
                f"turned_indicator={turned_indicator}, radial_motion={radial_motion}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
