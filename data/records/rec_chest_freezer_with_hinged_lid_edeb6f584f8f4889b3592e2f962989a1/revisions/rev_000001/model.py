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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="patio_chest_cooler")

    # Overall proportions: a wide patio cooler with short legs and a thick lid.
    outer_w = 1.18
    outer_d = 0.58
    leg_h = 0.10
    body_h = 0.50
    body_top_z = leg_h + body_h

    base_t = 0.035
    outer_wall_t = 0.025
    liner_wall_t = 0.015
    insulation_gap = 0.03
    inner_floor_t = 0.02

    liner_outer_w = outer_w - 2.0 * (outer_wall_t + insulation_gap)
    liner_outer_d = outer_d - 2.0 * (outer_wall_t + insulation_gap)
    liner_inner_w = liner_outer_w - 2.0 * liner_wall_t
    liner_inner_d = liner_outer_d - 2.0 * liner_wall_t

    inner_floor_cz = 0.155
    inner_floor_top_z = inner_floor_cz + inner_floor_t / 2.0
    liner_top_z = 0.555
    liner_wall_h = liner_top_z - inner_floor_top_z
    liner_wall_cz = inner_floor_top_z + liner_wall_h / 2.0
    rim_h = body_top_z - liner_top_z
    rim_cz = liner_top_z + rim_h / 2.0

    drain_center_z = 0.197
    drain_open = 0.05
    drain_tunnel_len = insulation_gap
    drain_tunnel_y = outer_d / 2.0 - outer_wall_t - drain_tunnel_len / 2.0
    drain_tunnel_outer = 0.05
    drain_tunnel_wall = 0.009
    drain_tunnel_inner = drain_tunnel_outer - 2.0 * drain_tunnel_wall

    lid_w = 1.20
    lid_d = 0.60
    lid_top_t = 0.034
    lid_skirt_h = 0.055
    lid_front_skirt_d = 0.03
    lid_side_skirt_t = 0.03
    lid_underside_t = 0.015
    lid_rear_rail_d = 0.02
    hinge_axis_y = -(outer_d / 2.0 + 0.01)

    body_color = model.material("body_color", rgba=(0.77, 0.75, 0.70, 1.0))
    liner_color = model.material("liner_color", rgba=(0.95, 0.95, 0.93, 1.0))
    leg_color = model.material("leg_color", rgba=(0.18, 0.18, 0.18, 1.0))
    hardware_color = model.material("hardware_color", rgba=(0.30, 0.30, 0.32, 1.0))
    plug_color = model.material("plug_color", rgba=(0.12, 0.12, 0.12, 1.0))

    body = model.part("body")

    body.visual(
        Box((outer_w, outer_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, leg_h + base_t / 2.0)),
        material=body_color,
        name="base_pan",
    )

    body.visual(
        Box((outer_wall_t, outer_d, body_h)),
        origin=Origin(xyz=(outer_w / 2.0 - outer_wall_t / 2.0, 0.0, leg_h + body_h / 2.0)),
        material=body_color,
        name="outer_right_wall",
    )
    body.visual(
        Box((outer_wall_t, outer_d, body_h)),
        origin=Origin(xyz=(-(outer_w / 2.0 - outer_wall_t / 2.0), 0.0, leg_h + body_h / 2.0)),
        material=body_color,
        name="outer_left_wall",
    )

    outer_front_span = outer_w - 2.0 * outer_wall_t
    outer_front_side_w = (outer_front_span - drain_open) / 2.0
    outer_front_lower_h = drain_center_z - drain_open / 2.0 - leg_h
    outer_front_upper_h = body_top_z - (drain_center_z + drain_open / 2.0)

    body.visual(
        Box((outer_front_span, outer_wall_t, outer_front_lower_h)),
        origin=Origin(
            xyz=(0.0, outer_d / 2.0 - outer_wall_t / 2.0, leg_h + outer_front_lower_h / 2.0)
        ),
        material=body_color,
        name="outer_front_lower",
    )
    body.visual(
        Box((outer_front_side_w, outer_wall_t, drain_open)),
        origin=Origin(
            xyz=(
                -(drain_open / 2.0 + outer_front_side_w / 2.0),
                outer_d / 2.0 - outer_wall_t / 2.0,
                drain_center_z,
            )
        ),
        material=body_color,
        name="outer_front_left_mid",
    )
    body.visual(
        Box((outer_front_side_w, outer_wall_t, drain_open)),
        origin=Origin(
            xyz=(
                drain_open / 2.0 + outer_front_side_w / 2.0,
                outer_d / 2.0 - outer_wall_t / 2.0,
                drain_center_z,
            )
        ),
        material=body_color,
        name="outer_front_right_mid",
    )
    body.visual(
        Box((outer_front_span, outer_wall_t, outer_front_upper_h)),
        origin=Origin(
            xyz=(
                0.0,
                outer_d / 2.0 - outer_wall_t / 2.0,
                drain_center_z + drain_open / 2.0 + outer_front_upper_h / 2.0,
            )
        ),
        material=body_color,
        name="outer_front_upper",
    )

    body.visual(
        Box((outer_front_span, outer_wall_t, body_h)),
        origin=Origin(xyz=(0.0, -(outer_d / 2.0 - outer_wall_t / 2.0), leg_h + body_h / 2.0)),
        material=body_color,
        name="outer_rear_wall",
    )

    leg_size = (0.075, 0.075, leg_h)
    leg_x = outer_w / 2.0 - 0.10
    leg_y = outer_d / 2.0 - 0.11
    for name, x_sign, y_sign in (
        ("front_left_leg", -1.0, 1.0),
        ("front_right_leg", 1.0, 1.0),
        ("rear_left_leg", -1.0, -1.0),
        ("rear_right_leg", 1.0, -1.0),
    ):
        body.visual(
            Box(leg_size),
            origin=Origin(xyz=(x_sign * leg_x, y_sign * leg_y, leg_h / 2.0)),
            material=leg_color,
            name=name,
        )

    body.visual(
        Box((liner_inner_w, liner_inner_d, inner_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, inner_floor_cz)),
        material=liner_color,
        name="inner_floor",
    )
    body.visual(
        Box((liner_wall_t, liner_outer_d, liner_wall_h)),
        origin=Origin(
            xyz=(liner_outer_w / 2.0 - liner_wall_t / 2.0, 0.0, liner_wall_cz)
        ),
        material=liner_color,
        name="liner_right_wall",
    )
    body.visual(
        Box((liner_wall_t, liner_outer_d, liner_wall_h)),
        origin=Origin(
            xyz=(-(liner_outer_w / 2.0 - liner_wall_t / 2.0), 0.0, liner_wall_cz)
        ),
        material=liner_color,
        name="liner_left_wall",
    )
    body.visual(
        Box((liner_inner_w, liner_wall_t, liner_wall_h)),
        origin=Origin(
            xyz=(0.0, -(liner_outer_d / 2.0 - liner_wall_t / 2.0), liner_wall_cz)
        ),
        material=liner_color,
        name="liner_rear_wall",
    )

    liner_front_side_w = (liner_inner_w - drain_open) / 2.0
    liner_front_lower_h = drain_center_z - drain_open / 2.0 - inner_floor_top_z
    liner_front_upper_h = liner_top_z - (drain_center_z + drain_open / 2.0)

    body.visual(
        Box((liner_inner_w, liner_wall_t, liner_front_lower_h)),
        origin=Origin(
            xyz=(
                0.0,
                liner_outer_d / 2.0 - liner_wall_t / 2.0,
                inner_floor_top_z + liner_front_lower_h / 2.0,
            )
        ),
        material=liner_color,
        name="liner_front_lower",
    )
    body.visual(
        Box((liner_front_side_w, liner_wall_t, drain_open)),
        origin=Origin(
            xyz=(
                -(drain_open / 2.0 + liner_front_side_w / 2.0),
                liner_outer_d / 2.0 - liner_wall_t / 2.0,
                drain_center_z,
            )
        ),
        material=liner_color,
        name="liner_front_left_mid",
    )
    body.visual(
        Box((liner_front_side_w, liner_wall_t, drain_open)),
        origin=Origin(
            xyz=(
                drain_open / 2.0 + liner_front_side_w / 2.0,
                liner_outer_d / 2.0 - liner_wall_t / 2.0,
                drain_center_z,
            )
        ),
        material=liner_color,
        name="liner_front_right_mid",
    )
    body.visual(
        Box((liner_inner_w, liner_wall_t, liner_front_upper_h)),
        origin=Origin(
            xyz=(
                0.0,
                liner_outer_d / 2.0 - liner_wall_t / 2.0,
                drain_center_z + drain_open / 2.0 + liner_front_upper_h / 2.0,
            )
        ),
        material=liner_color,
        name="liner_front_upper",
    )

    body.visual(
        Box((insulation_gap, outer_d - 2.0 * outer_wall_t, rim_h)),
        origin=Origin(xyz=(outer_w / 2.0 - outer_wall_t - insulation_gap / 2.0, 0.0, rim_cz)),
        material=body_color,
        name="right_top_rim",
    )
    body.visual(
        Box((insulation_gap, outer_d - 2.0 * outer_wall_t, rim_h)),
        origin=Origin(xyz=(-(outer_w / 2.0 - outer_wall_t - insulation_gap / 2.0), 0.0, rim_cz)),
        material=body_color,
        name="left_top_rim",
    )
    body.visual(
        Box((liner_outer_w, insulation_gap, rim_h)),
        origin=Origin(xyz=(0.0, outer_d / 2.0 - outer_wall_t - insulation_gap / 2.0, rim_cz)),
        material=body_color,
        name="front_top_rim",
    )
    body.visual(
        Box((liner_outer_w, insulation_gap, rim_h)),
        origin=Origin(xyz=(0.0, -(outer_d / 2.0 - outer_wall_t - insulation_gap / 2.0), rim_cz)),
        material=body_color,
        name="rear_top_rim",
    )

    support_size = (0.06, 0.06, 0.02)
    for name, x_sign, y_sign in (
        ("inner_support_fl", -1.0, 1.0),
        ("inner_support_fr", 1.0, 1.0),
        ("inner_support_rl", -1.0, -1.0),
        ("inner_support_rr", 1.0, -1.0),
    ):
        body.visual(
            Box(support_size),
            origin=Origin(
                xyz=(
                    x_sign * (liner_outer_w / 2.0 - 0.045),
                    y_sign * (liner_outer_d / 2.0 - 0.045),
                    inner_floor_cz - 0.01,
                )
            ),
            material=body_color,
            name=name,
        )

    body.visual(
        Box((drain_tunnel_outer, drain_tunnel_len, drain_tunnel_wall)),
        origin=Origin(xyz=(0.0, drain_tunnel_y, drain_center_z + drain_tunnel_outer / 2.0 - drain_tunnel_wall / 2.0)),
        material=body_color,
        name="drain_tunnel_top",
    )
    body.visual(
        Box((drain_tunnel_outer, drain_tunnel_len, drain_tunnel_wall)),
        origin=Origin(xyz=(0.0, drain_tunnel_y, drain_center_z - drain_tunnel_outer / 2.0 + drain_tunnel_wall / 2.0)),
        material=body_color,
        name="drain_tunnel_bottom",
    )
    body.visual(
        Box((drain_tunnel_wall, drain_tunnel_len, drain_tunnel_inner)),
        origin=Origin(
            xyz=(
                -(drain_tunnel_outer / 2.0 - drain_tunnel_wall / 2.0),
                drain_tunnel_y,
                drain_center_z,
            )
        ),
        material=body_color,
        name="drain_tunnel_left",
    )
    body.visual(
        Box((drain_tunnel_wall, drain_tunnel_len, drain_tunnel_inner)),
        origin=Origin(
            xyz=(
                drain_tunnel_outer / 2.0 - drain_tunnel_wall / 2.0,
                drain_tunnel_y,
                drain_center_z,
            )
        ),
        material=body_color,
        name="drain_tunnel_right",
    )

    body.visual(
        Box((0.095, 0.048, 0.028)),
        origin=Origin(xyz=(-0.34, hinge_axis_y - 0.004, body_top_z - 0.061)),
        material=hardware_color,
        name="left_hinge_pad",
    )
    body.visual(
        Box((0.095, 0.048, 0.028)),
        origin=Origin(xyz=(0.34, hinge_axis_y - 0.004, body_top_z - 0.061)),
        material=hardware_color,
        name="right_hinge_pad",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_top_t)),
        origin=Origin(xyz=(0.0, lid_d / 2.0 - 0.01, lid_top_t / 2.0)),
        material=body_color,
        name="lid_top",
    )
    lid.visual(
        Box((lid_w - 0.08, lid_front_skirt_d, lid_skirt_h)),
        origin=Origin(
            xyz=(
                0.0,
                lid_d + lid_front_skirt_d / 2.0 - 0.01,
                -lid_skirt_h / 2.0,
            )
        ),
        material=body_color,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((lid_side_skirt_t, lid_d - 0.01, lid_skirt_h)),
        origin=Origin(
            xyz=(
                lid_w / 2.0 + lid_side_skirt_t / 2.0,
                (lid_d - 0.01) / 2.0,
                -lid_skirt_h / 2.0,
            )
        ),
        material=body_color,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((lid_side_skirt_t, lid_d - 0.01, lid_skirt_h)),
        origin=Origin(
            xyz=(
                -(lid_w / 2.0 + lid_side_skirt_t / 2.0),
                (lid_d - 0.01) / 2.0,
                -lid_skirt_h / 2.0,
            )
        ),
        material=body_color,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((lid_w - 0.08, lid_rear_rail_d, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=body_color,
        name="lid_rear_rail",
    )
    lid.visual(
        Box((0.98, 0.40, lid_underside_t)),
        origin=Origin(xyz=(0.0, 0.30, -0.0225)),
        material=liner_color,
        name="lid_underside",
    )
    lid.visual(
        Box((1.00, 0.012, 0.03)),
        origin=Origin(xyz=(0.0, 0.096, -0.015)),
        material=liner_color,
        name="lid_inner_rear_wall",
    )
    lid.visual(
        Box((1.00, 0.012, 0.03)),
        origin=Origin(xyz=(0.0, 0.504, -0.015)),
        material=liner_color,
        name="lid_inner_front_wall",
    )
    lid.visual(
        Box((0.012, 0.42, 0.03)),
        origin=Origin(xyz=(0.494, 0.30, -0.015)),
        material=liner_color,
        name="lid_inner_right_wall",
    )
    lid.visual(
        Box((0.012, 0.42, 0.03)),
        origin=Origin(xyz=(-0.494, 0.30, -0.015)),
        material=liner_color,
        name="lid_inner_left_wall",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.34, -0.015, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_color,
        name="left_hinge_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.34, -0.015, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_color,
        name="right_hinge_knuckle",
    )

    drain_plug = model.part("drain_plug")
    drain_plug.visual(
        Cylinder(radius=0.014, length=0.08),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plug_color,
        name="plug_stem",
    )
    drain_plug.visual(
        Cylinder(radius=0.028, length=0.01),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plug_color,
        name="plug_cap",
    )
    drain_plug.visual(
        Box((0.012, 0.01, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, 0.018)),
        material=plug_color,
        name="plug_tab",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, body_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.4,
        ),
    )
    model.articulation(
        "body_to_drain_plug",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain_plug,
        origin=Origin(xyz=(0.0, outer_d / 2.0, drain_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
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
    lid = object_model.get_part("lid")
    drain_plug = object_model.get_part("drain_plug")
    lid_hinge = object_model.get_articulation("body_to_lid")
    plug_slide = object_model.get_articulation("body_to_drain_plug")

    _ = lid.get_visual("left_hinge_knuckle")
    _ = lid.get_visual("right_hinge_knuckle")
    _ = drain_plug.get_visual("plug_stem")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.50,
            name="closed lid covers the cooler opening",
        )
        ctx.expect_contact(
            drain_plug,
            body,
            contact_tol=0.0015,
            name="drain plug seats against the front drain face",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    with ctx.pose({lid_hinge: 1.2}):
        open_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    ctx.check(
        "lid front edge rises when opened",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.22,
        details=f"closed={closed_front}, open={open_front}",
    )

    rest_plug = ctx.part_world_position(drain_plug)
    with ctx.pose({plug_slide: 0.018}):
        extended_plug = ctx.part_world_position(drain_plug)
    ctx.check(
        "drain plug slides outward from the body",
        rest_plug is not None
        and extended_plug is not None
        and extended_plug[1] > rest_plug[1] + 0.012,
        details=f"rest={rest_plug}, extended={extended_plug}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
