from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BASE_W = 0.20
BASE_D = 0.12
BASE_H = 0.03

BODY_W = 0.11
BODY_D = 0.085
BODY_H = 0.72
WALL_T = 0.007
FRONT_LIP_D = 0.006
INNER_W = BODY_W - 2.0 * WALL_T

VENT_BOTTOM = BASE_H + 0.06
VENT_TOP = BASE_H + 0.64
VENT_H = VENT_TOP - VENT_BOTTOM
BODY_TOP = BASE_H + BODY_H
BODY_UNDERSIDE_TOP = BODY_TOP - WALL_T

ROTOR_Y = 0.006
ROTOR_CENTER_Z = 0.5 * (VENT_BOTTOM + VENT_TOP)
ROTOR_RADIUS = 0.025
ROTOR_CORE_RADIUS = 0.010
ROTOR_PACK_L = 0.56
ROTOR_BLADE_L = 0.54
ROTOR_ENDCAP_T = 0.01
ROTOR_SHAFT_L = 0.01
ROTOR_SHAFT_R = 0.004

SUPPORT_D = 0.012
SUPPORT_T = 0.008

KNOB_PAD_W = 0.054
KNOB_PAD_D = 0.022
KNOB_PAD_H = 0.004
KNOB_R = 0.016
KNOB_H = 0.012
KNOB_INDICATOR_W = 0.010
KNOB_INDICATOR_D = 0.004
KNOB_INDICATOR_H = 0.002
BASE_CORNER_R = 0.018
PAD_CORNER_R = 0.006


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_box_fan", assets=ASSETS)

    shell = model.material("shell", rgba=(0.87, 0.88, 0.90, 1.0))
    trim = model.material("trim", rgba=(0.16, 0.18, 0.20, 1.0))
    base_mat = model.material("base", rgba=(0.20, 0.22, 0.24, 1.0))
    rotor_mat = model.material("rotor", rgba=(0.34, 0.36, 0.39, 1.0))
    knob_mat = model.material("knob", rgba=(0.12, 0.13, 0.14, 1.0))

    pedestal_geom = ExtrudeGeometry.centered(
        rounded_rect_profile(BASE_W * 0.92, BASE_D * 0.88, BASE_CORNER_R),
        BASE_H,
        cap=True,
        closed=True,
    )
    pedestal_geom.translate(0.0, 0.0, BASE_H / 2.0)
    pedestal_mesh = mesh_from_geometry(
        pedestal_geom,
        "assets/meshes/tower_fan_pedestal.obj",
    )

    control_pad_geom = ExtrudeGeometry.centered(
        rounded_rect_profile(KNOB_PAD_W, KNOB_PAD_D, PAD_CORNER_R),
        KNOB_PAD_H,
        cap=True,
        closed=True,
    )
    control_pad_geom.translate(
        0.0,
        BODY_D / 2.0 - KNOB_PAD_D / 2.0,
        BODY_TOP + KNOB_PAD_H / 2.0,
    )
    control_pad_mesh = mesh_from_geometry(
        control_pad_geom,
        "assets/meshes/tower_fan_control_pad.obj",
    )

    body = model.part("body")
    body.visual(
        pedestal_mesh,
        material=base_mat,
        name="pedestal",
    )
    body.visual(
        Box((BODY_W, WALL_T, BODY_H - 2.0 * WALL_T)),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 + WALL_T / 2.0, BASE_H + BODY_H / 2.0)
        ),
        material=shell,
        name="rear_panel",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H - 2.0 * WALL_T)),
        origin=Origin(
            xyz=(-BODY_W / 2.0 + WALL_T / 2.0, 0.0, BASE_H + BODY_H / 2.0)
        ),
        material=shell,
        name="left_wall",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H - 2.0 * WALL_T)),
        origin=Origin(
            xyz=(BODY_W / 2.0 - WALL_T / 2.0, 0.0, BASE_H + BODY_H / 2.0)
        ),
        material=shell,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W, BODY_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + WALL_T / 2.0)),
        material=shell,
        name="bottom_cap",
    )
    body.visual(
        Box((BODY_W, BODY_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP - WALL_T / 2.0)),
        material=shell,
        name="top_cap",
    )

    lower_panel_h = VENT_BOTTOM - (BASE_H + WALL_T)
    control_panel_h = BODY_UNDERSIDE_TOP - VENT_TOP
    front_panel_y = BODY_D / 2.0 - FRONT_LIP_D / 2.0

    body.visual(
        Box((INNER_W, FRONT_LIP_D, lower_panel_h)),
        origin=Origin(
            xyz=(0.0, front_panel_y, (BASE_H + WALL_T + VENT_BOTTOM) / 2.0)
        ),
        material=shell,
        name="front_lower_panel",
    )
    body.visual(
        Box((INNER_W, FRONT_LIP_D, control_panel_h)),
        origin=Origin(xyz=(0.0, front_panel_y, (VENT_TOP + BODY_UNDERSIDE_TOP) / 2.0)),
        material=shell,
        name="front_control_panel",
    )

    slat_count = 5
    slat_w = 0.004
    slat_d = 0.005
    slat_span = INNER_W - 0.018
    for idx in range(slat_count):
        if slat_count == 1:
            x_pos = 0.0
        else:
            x_pos = -slat_span / 2.0 + idx * (slat_span / (slat_count - 1))
        body.visual(
            Box((slat_w, slat_d, VENT_H)),
            origin=Origin(xyz=(x_pos, BODY_D / 2.0 - slat_d / 2.0 - 0.001, ROTOR_CENTER_Z)),
            material=trim,
            name=f"slat_{idx}",
        )

    body.visual(
        Box((INNER_W, SUPPORT_D, SUPPORT_T)),
        origin=Origin(xyz=(0.0, ROTOR_Y, VENT_BOTTOM - SUPPORT_T / 2.0)),
        material=trim,
        name="bottom_support",
    )
    body.visual(
        Box((INNER_W, SUPPORT_D, SUPPORT_T)),
        origin=Origin(xyz=(0.0, ROTOR_Y, VENT_TOP + SUPPORT_T / 2.0)),
        material=trim,
        name="top_support",
    )
    body.visual(
        control_pad_mesh,
        material=trim,
        name="control_pad",
    )
    body.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BODY_TOP)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=ROTOR_CORE_RADIUS, length=ROTOR_PACK_L),
        material=rotor_mat,
        name="hub_core",
    )
    rotor.visual(
        Cylinder(radius=ROTOR_RADIUS, length=ROTOR_ENDCAP_T),
        origin=Origin(xyz=(0.0, 0.0, ROTOR_PACK_L / 2.0 - ROTOR_ENDCAP_T / 2.0)),
        material=rotor_mat,
        name="top_endcap",
    )
    rotor.visual(
        Cylinder(radius=ROTOR_RADIUS, length=ROTOR_ENDCAP_T),
        origin=Origin(xyz=(0.0, 0.0, -ROTOR_PACK_L / 2.0 + ROTOR_ENDCAP_T / 2.0)),
        material=rotor_mat,
        name="bottom_endcap",
    )
    rotor.visual(
        Cylinder(radius=ROTOR_SHAFT_R, length=ROTOR_SHAFT_L),
        origin=Origin(xyz=(0.0, 0.0, ROTOR_PACK_L / 2.0 + ROTOR_SHAFT_L / 2.0)),
        material=trim,
        name="top_shaft",
    )
    rotor.visual(
        Cylinder(radius=ROTOR_SHAFT_R, length=ROTOR_SHAFT_L),
        origin=Origin(xyz=(0.0, 0.0, -ROTOR_PACK_L / 2.0 - ROTOR_SHAFT_L / 2.0)),
        material=trim,
        name="bottom_shaft",
    )

    blade_count = 7
    blade_radial_t = 0.004
    blade_tangent_d = 0.012
    blade_radius = ROTOR_RADIUS - blade_radial_t / 2.0
    for idx in range(blade_count):
        theta = pi / 2.0 + idx * (2.0 * pi / blade_count)
        rotor.visual(
            Box((blade_radial_t, blade_tangent_d, ROTOR_BLADE_L)),
            origin=Origin(
                xyz=(blade_radius * cos(theta), blade_radius * sin(theta), 0.0),
                rpy=(0.0, 0.0, theta),
            ),
            material=rotor_mat,
            name=f"blade_{idx}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=ROTOR_RADIUS, length=ROTOR_PACK_L + 2.0 * ROTOR_SHAFT_L),
        mass=0.9,
        origin=Origin(),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=KNOB_R, length=KNOB_H),
        material=knob_mat,
        name="dial",
    )
    knob.visual(
        Box((KNOB_INDICATOR_W, KNOB_INDICATOR_D, KNOB_INDICATOR_H)),
        origin=Origin(
            xyz=(
                0.0,
                KNOB_R * 0.45,
                KNOB_H / 2.0 + KNOB_INDICATOR_H / 2.0,
            )
        ),
        material=trim,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_R, length=KNOB_H),
        mass=0.05,
        origin=Origin(),
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(xyz=(0.0, ROTOR_Y, ROTOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=20.0,
        ),
    )
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(
            xyz=(
                0.0,
                BODY_D / 2.0 - KNOB_PAD_D / 2.0,
                BODY_TOP + KNOB_PAD_H + KNOB_H / 2.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=0.0,
            upper=4.71,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    body = object_model.get_part("body")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("knob")
    rotor_spin = object_model.get_articulation("rotor_spin")
    knob_turn = object_model.get_articulation("knob_turn")

    left_wall = body.get_visual("left_wall")
    right_wall = body.get_visual("right_wall")
    front_lower_panel = body.get_visual("front_lower_panel")
    front_control_panel = body.get_visual("front_control_panel")
    control_pad = body.get_visual("control_pad")
    center_slat = body.get_visual("slat_2")
    top_support = body.get_visual("top_support")
    bottom_support = body.get_visual("bottom_support")

    top_shaft = rotor.get_visual("top_shaft")
    bottom_shaft = rotor.get_visual("bottom_shaft")
    knob_indicator = knob.get_visual("indicator")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_has_world_aabb", "Body should produce measurable world bounds")
    else:
        body_size = tuple(
            body_aabb[1][axis] - body_aabb[0][axis] for axis in range(3)
        )
        ctx.check(
            "tower_body_is_tall_and_slim",
            0.72 <= body_size[2] <= 0.78
            and body_size[0] <= 0.21
            and body_size[1] <= 0.13
            and body_size[2] >= 3.5 * body_size[0]
            and body_size[2] >= 6.0 * BODY_D,
            f"body_size={body_size!r}",
        )

    rotor_aabb = ctx.part_world_aabb(rotor)
    if rotor_aabb is None:
        ctx.fail("rotor_has_world_aabb", "Rotor should produce measurable world bounds")
    else:
        rotor_height = rotor_aabb[1][2] - rotor_aabb[0][2]
        ctx.check(
            "rotor_spans_most_of_tower_vent",
            0.54 <= rotor_height <= 0.60,
            f"rotor_height={rotor_height:.4f}",
        )

    ctx.check(
        "rotor_joint_axis_is_vertical",
        tuple(rotor_spin.axis) == (0.0, 0.0, 1.0),
        f"rotor axis={rotor_spin.axis!r}",
    )
    knob_limits = knob_turn.motion_limits
    ctx.check(
        "knob_joint_axis_is_vertical",
        tuple(knob_turn.axis) == (0.0, 0.0, 1.0),
        f"knob axis={knob_turn.axis!r}",
    )
    ctx.check(
        "knob_has_realistic_dial_range",
        knob_limits is not None
        and knob_limits.lower is not None
        and knob_limits.upper is not None
        and 4.0 <= knob_limits.upper - knob_limits.lower <= 5.5,
        (
            "knob limits="
            f"{None if knob_limits is None else (knob_limits.lower, knob_limits.upper)}"
        ),
    )

    ctx.expect_gap(
        body,
        body,
        axis="x",
        positive_elem=right_wall,
        negative_elem=left_wall,
        min_gap=0.09,
        name="tower_housing_keeps_a_slim_wide_front_channel",
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem=front_control_panel,
        negative_elem=front_lower_panel,
        min_gap=0.55,
        name="front_vent_runs_through_most_of_the_tower_height",
    )
    ctx.expect_gap(
        rotor,
        body,
        axis="x",
        negative_elem=left_wall,
        min_gap=0.015,
        name="rotor_clears_left_side_of_housing",
    )
    ctx.expect_gap(
        body,
        rotor,
        axis="x",
        positive_elem=right_wall,
        min_gap=0.015,
        name="rotor_clears_right_side_of_housing",
    )
    ctx.expect_gap(
        body,
        rotor,
        axis="y",
        positive_elem=center_slat,
        min_gap=0.003,
        name="rotor_sits_just_behind_the_front_grille",
    )
    ctx.expect_gap(
        body,
        rotor,
        axis="z",
        positive_elem=top_support,
        negative_elem=top_shaft,
        max_gap=0.001,
        max_penetration=0.0,
        name="top_axle_is_seated_against_upper_support",
    )
    ctx.expect_gap(
        rotor,
        body,
        axis="z",
        positive_elem=bottom_shaft,
        negative_elem=bottom_support,
        max_gap=0.001,
        max_penetration=1e-6,
        name="bottom_axle_is_seated_against_lower_support",
    )
    ctx.expect_contact(rotor, body, elem_a=top_shaft, elem_b=top_support)
    ctx.expect_contact(rotor, body, elem_a=bottom_shaft, elem_b=bottom_support)
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        negative_elem=control_pad,
        max_gap=0.001,
        max_penetration=1e-6,
        name="dial_knob_is_seated_on_the_top_front_pad",
    )
    ctx.expect_contact(knob, body, elem_b=control_pad)
    ctx.expect_origin_distance(knob, body, axes="x", max_dist=0.01)
    ctx.expect_origin_gap(
        knob,
        body,
        axis="y",
        min_gap=0.025,
        max_gap=0.035,
        name="dial_knob_sits_at_the_front_edge",
    )
    ctx.expect_origin_gap(
        knob,
        body,
        axis="z",
        min_gap=0.74,
        max_gap=0.78,
        name="dial_knob_sits_on_top_of_the_tower",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="x",
        elem_a=knob_indicator,
        elem_b=control_pad,
        min_overlap=0.008,
        name="dial_indicator_stays_over_the_control_pad",
    )

    with ctx.pose({rotor_spin: 1.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotor_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="rotor_quarter_turn_no_floating")
        ctx.expect_gap(
            body,
            rotor,
            axis="y",
            positive_elem=center_slat,
            min_gap=0.003,
            name="rotor_keeps_grille_clearance_in_a_spun_pose",
        )
        ctx.expect_gap(
            body,
            rotor,
            axis="z",
            positive_elem=top_support,
            negative_elem=top_shaft,
            max_gap=0.001,
            max_penetration=0.0,
            name="top_axle_stays_seated_when_rotated",
        )
        ctx.expect_gap(
            rotor,
            body,
            axis="z",
            positive_elem=bottom_shaft,
            negative_elem=bottom_support,
            max_gap=0.001,
            max_penetration=1e-6,
            name="bottom_axle_stays_seated_when_rotated",
        )
    if knob_limits is not None and knob_limits.upper is not None:
        with ctx.pose({knob_turn: knob_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="knob_at_max_turn_no_overlap")
            ctx.fail_if_isolated_parts(name="knob_at_max_turn_no_floating")
            ctx.expect_gap(
                knob,
                body,
                axis="z",
                negative_elem=control_pad,
                max_gap=0.001,
                max_penetration=1e-6,
                name="dial_knob_stays_seated_at_max_turn",
            )
            ctx.expect_contact(
                knob,
                body,
                elem_b=control_pad,
                name="dial_knob_keeps_contact_at_max_turn",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
