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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_mesh_router")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.15, 0.16, 1.0))
    soft_blue = model.material("soft_blue", rgba=(0.45, 0.66, 0.85, 1.0))

    body_width = 0.118
    body_depth = 0.118
    body_height = 0.197
    base_height = 0.008
    wall_height = 0.174
    wall_top_z = base_height + wall_height
    top_frame_height = body_height - wall_top_z
    corner_radius = 0.012
    wall_thickness = 0.010

    front_flap_width = 0.058
    front_flap_height = 0.040
    front_flap_top_z = 0.170
    rear_flap_width = 0.050
    rear_flap_height = 0.036
    rear_flap_top_z = 0.056

    top_frame_outer = rounded_rect_profile(body_width, body_depth, radius=0.016, corner_segments=8)
    top_frame_inner = rounded_rect_profile(0.082, 0.082, radius=0.008, corner_segments=8)
    top_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            top_frame_outer,
            [top_frame_inner],
            height=top_frame_height,
            center=True,
        ),
        "router_top_frame",
    )

    vent_slot = rounded_rect_profile(0.010, 0.030, radius=0.004, corner_segments=6)
    vent_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.082, 0.082, radius=0.008, corner_segments=8),
            [
                _shift_profile(vent_slot, dx=-0.022, dy=-0.018),
                _shift_profile(vent_slot, dx=-0.022, dy=0.018),
                _shift_profile(vent_slot, dx=0.022, dy=-0.018),
                _shift_profile(vent_slot, dx=0.022, dy=0.018),
            ],
            height=0.003,
            center=True,
        ),
        "router_vent_panel",
    )

    router_body = model.part("router_body")
    router_body.visual(
        Box((0.110, 0.110, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=shell_white,
        name="base_plinth",
    )
    router_body.visual(
        Box((wall_thickness, 0.094, wall_height)),
        origin=Origin(
            xyz=(
                -(body_width / 2.0 - wall_thickness / 2.0),
                0.0,
                base_height + wall_height / 2.0,
            )
        ),
        material=shell_white,
        name="left_shell_wall",
    )
    router_body.visual(
        Box((wall_thickness, 0.094, wall_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - wall_thickness / 2.0,
                0.0,
                base_height + wall_height / 2.0,
            )
        ),
        material=shell_white,
        name="right_shell_wall",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            router_body.visual(
                Cylinder(radius=corner_radius, length=wall_height),
                origin=Origin(
                    xyz=(
                        x_sign * (body_width / 2.0 - corner_radius),
                        y_sign * (body_depth / 2.0 - corner_radius),
                        base_height + wall_height / 2.0,
                    )
                ),
                material=shell_white,
                name=f"corner_post_{'p' if x_sign > 0 else 'n'}x_{'p' if y_sign > 0 else 'n'}y",
            )

    router_body.visual(
        Box((0.094, wall_thickness, front_flap_top_z - base_height - front_flap_height)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - wall_thickness / 2.0,
                (base_height + (front_flap_top_z - front_flap_height)) / 2.0,
            )
        ),
        material=shell_white,
        name="front_lower_shell",
    )
    router_body.visual(
        Box((0.094, wall_thickness, wall_top_z - front_flap_top_z)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - wall_thickness / 2.0,
                (front_flap_top_z + wall_top_z) / 2.0,
            )
        ),
        material=shell_white,
        name="front_upper_bridge",
    )

    router_body.visual(
        Box((0.094, wall_thickness, rear_flap_top_z - base_height - rear_flap_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth / 2.0 - wall_thickness / 2.0),
                (base_height + (rear_flap_top_z - rear_flap_height)) / 2.0,
            )
        ),
        material=shell_white,
        name="rear_lower_sill",
    )
    router_body.visual(
        Box((0.094, wall_thickness, wall_top_z - rear_flap_top_z)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth / 2.0 - wall_thickness / 2.0),
                (rear_flap_top_z + wall_top_z) / 2.0,
            )
        ),
        material=shell_white,
        name="rear_upper_shell",
    )

    router_body.visual(
        Box((0.094, 0.094, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=charcoal,
        name="inner_baffle",
    )
    router_body.visual(
        Box((0.052, 0.014, 0.028)),
        origin=Origin(xyz=(0.0, -0.052, 0.034)),
        material=charcoal,
        name="rear_port_cavity",
    )
    router_body.visual(
        Box((0.040, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.049, 0.150)),
        material=charcoal,
        name="setup_recess_shadow",
    )
    router_body.visual(
        top_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, wall_top_z + top_frame_height / 2.0)),
        material=shell_white,
        name="top_frame",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            router_body.visual(
                Cylinder(radius=0.007, length=0.003),
                origin=Origin(
                    xyz=(
                        x_sign * 0.037,
                        y_sign * 0.037,
                        0.0015,
                    )
                ),
                material=rubber,
                name=f"foot_{'p' if x_sign > 0 else 'n'}x_{'p' if y_sign > 0 else 'n'}y",
            )
    router_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    top_vent_panel = model.part("top_vent_panel")
    top_vent_panel.visual(
        vent_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=graphite,
        name="vent_panel",
    )
    top_vent_panel.inertial = Inertial.from_geometry(
        Box((0.082, 0.082, 0.003)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
    )

    setup_dial = model.part("setup_dial")
    setup_dial.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=graphite,
        name="dial_base",
    )
    setup_dial.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=charcoal,
        name="dial_cap",
    )
    setup_dial.visual(
        Box((0.011, 0.0025, 0.0015)),
        origin=Origin(xyz=(0.0065, 0.0, 0.0105)),
        material=soft_blue,
        name="dial_indicator",
    )
    setup_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.017, length=0.010),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    setup_flap = model.part("setup_flap")
    setup_flap.visual(
        Box((front_flap_width, 0.003, front_flap_height)),
        origin=Origin(xyz=(0.0, 0.0015, -front_flap_height / 2.0)),
        material=graphite,
        name="setup_flap_panel",
    )
    setup_flap.visual(
        Box((0.024, 0.0018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0030, -front_flap_height + 0.004)),
        material=hinge_dark,
        name="setup_flap_pull",
    )
    for x_pos in (-0.018, 0.018):
        setup_flap.visual(
            Cylinder(radius=0.003, length=0.012),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_dark,
            name=f"setup_hinge_knuckle_{'left' if x_pos < 0 else 'right'}",
        )
    setup_flap.inertial = Inertial.from_geometry(
        Box((front_flap_width, 0.006, front_flap_height)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0015, -front_flap_height / 2.0)),
    )

    rear_access_flap = model.part("rear_access_flap")
    rear_access_flap.visual(
        Box((rear_flap_width, 0.003, rear_flap_height)),
        origin=Origin(xyz=(0.0, -0.0015, -rear_flap_height / 2.0)),
        material=shell_white,
        name="access_flap_panel",
    )
    rear_access_flap.visual(
        Box((0.018, 0.0015, 0.0035)),
        origin=Origin(xyz=(0.0, -0.0030, -rear_flap_height + 0.004)),
        material=hinge_dark,
        name="access_flap_pull",
    )
    for x_pos in (-0.014, 0.014):
        rear_access_flap.visual(
            Cylinder(radius=0.0028, length=0.010),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_dark,
            name=f"access_hinge_knuckle_{'left' if x_pos < 0 else 'right'}",
        )
    rear_access_flap.inertial = Inertial.from_geometry(
        Box((rear_flap_width, 0.006, rear_flap_height)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.0015, -rear_flap_height / 2.0)),
    )

    model.articulation(
        "body_to_top_vent_panel",
        ArticulationType.FIXED,
        parent=router_body,
        child=top_vent_panel,
        origin=Origin(xyz=(0.0, 0.0, body_height - 0.003)),
    )
    model.articulation(
        "top_panel_to_setup_dial",
        ArticulationType.CONTINUOUS,
        parent=top_vent_panel,
        child=setup_dial,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )
    model.articulation(
        "body_to_setup_flap",
        ArticulationType.REVOLUTE,
        parent=router_body,
        child=setup_flap,
        origin=Origin(xyz=(0.0, body_depth / 2.0, front_flap_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_rear_access_flap",
        ArticulationType.REVOLUTE,
        parent=router_body,
        child=rear_access_flap,
        origin=Origin(xyz=(0.0, -body_depth / 2.0, rear_flap_top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.5,
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

    router_body = object_model.get_part("router_body")
    top_vent_panel = object_model.get_part("top_vent_panel")
    setup_dial = object_model.get_part("setup_dial")
    setup_flap = object_model.get_part("setup_flap")
    rear_access_flap = object_model.get_part("rear_access_flap")

    setup_flap_joint = object_model.get_articulation("body_to_setup_flap")
    rear_flap_joint = object_model.get_articulation("body_to_rear_access_flap")
    dial_joint = object_model.get_articulation("top_panel_to_setup_dial")

    with ctx.pose({setup_flap_joint: 0.0, rear_flap_joint: 0.0, dial_joint: 0.0}):
        ctx.expect_gap(
            setup_flap,
            router_body,
            axis="y",
            min_gap=0.0,
            max_gap=0.0025,
            positive_elem="setup_flap_panel",
            name="setup flap closes flush with the front face",
        )
        ctx.expect_gap(
            router_body,
            rear_access_flap,
            axis="y",
            min_gap=0.0,
            max_gap=0.0025,
            negative_elem="access_flap_panel",
            name="rear access flap closes flush with the rear face",
        )
        ctx.expect_within(
            setup_dial,
            top_vent_panel,
            axes="xy",
            inner_elem="dial_base",
            outer_elem="vent_panel",
            margin=0.002,
            name="setup dial stays centered on the vent panel",
        )
        ctx.expect_gap(
            setup_dial,
            top_vent_panel,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="dial_base",
            negative_elem="vent_panel",
            name="setup dial sits on the top vent panel",
        )

    setup_closed = ctx.part_element_world_aabb(setup_flap, elem="setup_flap_panel")
    with ctx.pose({setup_flap_joint: math.radians(72.0)}):
        setup_open = ctx.part_element_world_aabb(setup_flap, elem="setup_flap_panel")
    ctx.check(
        "setup flap folds upward and outward",
        (
            setup_closed is not None
            and setup_open is not None
            and setup_open[1][1] > setup_closed[1][1] + 0.010
            and setup_open[0][2] > setup_closed[0][2] + 0.020
        ),
        details=f"closed={setup_closed}, open={setup_open}",
    )

    rear_closed = ctx.part_element_world_aabb(rear_access_flap, elem="access_flap_panel")
    with ctx.pose({rear_flap_joint: math.radians(68.0)}):
        rear_open = ctx.part_element_world_aabb(rear_access_flap, elem="access_flap_panel")
    ctx.check(
        "rear access flap swings outward from the back",
        (
            rear_closed is not None
            and rear_open is not None
            and rear_open[0][1] < rear_closed[0][1] - 0.008
            and rear_open[0][2] > rear_closed[0][2] + 0.015
        ),
        details=f"closed={rear_closed}, open={rear_open}",
    )

    dial_rest = ctx.part_element_world_aabb(setup_dial, elem="dial_indicator")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_quarter = ctx.part_element_world_aabb(setup_dial, elem="dial_indicator")
    dial_rest_dx = None if dial_rest is None else dial_rest[1][0] - dial_rest[0][0]
    dial_rest_dy = None if dial_rest is None else dial_rest[1][1] - dial_rest[0][1]
    dial_quarter_dx = None if dial_quarter is None else dial_quarter[1][0] - dial_quarter[0][0]
    dial_quarter_dy = None if dial_quarter is None else dial_quarter[1][1] - dial_quarter[0][1]
    ctx.check(
        "setup dial indicator rotates around the vertical axis",
        (
            dial_rest_dx is not None
            and dial_rest_dy is not None
            and dial_quarter_dx is not None
            and dial_quarter_dy is not None
            and dial_rest_dx > dial_rest_dy * 2.0
            and dial_quarter_dy > dial_quarter_dx * 2.0
        ),
        details=(
            f"rest_aabb={dial_rest}, quarter_turn_aabb={dial_quarter}, "
            f"rest_dx={dial_rest_dx}, rest_dy={dial_rest_dy}, "
            f"quarter_dx={dial_quarter_dx}, quarter_dy={dial_quarter_dy}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
