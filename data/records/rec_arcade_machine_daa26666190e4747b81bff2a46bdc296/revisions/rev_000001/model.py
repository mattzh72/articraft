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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_arcade_machine")

    body_blue = model.material("body_blue", rgba=(0.10, 0.18, 0.34, 1.0))
    trim_black = model.material("trim_black", rgba=(0.07, 0.08, 0.09, 1.0))
    base_black = model.material("base_black", rgba=(0.09, 0.09, 0.10, 1.0))
    marquee_glow = model.material("marquee_glow", rgba=(0.95, 0.72, 0.18, 0.78))
    screen_glass = model.material("screen_glass", rgba=(0.10, 0.16, 0.18, 0.40))
    metal_dark = model.material("metal_dark", rgba=(0.32, 0.34, 0.36, 1.0))
    metal_light = model.material("metal_light", rgba=(0.70, 0.73, 0.76, 1.0))
    button_red = model.material("button_red", rgba=(0.72, 0.12, 0.12, 1.0))
    button_blue = model.material("button_blue", rgba=(0.12, 0.32, 0.76, 1.0))
    service_black = model.material("service_black", rgba=(0.03, 0.03, 0.04, 1.0))

    deck_top_z = 0.926

    def _sleeve_mesh(name: str, *, outer_radius: float, inner_radius: float, height: float):
        outer_profile = [
            (outer_radius, 0.0),
            (outer_radius, height),
        ]
        inner_profile = [
            (inner_radius, 0.001),
            (inner_radius, height - 0.001),
        ]
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=48,
            ),
            name,
        )

    button_guide_mesh = _sleeve_mesh(
        "button_guide_sleeve",
        outer_radius=0.020,
        inner_radius=0.0135,
        height=0.012,
    )
    spinner_bezel_mesh = _sleeve_mesh(
        "spinner_bezel_ring",
        outer_radius=0.053,
        inner_radius=0.043,
        height=0.006,
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.52, 0.54, 0.07)),
        origin=Origin(xyz=(-0.04, 0.0, 0.035)),
        material=base_black,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.30, 0.24, 0.80)),
        origin=Origin(xyz=(-0.01, 0.0, 0.47)),
        material=body_blue,
        name="pedestal_body",
    )
    cabinet.visual(
        Box((0.22, 0.44, 0.022)),
        origin=Origin(xyz=(0.11, 0.0, 0.915)),
        material=trim_black,
        name="control_top",
    )
    cabinet.visual(
        Box((0.020, 0.40, 0.185)),
        origin=Origin(xyz=(0.157, 0.0, 0.828), rpy=(0.0, -0.56, 0.0)),
        material=body_blue,
        name="control_apron",
    )
    cabinet.visual(
        Box((0.012, 0.44, 0.034)),
        origin=Origin(xyz=(0.217, 0.0, 0.902)),
        material=trim_black,
        name="control_front_lip",
    )
    cabinet.visual(
        Box((0.24, 0.62, 0.29)),
        origin=Origin(xyz=(-0.02, 0.0, 1.115)),
        material=body_blue,
        name="marquee_head",
    )
    cabinet.visual(
        Box((0.14, 0.28, 0.12)),
        origin=Origin(xyz=(-0.03, 0.0, 0.915)),
        material=body_blue,
        name="monitor_neck",
    )
    cabinet.visual(
        Box((0.012, 0.50, 0.090)),
        origin=Origin(xyz=(0.106, 0.0, 1.175)),
        material=marquee_glow,
        name="marquee_lens",
    )
    cabinet.visual(
        Box((0.010, 0.44, 0.185)),
        origin=Origin(xyz=(0.105, 0.0, 1.055)),
        material=screen_glass,
        name="screen_glass",
    )
    cabinet.visual(
        Box((0.035, 0.50, 0.018)),
        origin=Origin(xyz=(0.088, 0.0, 0.992), rpy=(0.0, -0.54, 0.0)),
        material=trim_black,
        name="screen_bezel_lower",
    )
    cabinet.visual(
        Box((0.008, 0.12, 0.22)),
        origin=Origin(xyz=(0.144, 0.0, 0.42)),
        material=metal_dark,
        name="coin_door",
    )
    cabinet.visual(
        Box((0.008, 0.19, 0.48)),
        origin=Origin(xyz=(-0.147, 0.0, 0.45)),
        material=service_black,
        name="service_bay_shadow",
    )
    cabinet.visual(
        Box((0.020, 0.020, 0.52)),
        origin=Origin(xyz=(-0.150, 0.110, 0.45)),
        material=metal_dark,
        name="cover_hinge_post",
    )

    spinner_origin = Origin(xyz=(0.100, -0.075, deck_top_z))
    left_button_origin = Origin(xyz=(0.118, 0.040, deck_top_z))
    right_button_origin = Origin(xyz=(0.118, 0.105, deck_top_z))

    cabinet.visual(
        spinner_bezel_mesh,
        origin=spinner_origin,
        material=metal_dark,
        name="spinner_bezel",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.100, -0.075, deck_top_z + 0.003)),
        material=metal_dark,
        name="spinner_spindle_post",
    )
    cabinet.visual(
        button_guide_mesh,
        origin=left_button_origin,
        material=metal_light,
        name="left_button_guide",
    )
    cabinet.visual(
        button_guide_mesh,
        origin=right_button_origin,
        material=metal_light,
        name="right_button_guide",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.52, 0.62, 1.23)),
        mass=42.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.615)),
    )

    spinner = model.part("spinner")
    spinner.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=metal_light,
        name="spinner_disc",
    )
    spinner.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=metal_dark,
        name="spinner_hub",
    )
    spinner.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.015, 0.0, 0.018)),
        material=trim_black,
        name="spinner_grip",
    )
    spinner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.024),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    left_button = model.part("left_button")
    left_button.visual(
        Cylinder(radius=0.016, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=button_red,
        name="button_cap",
    )
    left_button.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=button_red,
        name="button_skirt",
    )
    left_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.012),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    right_button = model.part("right_button")
    right_button.visual(
        Cylinder(radius=0.016, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=button_blue,
        name="button_cap",
    )
    right_button.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=button_blue,
        name="button_skirt",
    )
    right_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.012),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    rear_cover = model.part("rear_cover")
    rear_cover.visual(
        Box((0.010, 0.20, 0.48)),
        origin=Origin(xyz=(-0.005, -0.10, 0.24)),
        material=metal_dark,
        name="cover_panel",
    )
    rear_cover.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(-0.009, -0.165, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_light,
        name="cover_latch",
    )
    rear_cover.inertial = Inertial.from_geometry(
        Box((0.012, 0.21, 0.50)),
        mass=1.2,
        origin=Origin(xyz=(-0.005, -0.10, 0.24)),
    )

    model.articulation(
        "cabinet_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=spinner,
        origin=Origin(xyz=(0.100, -0.075, deck_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=14.0),
    )
    model.articulation(
        "cabinet_to_left_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=left_button,
        origin=Origin(xyz=(0.118, 0.040, deck_top_z + 0.004)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )
    model.articulation(
        "cabinet_to_right_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=right_button,
        origin=Origin(xyz=(0.118, 0.105, deck_top_z + 0.004)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )
    model.articulation(
        "cabinet_to_rear_cover",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=rear_cover,
        origin=Origin(xyz=(-0.160, 0.100, 0.21)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=0.0,
            upper=1.8,
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

    cabinet = object_model.get_part("cabinet")
    spinner = object_model.get_part("spinner")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    rear_cover = object_model.get_part("rear_cover")

    spinner_joint = object_model.get_articulation("cabinet_to_spinner")
    left_button_joint = object_model.get_articulation("cabinet_to_left_button")
    right_button_joint = object_model.get_articulation("cabinet_to_right_button")
    rear_cover_joint = object_model.get_articulation("cabinet_to_rear_cover")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) * 0.5,
            (lo[1] + hi[1]) * 0.5,
            (lo[2] + hi[2]) * 0.5,
        )

    ctx.expect_overlap(
        spinner,
        cabinet,
        axes="xy",
        elem_a="spinner_disc",
        elem_b="control_top",
        min_overlap=0.05,
        name="spinner sits over the control ledge",
    )
    ctx.expect_gap(
        spinner,
        cabinet,
        axis="z",
        positive_elem="spinner_disc",
        negative_elem="control_top",
        min_gap=0.0,
        max_gap=0.006,
        name="spinner disc sits just above the control ledge",
    )
    ctx.expect_contact(
        spinner,
        cabinet,
        elem_a="spinner_hub",
        elem_b="spinner_spindle_post",
        contact_tol=1e-6,
        name="spinner hub seats on the spindle post",
    )
    ctx.expect_overlap(
        left_button,
        cabinet,
        axes="xy",
        elem_a="button_cap",
        elem_b="left_button_guide",
        min_overlap=0.025,
        name="left button is centered in its guide",
    )
    ctx.expect_overlap(
        right_button,
        cabinet,
        axes="xy",
        elem_a="button_cap",
        elem_b="right_button_guide",
        min_overlap=0.025,
        name="right button is centered in its guide",
    )
    ctx.expect_gap(
        cabinet,
        rear_cover,
        axis="x",
        positive_elem="pedestal_body",
        negative_elem="cover_panel",
        min_gap=0.0,
        max_gap=0.006,
        name="rear cover closes near the cabinet back",
    )
    ctx.expect_overlap(
        cabinet,
        rear_cover,
        axes="yz",
        elem_a="service_bay_shadow",
        elem_b="cover_panel",
        min_overlap=0.18,
        name="rear cover spans the service opening",
    )

    rest_left_cap = _aabb_center(ctx.part_element_world_aabb(left_button, elem="button_cap"))
    rest_right_cap = _aabb_center(ctx.part_element_world_aabb(right_button, elem="button_cap"))
    with ctx.pose({left_button_joint: 0.003, right_button_joint: 0.003}):
        pressed_left_cap = _aabb_center(ctx.part_element_world_aabb(left_button, elem="button_cap"))
        pressed_right_cap = _aabb_center(ctx.part_element_world_aabb(right_button, elem="button_cap"))
    ctx.check(
        "left button depresses downward",
        rest_left_cap is not None
        and pressed_left_cap is not None
        and pressed_left_cap[2] < rest_left_cap[2] - 0.0025,
        details=f"rest={rest_left_cap}, pressed={pressed_left_cap}",
    )
    ctx.check(
        "right button depresses downward",
        rest_right_cap is not None
        and pressed_right_cap is not None
        and pressed_right_cap[2] < rest_right_cap[2] - 0.0025,
        details=f"rest={rest_right_cap}, pressed={pressed_right_cap}",
    )

    rest_grip = _aabb_center(ctx.part_element_world_aabb(spinner, elem="spinner_grip"))
    with ctx.pose({spinner_joint: math.pi / 2.0}):
        quarter_turn_grip = _aabb_center(ctx.part_element_world_aabb(spinner, elem="spinner_grip"))
    ctx.check(
        "spinner grip moves around the vertical spindle",
        rest_grip is not None
        and quarter_turn_grip is not None
        and abs(rest_grip[0] - quarter_turn_grip[0]) > 0.01
        and abs(rest_grip[1] - quarter_turn_grip[1]) > 0.01,
        details=f"rest={rest_grip}, quarter_turn={quarter_turn_grip}",
    )

    closed_cover_center = _aabb_center(ctx.part_element_world_aabb(rear_cover, elem="cover_panel"))
    with ctx.pose({rear_cover_joint: 1.2}):
        open_cover_center = _aabb_center(ctx.part_element_world_aabb(rear_cover, elem="cover_panel"))
    ctx.check(
        "rear cover swings outward from the cabinet back",
        closed_cover_center is not None
        and open_cover_center is not None
        and open_cover_center[0] < closed_cover_center[0] - 0.05,
        details=f"closed={closed_cover_center}, open={open_cover_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
