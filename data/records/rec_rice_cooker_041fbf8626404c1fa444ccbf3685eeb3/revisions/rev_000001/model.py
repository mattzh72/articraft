from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _rounded_vertical_panel(width: float, height: float, depth: float, radius: float):
    """Rounded rectangle in X/Z, extruded along Y for a front appliance face."""
    return ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        depth,
        cap=True,
        center=True,
    ).rotate_x(pi / 2.0)


def _button_cap(width: float, height: float, depth: float, radius: float):
    """Button with rear mounting face at local y=0 and travel into +Y."""
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        depth,
        cap=True,
    ).rotate_x(pi / 2.0)


def _inner_pot_geometry():
    # Thin-walled circular insert, visible when the lid is opened.
    return LatheGeometry.from_shell_profiles(
        [
            (0.118, 0.036),
            (0.132, 0.062),
            (0.143, 0.092),
            (0.148, 0.126),
            (0.150, 0.145),
        ],
        [
            (0.096, 0.040),
            (0.110, 0.066),
            (0.122, 0.096),
            (0.128, 0.126),
            (0.130, 0.140),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_size_rice_cooker")

    warm_white = model.material("warm_white_plastic", rgba=(0.88, 0.86, 0.80, 1.0))
    satin_lid = model.material("satin_lid_plastic", rgba=(0.82, 0.83, 0.79, 1.0))
    dark_panel = model.material("dark_front_panel", rgba=(0.045, 0.050, 0.055, 1.0))
    button_white = model.material("button_white", rgba=(0.96, 0.95, 0.90, 1.0))
    button_grey = model.material("button_grey", rgba=(0.66, 0.68, 0.66, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.020, 0.022, 0.024, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    display_glass = model.material("smoked_display", rgba=(0.02, 0.05, 0.06, 1.0))

    body = model.part("body")
    body_shell = superellipse_side_loft(
        [
            (-0.170, 0.126, 0.018, 0.330),
            (-0.125, 0.145, 0.014, 0.420),
            (-0.020, 0.158, 0.012, 0.455),
            (0.095, 0.154, 0.014, 0.440),
            (0.165, 0.130, 0.018, 0.350),
        ],
        exponents=2.8,
        segments=64,
    )
    body.visual(
        _save_mesh(body_shell, "wide_rounded_body"),
        material=warm_white,
        name="wide_shell",
    )
    body.visual(
        _save_mesh(_inner_pot_geometry(), "inner_pot"),
        material=brushed_steel,
        name="inner_pot",
    )
    body.visual(
        _save_mesh(TorusGeometry(radius=0.145, tube=0.005, radial_segments=18, tubular_segments=72), "pot_rim_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
        material=brushed_steel,
        name="pot_rim",
    )
    body.visual(
        _save_mesh(TorusGeometry(radius=0.119, tube=0.004, radial_segments=16, tubular_segments=64), "gasket_shadow_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=black_rubber,
        name="steam_gasket_shadow",
    )
    body.visual(
        _save_mesh(_rounded_vertical_panel(0.205, 0.090, 0.012, 0.014), "front_panel"),
        origin=Origin(xyz=(0.0, -0.1755, 0.071)),
        material=dark_panel,
        name="front_panel",
    )
    body.visual(
        _save_mesh(_rounded_vertical_panel(0.082, 0.020, 0.004, 0.004), "display_window"),
        origin=Origin(xyz=(0.0, -0.1835, 0.095)),
        material=display_glass,
        name="display_window",
    )
    body.visual(
        Box((0.360, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.174, 0.147)),
        material=button_grey,
        name="rear_hinge_band",
    )
    body.visual(
        Box((0.345, 0.250, 0.008)),
        origin=Origin(xyz=(0.0, -0.005, 0.009)),
        material=black_rubber,
        name="counter_foot_pad",
    )

    lid = model.part("lid")
    lid_shell = superellipse_side_loft(
        [
            (-0.335, 0.018, -0.006, 0.300),
            (-0.278, 0.046, -0.006, 0.382),
            (-0.140, 0.064, -0.004, 0.408),
            (-0.035, 0.048, -0.006, 0.382),
            (0.006, 0.018, -0.006, 0.290),
        ],
        exponents=2.8,
        segments=64,
    )
    lid.visual(
        _save_mesh(lid_shell, "low_broad_lid"),
        material=satin_lid,
        name="lid_shell",
    )
    lid.visual(
        Box((0.330, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.000, -0.0065)),
        material=button_grey,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.075),
        origin=Origin(xyz=(0.0, -0.145, 0.060), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="steam_vent",
    )
    lid.visual(
        Box((0.125, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, -0.327, 0.004)),
        material=button_grey,
        name="front_latch_strike",
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        _save_mesh(_button_cap(0.086, 0.024, 0.015, 0.006), "latch_button_cap"),
        material=button_white,
        name="button_cap",
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        _save_mesh(_button_cap(0.047, 0.021, 0.013, 0.005), "menu_button_0_cap"),
        material=button_grey,
        name="button_cap",
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        _save_mesh(_button_cap(0.047, 0.021, 0.013, 0.005), "menu_button_1_cap"),
        material=button_grey,
        name="button_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.165, 0.176)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "latch_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(0.0, -0.1815, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.009),
    )
    model.articulation(
        "menu_0_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_0,
        origin=Origin(xyz=(-0.040, -0.1815, 0.044)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.006),
    )
    model.articulation(
        "menu_1_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_1,
        origin=Origin(xyz=(0.040, -0.1815, 0.044)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.006),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_button")
    menu_0 = object_model.get_part("menu_button_0")
    menu_1 = object_model.get_part("menu_button_1")

    lid_joint = object_model.get_articulation("body_to_lid")
    latch_joint = object_model.get_articulation("latch_slide")
    menu_0_joint = object_model.get_articulation("menu_0_slide")
    menu_1_joint = object_model.get_articulation("menu_1_slide")

    ctx.expect_gap(
        body,
        latch,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="front_panel",
        negative_elem="button_cap",
        name="latch button sits proud on the front panel",
    )
    ctx.expect_gap(
        body,
        menu_0,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="front_panel",
        negative_elem="button_cap",
        name="first menu button is discrete on the panel",
    )
    ctx.expect_gap(
        body,
        menu_1,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="front_panel",
        negative_elem="button_cap",
        name="second menu button is discrete on the panel",
    )
    ctx.expect_origin_gap(
        menu_1,
        menu_0,
        axis="x",
        min_gap=0.070,
        max_gap=0.090,
        name="menu buttons are separate side-by-side controls",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.25}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward around the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_latch_position = ctx.part_world_position(latch)
    rest_menu_0_position = ctx.part_world_position(menu_0)
    rest_menu_1_position = ctx.part_world_position(menu_1)
    with ctx.pose({latch_joint: 0.009, menu_0_joint: 0.006, menu_1_joint: 0.006}):
        depressed_latch_position = ctx.part_world_position(latch)
        depressed_menu_0_position = ctx.part_world_position(menu_0)
        depressed_menu_1_position = ctx.part_world_position(menu_1)
    ctx.check(
        "front latch translates into the body",
        rest_latch_position is not None
        and depressed_latch_position is not None
        and depressed_latch_position[1] > rest_latch_position[1] + 0.007,
        details=f"rest={rest_latch_position}, depressed={depressed_latch_position}",
    )
    ctx.check(
        "menu buttons depress independently into the body",
        rest_menu_0_position is not None
        and rest_menu_1_position is not None
        and depressed_menu_0_position is not None
        and depressed_menu_1_position is not None
        and depressed_menu_0_position[1] > rest_menu_0_position[1] + 0.004
        and depressed_menu_1_position[1] > rest_menu_1_position[1] + 0.004,
        details=(
            f"menu0 rest={rest_menu_0_position}, depressed={depressed_menu_0_position}; "
            f"menu1 rest={rest_menu_1_position}, depressed={depressed_menu_1_position}"
        ),
    )
    with ctx.pose({menu_0_joint: 0.006}):
        menu_0_only_position = ctx.part_world_position(menu_0)
        menu_1_during_menu_0 = ctx.part_world_position(menu_1)
    with ctx.pose({menu_1_joint: 0.006}):
        menu_0_during_menu_1 = ctx.part_world_position(menu_0)
        menu_1_only_position = ctx.part_world_position(menu_1)
    ctx.check(
        "each menu button can move without driving the other",
        rest_menu_0_position is not None
        and rest_menu_1_position is not None
        and menu_0_only_position is not None
        and menu_1_only_position is not None
        and menu_1_during_menu_0 is not None
        and menu_0_during_menu_1 is not None
        and menu_0_only_position[1] > rest_menu_0_position[1] + 0.004
        and abs(menu_1_during_menu_0[1] - rest_menu_1_position[1]) < 0.0005
        and menu_1_only_position[1] > rest_menu_1_position[1] + 0.004
        and abs(menu_0_during_menu_1[1] - rest_menu_0_position[1]) < 0.0005,
        details=(
            f"menu0_only={menu_0_only_position}, menu1_during_menu0={menu_1_during_menu_0}; "
            f"menu1_only={menu_1_only_position}, menu0_during_menu1={menu_0_during_menu_1}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
