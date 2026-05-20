from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_drawer_bedside_nightstand")

    warm_oak = Material("warm_oak", color=(0.72, 0.46, 0.25, 1.0))
    drawer_oak = Material("drawer_oak", color=(0.78, 0.52, 0.30, 1.0))
    shadow = Material("shadow_gap", color=(0.025, 0.022, 0.018, 1.0))
    dark_metal = Material("dark_metal", color=(0.03, 0.032, 0.035, 1.0))
    rail_metal = Material("brushed_rail", color=(0.62, 0.62, 0.58, 1.0))

    body_w = 0.56
    body_d = 0.44
    body_h = 0.56
    body_z = 0.05
    panel_t = 0.025
    back_t = 0.020
    shelf_t = 0.018
    rail_len = 0.34
    rail_h = 0.018
    rail_t = 0.008

    cabinet = model.part("cabinet")

    # Square fronted, hollow carcass with a real top, bottom, sides, back, and
    # center divider rather than a solid block.
    cabinet.visual(
        Box((body_d, panel_t, body_h)),
        origin=Origin(xyz=(0.0, -(body_w - panel_t) / 2.0, body_z + body_h / 2.0)),
        material=warm_oak,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((body_d, panel_t, body_h)),
        origin=Origin(xyz=(0.0, (body_w - panel_t) / 2.0, body_z + body_h / 2.0)),
        material=warm_oak,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((body_d, body_w, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, body_z + body_h - panel_t / 2.0)),
        material=warm_oak,
        name="top_panel",
    )
    cabinet.visual(
        Box((body_d, body_w, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, body_z + panel_t / 2.0)),
        material=warm_oak,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((back_t, body_w, body_h)),
        origin=Origin(xyz=(-body_d / 2.0 + back_t / 2.0, 0.0, body_z + body_h / 2.0)),
        material=warm_oak,
        name="back_panel",
    )
    cabinet.visual(
        Box((body_d, body_w - 2.0 * panel_t, shelf_t)),
        origin=Origin(xyz=(0.0, 0.0, body_z + body_h / 2.0)),
        material=warm_oak,
        name="center_divider",
    )

    # Black reveal strips just inside the drawer gaps keep the square body from
    # reading as a featureless cube.
    front_x = body_d / 2.0
    cabinet.visual(
        Box((0.010, body_w - 2.0 * panel_t, 0.012)),
        origin=Origin(xyz=(front_x - 0.006, 0.0, body_z + body_h / 2.0)),
        material=shadow,
        name="middle_shadow",
    )
    cabinet.visual(
        Box((0.010, 0.012, body_h - 2.0 * panel_t)),
        origin=Origin(xyz=(front_x - 0.006, -(body_w / 2.0 - panel_t - 0.006), body_z + body_h / 2.0)),
        material=shadow,
        name="side_shadow_0",
    )
    cabinet.visual(
        Box((0.010, 0.012, body_h - 2.0 * panel_t)),
        origin=Origin(xyz=(front_x - 0.006, (body_w / 2.0 - panel_t - 0.006), body_z + body_h / 2.0)),
        material=shadow,
        name="side_shadow_1",
    )

    # Short dark feet support the wooden case while keeping the main body square.
    for i, (x, y) in enumerate(
        (
            (0.145, -0.195),
            (0.145, 0.195),
            (-0.145, -0.195),
            (-0.145, 0.195),
        )
    ):
        cabinet.visual(
            Box((0.060, 0.060, body_z)),
            origin=Origin(xyz=(x, y, body_z / 2.0)),
            material=dark_metal,
            name=f"foot_{i}",
        )

    lower_center_z = body_z + 0.150
    upper_center_z = body_z + 0.410
    rail_y = body_w / 2.0 - panel_t - rail_t / 2.0
    rail_x = 0.035
    for rail_names, zc in (
        (("lower_fixed_rail_0", "lower_fixed_rail_1"), lower_center_z),
        (("upper_fixed_rail_0", "upper_fixed_rail_1"), upper_center_z),
    ):
        for rail_name, y in zip(rail_names, (-rail_y, rail_y)):
            cabinet.visual(
                Box((rail_len, rail_t, rail_h)),
                origin=Origin(xyz=(rail_x, y, zc - 0.055)),
                material=rail_metal,
                name=rail_name,
            )

    def add_drawer(part_name: str, center_z: float) -> object:
        drawer = model.part(part_name)
        face_t = 0.024
        face_w = 0.500
        face_h = 0.245
        tray_len = 0.380
        tray_w = 0.460
        wall_t = 0.014
        tray_h = 0.102
        bottom_t = 0.012
        local_tray_x = -face_t / 2.0 - tray_len / 2.0 + 0.001
        local_tray_z = -0.078
        rail_local_y = tray_w / 2.0 + 0.014

        drawer.visual(
            Box((face_t, face_w, face_h)),
            origin=Origin(),
            material=drawer_oak,
            name="front_panel",
        )
        drawer.visual(
            Box((tray_len, tray_w, bottom_t)),
            origin=Origin(xyz=(local_tray_x, 0.0, local_tray_z)),
            material=drawer_oak,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((tray_len, wall_t, tray_h)),
            origin=Origin(xyz=(local_tray_x, -(tray_w - wall_t) / 2.0, local_tray_z + (tray_h + bottom_t) / 2.0 - 0.001)),
            material=drawer_oak,
            name="drawer_side_0",
        )
        drawer.visual(
            Box((tray_len, wall_t, tray_h)),
            origin=Origin(xyz=(local_tray_x, (tray_w - wall_t) / 2.0, local_tray_z + (tray_h + bottom_t) / 2.0 - 0.001)),
            material=drawer_oak,
            name="drawer_side_1",
        )
        drawer.visual(
            Box((wall_t, tray_w, tray_h)),
            origin=Origin(xyz=(-face_t / 2.0 - tray_len + wall_t / 2.0 + 0.002, 0.0, local_tray_z + (tray_h + bottom_t) / 2.0 - 0.001)),
            material=drawer_oak,
            name="drawer_back",
        )

        # Sliding members ride beside the wooden drawer box and remain engaged
        # with the fixed cabinet rails through the full travel.
        for rail_name, y in zip(("slide_rail_0", "slide_rail_1"), (-rail_local_y, rail_local_y)):
            drawer.visual(
                Box((0.360, 0.006, 0.014)),
                origin=Origin(xyz=(-0.188, y, -0.055)),
                material=rail_metal,
                name=rail_name,
            )

        # A low-profile flat bar pull with two standoffs, all rigidly mounted to
        # the drawer front so it moves with the drawer.
        post_depth = 0.022
        bar_depth = 0.014
        post_x = face_t / 2.0 + post_depth / 2.0 - 0.001
        bar_x = face_t / 2.0 + post_depth + bar_depth / 2.0 - 0.002
        for i, y in enumerate((-0.130, 0.130)):
            drawer.visual(
                Box((post_depth, 0.026, 0.044)),
                origin=Origin(xyz=(post_x, y, 0.010)),
                material=dark_metal,
                name=f"handle_post_{i}",
            )
        drawer.visual(
            Box((bar_depth, 0.340, 0.022)),
            origin=Origin(xyz=(bar_x, 0.0, 0.010)),
            material=dark_metal,
            name="pull_bar",
        )

        model.articulation(
            f"cabinet_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(front_x + face_t / 2.0 + 0.002, 0.0, center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.220),
        )
        return drawer

    add_drawer("lower_drawer", lower_center_z)
    add_drawer("upper_drawer", upper_center_z)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lower = object_model.get_part("lower_drawer")
    upper = object_model.get_part("upper_drawer")
    lower_slide = object_model.get_articulation("cabinet_to_lower_drawer")
    upper_slide = object_model.get_articulation("cabinet_to_upper_drawer")

    # Closed drawers sit just proud of the square carcass front with a narrow,
    # deliberate reveal rather than penetrating the cabinet.
    ctx.expect_gap(
        lower,
        cabinet,
        axis="x",
        positive_elem="front_panel",
        max_gap=0.004,
        max_penetration=0.0,
        name="lower drawer front sits proud",
    )
    ctx.expect_gap(
        upper,
        cabinet,
        axis="x",
        positive_elem="front_panel",
        max_gap=0.004,
        max_penetration=0.0,
        name="upper drawer front sits proud",
    )
    ctx.expect_overlap(
        lower,
        upper,
        axes="y",
        elem_a="front_panel",
        elem_b="front_panel",
        min_overlap=0.49,
        name="drawer fronts have equal width",
    )

    lower_aabb = ctx.part_element_world_aabb(lower, elem="front_panel")
    upper_aabb = ctx.part_element_world_aabb(upper, elem="front_panel")
    equal_height = False
    if lower_aabb is not None and upper_aabb is not None:
        lower_h = lower_aabb[1][2] - lower_aabb[0][2]
        upper_h = upper_aabb[1][2] - upper_aabb[0][2]
        equal_height = abs(lower_h - upper_h) < 1e-6 and lower_h > 0.23
    ctx.check("two drawer fronts are equal height", equal_height)

    lower_rest = ctx.part_world_position(lower)
    with ctx.pose({lower_slide: 0.220}):
        lower_extended = ctx.part_world_position(lower)
        ctx.expect_overlap(
            lower,
            cabinet,
            axes="x",
            elem_a="slide_rail_0",
            elem_b="lower_fixed_rail_0",
            min_overlap=0.090,
            name="lower drawer rail remains retained",
        )
    upper_rest = ctx.part_world_position(upper)
    with ctx.pose({upper_slide: 0.220}):
        upper_extended = ctx.part_world_position(upper)
        ctx.expect_overlap(
            upper,
            cabinet,
            axes="x",
            elem_a="slide_rail_1",
            elem_b="upper_fixed_rail_1",
            min_overlap=0.090,
            name="upper drawer rail remains retained",
        )

    ctx.check(
        "lower drawer extends outward",
        lower_rest is not None and lower_extended is not None and lower_extended[0] > lower_rest[0] + 0.20,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )
    ctx.check(
        "upper drawer extends outward",
        upper_rest is not None and upper_extended is not None and upper_extended[0] > upper_rest[0] + 0.20,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    return ctx.report()


object_model = build_object_model()
