from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _oval_housing_shell(
    *,
    width: float,
    depth: float,
    inner_width: float,
    inner_depth: float,
    z0: float,
    z1: float,
    segments: int = 80,
) -> MeshGeometry:
    """Open-top superellipse shell: continuous side wall, top rim, and bottom annulus."""
    outer = superellipse_profile(width, depth, exponent=2.55, segments=segments)
    inner = superellipse_profile(inner_width, inner_depth, exponent=2.35, segments=segments)
    geom = MeshGeometry()

    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for (ox, oy), (ix, iy) in zip(outer, inner):
        outer_bottom.append(geom.add_vertex(ox, oy, z0))
        outer_top.append(geom.add_vertex(ox, oy, z1))
        inner_bottom.append(geom.add_vertex(ix, iy, z0))
        inner_top.append(geom.add_vertex(ix, iy, z1))

    for i in range(segments):
        j = (i + 1) % segments
        _quad(geom, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        # Reverse winding for the inward-facing wall.
        _quad(geom, inner_bottom[j], inner_bottom[i], inner_top[i], inner_top[j])
        _quad(geom, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        _quad(geom, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])

    return geom


def _oval_lid_crown(
    *,
    width: float,
    depth: float,
    rear_y: float,
    base_z: float,
    crown_height: float,
    segments: int = 80,
) -> MeshGeometry:
    """Low domed oval lid crown in the lid frame; rear edge lies near the hinge."""
    center_y = rear_y - depth / 2.0
    base_loop = [(x, y + center_y) for x, y in superellipse_profile(width, depth, exponent=2.45, segments=segments)]
    geom = MeshGeometry()

    rings: list[list[int]] = []
    # Scale rings toward the center to make a soft crown rather than a flat puck.
    for scale, z in (
        (1.00, base_z),
        (0.99, base_z + 0.010),
        (0.82, base_z + crown_height * 0.66),
        (0.42, base_z + crown_height),
    ):
        ring: list[int] = []
        for x, y in base_loop:
            ring.append(geom.add_vertex(x * scale, center_y + (y - center_y) * scale, z))
        rings.append(ring)

    bottom_center = geom.add_vertex(0.0, center_y, base_z)
    top_center = geom.add_vertex(0.0, center_y, base_z + crown_height + 0.006)

    for r0, r1 in zip(rings, rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            _quad(geom, r0[i], r0[j], r1[j], r1[i])

    bottom = rings[0]
    top = rings[-1]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom_center, bottom[j], bottom[i])
        geom.add_face(top_center, top[i], top[j])

    return geom


def _rounded_front_mesh(width: float, height: float, thickness: float, radius: float) -> MeshGeometry:
    """Rounded rectangle in local XZ with shallow thickness in local Y."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
        cap=True,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return geom


def _rounded_horizontal_mesh(width: float, depth: float, thickness: float, radius: float) -> MeshGeometry:
    """Rounded rectangle in local XY with shallow thickness in local Z."""
    return ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=8),
        thickness,
        cap=True,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_rice_cooker")

    porcelain = model.material("warm_white_porcelain", rgba=(0.92, 0.90, 0.84, 1.0))
    satin_lid = model.material("satin_warm_gray", rgba=(0.72, 0.72, 0.68, 1.0))
    charcoal = model.material("charcoal_glass", rgba=(0.025, 0.030, 0.035, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    button_gray = model.material("soft_touch_gray", rgba=(0.45, 0.47, 0.48, 1.0))
    latch_white = model.material("latch_white", rgba=(0.86, 0.86, 0.82, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    vent_black = model.material("vent_black", rgba=(0.02, 0.02, 0.018, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(
            _oval_housing_shell(
                width=0.43,
                depth=0.36,
                inner_width=0.32,
                inner_depth=0.255,
                z0=0.040,
                z1=0.245,
            ),
            "oval_housing_shell",
        ),
        material=porcelain,
        name="oval_shell",
    )
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry(superellipse_profile(0.395, 0.325, exponent=2.7, segments=80), 0.040, cap=True, center=True),
            "lower_plinth",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_rubber,
        name="lower_plinth",
    )
    body.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.060, 0.050), (0.122, 0.078), (0.150, 0.210), (0.157, 0.232)],
                [(0.040, 0.064), (0.102, 0.090), (0.132, 0.205), (0.144, 0.223)],
                segments=80,
                end_cap="round",
                lip_samples=8,
            ),
            "inner_cooking_pot",
        ),
        material=steel,
        name="inner_pot",
    )
    body.visual(
        Cylinder(radius=0.146, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.222)),
        material=charcoal,
        name="pot_shadow",
    )
    body.visual(
        mesh_from_geometry(_rounded_front_mesh(0.285, 0.125, 0.012, 0.026), "front_control_panel"),
        origin=Origin(xyz=(0.0, -0.184, 0.136)),
        material=charcoal,
        name="control_panel",
    )
    body.visual(
        Box((0.210, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.194, 0.194)),
        material=button_gray,
        name="display_slot",
    )
    for x in (-0.145, 0.145):
        body.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(xyz=(x, -0.110, 0.018)),
            material=dark_rubber,
            name=f"front_foot_{0 if x < 0 else 1}",
        )
    for x in (-0.145, 0.145):
        body.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(xyz=(x, 0.110, 0.018)),
            material=dark_rubber,
            name=f"rear_foot_{0 if x < 0 else 1}",
        )
    for x in (-0.155, 0.155):
        body.visual(
            Cylinder(radius=0.016, length=0.052),
            origin=Origin(xyz=(x, 0.166, 0.270), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_lid,
            name=f"hinge_knuckle_{0 if x < 0 else 1}",
        )
        body.visual(
            Box((0.050, 0.030, 0.040)),
            origin=Origin(xyz=(x, 0.156, 0.250)),
            material=porcelain,
            name=f"hinge_boss_{0 if x < 0 else 1}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            _oval_lid_crown(width=0.385, depth=0.318, rear_y=0.006, base_z=0.000, crown_height=0.052),
            "lid_crown",
        ),
        material=satin_lid,
        name="lid_crown",
    )
    lid.visual(
        Cylinder(radius=0.0135, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_lid,
        name="hinge_barrel",
    )
    lid.visual(
        mesh_from_geometry(_rounded_horizontal_mesh(0.160, 0.056, 0.004, 0.026), "handle_recess"),
        origin=Origin(xyz=(0.0, -0.178, 0.055), rpy=(0.0, 0.0, 0.0)),
        material=button_gray,
        name="handle_recess",
    )
    lid.visual(
        Box((0.185, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, 0.006)),
        material=satin_lid,
        name="hinge_strap",
    )
    lid.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.105, -0.065, 0.048)),
        material=vent_black,
        name="steam_vent",
    )
    for i, dx in enumerate((-0.010, 0.0, 0.010)):
        lid.visual(
            Box((0.004, 0.024, 0.002)),
            origin=Origin(xyz=(0.105 + dx, -0.065, 0.054)),
            material=satin_lid,
            name=f"vent_slit_{i}",
        )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_geometry(_rounded_front_mesh(0.110, 0.034, 0.016, 0.016), "latch_button_cap"),
        material=latch_white,
        name="button_cap",
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        mesh_from_geometry(_rounded_front_mesh(0.075, 0.036, 0.014, 0.017), "menu_button_0_cap"),
        material=button_gray,
        name="button_cap",
    )
    menu_button_0.visual(
        Box((0.035, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, -0.008, 0.000)),
        material=charcoal,
        name="button_mark",
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        mesh_from_geometry(_rounded_front_mesh(0.075, 0.036, 0.014, 0.017), "menu_button_1_cap"),
        material=button_gray,
        name="button_cap",
    )
    menu_button_1.visual(
        Box((0.004, 0.002, 0.020)),
        origin=Origin(xyz=(0.0, -0.008, 0.000)),
        material=charcoal,
        name="button_mark",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.166, 0.270)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.22),
    )
    model.articulation(
        "body_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(0.0, -0.198, 0.172)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.012),
    )
    model.articulation(
        "body_to_menu_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_0,
        origin=Origin(xyz=(-0.055, -0.197, 0.108)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.10, lower=0.0, upper=0.007),
    )
    model.articulation(
        "body_to_menu_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_1,
        origin=Origin(xyz=(0.055, -0.197, 0.108)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.10, lower=0.0, upper=0.007),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch_button = object_model.get_part("latch_button")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_slide = object_model.get_articulation("body_to_latch_button")
    menu_slide_0 = object_model.get_articulation("body_to_menu_button_0")
    menu_slide_1 = object_model.get_articulation("body_to_menu_button_1")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_crown",
        negative_elem="oval_shell",
        min_gap=0.005,
        max_gap=0.050,
        name="closed lid sits just above the oval top rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_crown",
        elem_b="oval_shell",
        min_overlap=0.18,
        name="lid covers the top opening footprint",
    )
    ctx.expect_gap(
        body,
        latch_button,
        axis="y",
        positive_elem="control_panel",
        negative_elem="button_cap",
        min_gap=0.0,
        max_gap=0.002,
        name="latch button is seated on the front panel",
    )
    for idx, button in enumerate((menu_button_0, menu_button_1)):
        ctx.expect_gap(
            body,
            button,
            axis="y",
            positive_elem="control_panel",
            negative_elem="button_cap",
            min_gap=0.0,
            max_gap=0.002,
            name=f"menu button {idx} is seated on the front panel",
        )
    ctx.expect_gap(
        menu_button_1,
        menu_button_0,
        axis="x",
        positive_elem="button_cap",
        negative_elem="button_cap",
        min_gap=0.020,
        max_gap=0.050,
        name="menu buttons are visibly separate",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_crown")
    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_crown")
    ctx.check(
        "rear hinge opens the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    latch_rest = ctx.part_world_position(latch_button)
    menu_0_rest = ctx.part_world_position(menu_button_0)
    menu_1_rest = ctx.part_world_position(menu_button_1)
    with ctx.pose({latch_slide: 0.012, menu_slide_0: 0.007}):
        latch_pressed = ctx.part_world_position(latch_button)
        menu_0_pressed = ctx.part_world_position(menu_button_0)
        menu_1_unpressed = ctx.part_world_position(menu_button_1)
    ctx.check(
        "latch button translates into the body",
        latch_rest is not None and latch_pressed is not None and latch_pressed[1] > latch_rest[1] + 0.010,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )
    ctx.check(
        "menu buttons depress independently",
        menu_0_rest is not None
        and menu_0_pressed is not None
        and menu_1_rest is not None
        and menu_1_unpressed is not None
        and menu_0_pressed[1] > menu_0_rest[1] + 0.005
        and abs(menu_1_unpressed[1] - menu_1_rest[1]) < 0.001,
        details=f"menu0 rest/pressed={menu_0_rest}/{menu_0_pressed}, menu1 rest/unpressed={menu_1_rest}/{menu_1_unpressed}",
    )

    return ctx.report()


object_model = build_object_model()
