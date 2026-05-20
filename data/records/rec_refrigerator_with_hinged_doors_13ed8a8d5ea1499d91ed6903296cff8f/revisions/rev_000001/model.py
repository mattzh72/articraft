from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel(width: float, height: float, thickness: float, radius: float, name: str):
    """Rounded rectangular appliance panel in local X/Z, with thickness along local Y."""
    profile = rounded_rect_profile(width, height, radius, corner_segments=8)
    geom = ExtrudeGeometry(profile, thickness, center=True)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_top_freezer_refrigerator")

    white = model.material("warm_white_enamel", rgba=(0.92, 0.91, 0.86, 1.0))
    satin = model.material("satin_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    dark = model.material("charcoal_shadow", rgba=(0.025, 0.027, 0.030, 1.0))
    liner = model.material("white_plastic_liner", rgba=(0.96, 0.96, 0.92, 1.0))
    gasket = model.material("soft_black_gasket", rgba=(0.015, 0.015, 0.014, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.70, 0.88, 0.95, 0.42))
    clear = model.material("clear_crisper_plastic", rgba=(0.72, 0.90, 1.0, 0.38))
    trim = model.material("brushed_dark_trim", rgba=(0.18, 0.19, 0.19, 1.0))

    width = 0.86
    depth = 0.72
    height = 1.84
    wall = 0.045
    front_y = -depth / 2.0
    back_y = depth / 2.0
    hinge_x = -width / 2.0 - 0.018
    door_y = front_y - 0.038
    door_thick = 0.070
    door_width = 0.84

    cabinet = model.part("cabinet")

    # Substantial hollow refrigerator case: metal outside, white liner inside.
    cabinet.visual(Box((wall, depth, height)), origin=Origin(xyz=(-width / 2 + wall / 2, 0.0, height / 2)), material=white, name="side_wall_0")
    cabinet.visual(Box((wall, depth, height)), origin=Origin(xyz=(width / 2 - wall / 2, 0.0, height / 2)), material=white, name="side_wall_1")
    cabinet.visual(Box((width, wall, height)), origin=Origin(xyz=(0.0, back_y - wall / 2, height / 2)), material=white, name="back_wall")
    cabinet.visual(Box((width, depth, wall)), origin=Origin(xyz=(0.0, 0.0, height - wall / 2)), material=white, name="top_cap")
    cabinet.visual(Box((width, depth, wall)), origin=Origin(xyz=(0.0, 0.0, wall / 2)), material=white, name="bottom_pan")
    cabinet.visual(Box((width, depth, 0.052)), origin=Origin(xyz=(0.0, 0.0, 1.255)), material=white, name="freezer_divider")

    # Toe kick and vented compressor grille.
    cabinet.visual(Box((width, 0.075, 0.115)), origin=Origin(xyz=(0.0, front_y + 0.038, 0.060)), material=trim, name="toe_kick")
    grille = VentGrilleGeometry(
        (0.64, 0.070),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=0.018,
        slat_pitch=0.013,
        slat_width=0.006,
        corner_radius=0.006,
        slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=2, divider_width=0.004),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.001),
        sleeve=VentGrilleSleeve(style="none"),
        center=True,
    )
    cabinet.visual(mesh_from_geometry(grille, "toe_grille"), origin=Origin(xyz=(0.0, front_y + 0.006, 0.070), rpy=(pi / 2, 0.0, 0.0)), material=trim, name="toe_grille")

    # Interior shelves, rails, and control box are fixed to the cabinet and touch the liner.
    cabinet.visual(Box((width - 2 * wall, 0.49, 0.016)), origin=Origin(xyz=(0.0, 0.060, 0.765)), material=glass, name="lower_glass_shelf")
    cabinet.visual(Box((width - 2 * wall, 0.47, 0.016)), origin=Origin(xyz=(0.0, 0.070, 1.070)), material=glass, name="upper_glass_shelf")
    cabinet.visual(Box((width - 2 * wall, 0.43, 0.014)), origin=Origin(xyz=(0.0, 0.070, 1.545)), material=glass, name="freezer_shelf")
    for i, x in enumerate((-0.376, -0.028, 0.028, 0.376)):
        cabinet.visual(Box((0.018, 0.54, 0.020)), origin=Origin(xyz=(x, 0.080, 0.354)), material=liner, name=f"crisper_rail_{i}")
    cabinet.visual(Box((0.018, 0.54, 0.020)), origin=Origin(xyz=(0.0, 0.080, 0.354)), material=liner, name="center_crisper_rail")
    cabinet.visual(Box((0.42, 0.060, 0.055)), origin=Origin(xyz=(-0.065, -0.155, 1.215)), material=liner, name="thermostat_housing")

    # Exposed hinge barrels tied to the case with small hinge leaves.
    for i, z in enumerate((0.44, 1.05, 1.48, 1.72)):
        cabinet.visual(Cylinder(radius=0.017, length=0.160), origin=Origin(xyz=(hinge_x, door_y, z)), material=satin, name=f"hinge_barrel_{i}")
        cabinet.visual(Box((0.018, 0.040, 0.120)), origin=Origin(xyz=(hinge_x + 0.009, door_y + 0.020, z)), material=satin, name=f"hinge_leaf_{i}")

    # Lower fresh-food door.
    fridge_door = model.part("fridge_door")
    fridge_h = 1.105
    fridge_z = 0.685
    door_mesh = _rounded_panel(door_width, fridge_h, door_thick, 0.030, "fridge_door_panel")
    fridge_door.visual(door_mesh, origin=Origin(xyz=(0.018 + door_width / 2, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)), material=white, name="outer_panel")
    fridge_door.visual(Box((door_width - 0.065, 0.006, fridge_h - 0.090)), origin=Origin(xyz=(0.018 + door_width / 2, -0.035, 0.0)), material=Material("soft_highlight", rgba=(1.0, 1.0, 0.96, 1.0)), name="pressed_center")
    fridge_door.visual(Box((0.050, 0.012, fridge_h - 0.120)), origin=Origin(xyz=(0.018 + door_width - 0.060, -0.039, 0.0)), material=satin, name="handle_backplate")
    fridge_door.visual(Cylinder(radius=0.018, length=0.710), origin=Origin(xyz=(0.018 + door_width - 0.070, -0.100, -0.020)), material=satin, name="handle_grip")
    for z in (-0.355, 0.315):
        fridge_door.visual(Cylinder(radius=0.012, length=0.070), origin=Origin(xyz=(0.018 + door_width - 0.070, -0.071, z), rpy=(pi / 2, 0.0, 0.0)), material=satin, name=f"handle_standoff_{0 if z < 0 else 1}")
    # Door bins and magnetic gasket on the inner face.
    for i, z in enumerate((-0.205, 0.115, 0.405)):
        fridge_door.visual(Box((0.62, 0.012, 0.075)), origin=Origin(xyz=(0.018 + door_width / 2, 0.038, z)), material=liner, name=f"door_bin_back_{i}")
        fridge_door.visual(Box((0.62, 0.016, 0.060)), origin=Origin(xyz=(0.018 + door_width / 2, 0.124, z - 0.020)), material=clear, name=f"door_bin_lip_{i}")
        fridge_door.visual(Box((0.015, 0.095, 0.080)), origin=Origin(xyz=(0.018 + door_width / 2 - 0.310, 0.080, z - 0.010)), material=clear, name=f"door_bin_side_{i}_0")
        fridge_door.visual(Box((0.015, 0.095, 0.080)), origin=Origin(xyz=(0.018 + door_width / 2 + 0.310, 0.080, z - 0.010)), material=clear, name=f"door_bin_side_{i}_1")
    fridge_door.visual(Box((door_width - 0.120, 0.010, 0.026)), origin=Origin(xyz=(0.018 + door_width / 2, 0.038, fridge_h / 2 - 0.040)), material=gasket, name="gasket_top")
    fridge_door.visual(Box((door_width - 0.120, 0.010, 0.026)), origin=Origin(xyz=(0.018 + door_width / 2, 0.038, -fridge_h / 2 + 0.040)), material=gasket, name="gasket_bottom")
    fridge_door.visual(Box((0.026, 0.010, fridge_h - 0.080)), origin=Origin(xyz=(0.018 + 0.080, 0.038, 0.0)), material=gasket, name="gasket_hinge")
    fridge_door.visual(Box((0.026, 0.010, fridge_h - 0.080)), origin=Origin(xyz=(0.018 + door_width - 0.035, 0.038, 0.0)), material=gasket, name="gasket_latch")
    model.articulation(
        "cabinet_to_fridge_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=fridge_door,
        origin=Origin(xyz=(hinge_x, door_y, fridge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=80.0, velocity=1.6),
    )

    # Upper freezer door.
    freezer_door = model.part("freezer_door")
    freezer_h = 0.505
    freezer_z = 1.535
    freezer_mesh = _rounded_panel(door_width, freezer_h, door_thick, 0.027, "freezer_door_panel")
    freezer_door.visual(freezer_mesh, origin=Origin(xyz=(0.018 + door_width / 2, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)), material=white, name="outer_panel")
    freezer_door.visual(Box((door_width - 0.070, 0.006, freezer_h - 0.075)), origin=Origin(xyz=(0.018 + door_width / 2, -0.035, 0.0)), material=Material("subtle_upper_highlight", rgba=(0.98, 0.98, 0.94, 1.0)), name="pressed_center")
    freezer_door.visual(Box((0.050, 0.012, freezer_h - 0.145)), origin=Origin(xyz=(0.018 + door_width - 0.060, -0.039, -0.010)), material=satin, name="handle_backplate")
    freezer_door.visual(Cylinder(radius=0.017, length=0.300), origin=Origin(xyz=(0.018 + door_width - 0.070, -0.100, -0.005)), material=satin, name="handle_grip")
    for z in (-0.130, 0.115):
        freezer_door.visual(Cylinder(radius=0.011, length=0.070), origin=Origin(xyz=(0.018 + door_width - 0.070, -0.071, z), rpy=(pi / 2, 0.0, 0.0)), material=satin, name=f"handle_standoff_{0 if z < 0 else 1}")
    freezer_door.visual(Box((0.58, 0.012, 0.070)), origin=Origin(xyz=(0.018 + door_width / 2, 0.038, -0.100)), material=liner, name="door_bin_back")
    freezer_door.visual(Box((0.58, 0.016, 0.055)), origin=Origin(xyz=(0.018 + door_width / 2, 0.118, -0.115)), material=clear, name="door_bin_lip")
    freezer_door.visual(Box((0.015, 0.092, 0.070)), origin=Origin(xyz=(0.018 + door_width / 2 - 0.290, 0.078, -0.110)), material=clear, name="door_bin_side_0")
    freezer_door.visual(Box((0.015, 0.092, 0.070)), origin=Origin(xyz=(0.018 + door_width / 2 + 0.290, 0.078, -0.110)), material=clear, name="door_bin_side_1")
    freezer_door.visual(Box((door_width - 0.120, 0.010, 0.024)), origin=Origin(xyz=(0.018 + door_width / 2, 0.038, freezer_h / 2 - 0.035)), material=gasket, name="gasket_top")
    freezer_door.visual(Box((door_width - 0.120, 0.010, 0.024)), origin=Origin(xyz=(0.018 + door_width / 2, 0.038, -freezer_h / 2 + 0.035)), material=gasket, name="gasket_bottom")
    freezer_door.visual(Box((0.024, 0.010, freezer_h - 0.070)), origin=Origin(xyz=(0.018 + 0.080, 0.038, 0.0)), material=gasket, name="gasket_hinge")
    freezer_door.visual(Box((0.024, 0.010, freezer_h - 0.070)), origin=Origin(xyz=(0.018 + door_width - 0.035, 0.038, 0.0)), material=gasket, name="gasket_latch")
    model.articulation(
        "cabinet_to_freezer_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(hinge_x, door_y, freezer_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=60.0, velocity=1.8),
    )

    # Sliding crisper drawers: clear bins retained by cabinet rails.
    for i, x in enumerate((-0.195, 0.195)):
        drawer = model.part(f"crisper_drawer_{i}")
        drawer.visual(Box((0.335, 0.390, 0.012)), origin=Origin(xyz=(0.0, 0.0, -0.075)), material=clear, name="bottom")
        drawer.visual(Box((0.335, 0.018, 0.155)), origin=Origin(xyz=(0.0, -0.195, 0.000)), material=clear, name="front_wall")
        drawer.visual(Box((0.335, 0.014, 0.130)), origin=Origin(xyz=(0.0, 0.195, -0.010)), material=clear, name="back_wall")
        drawer.visual(Box((0.014, 0.390, 0.145)), origin=Origin(xyz=(-0.167, 0.0, -0.005)), material=clear, name="side_wall_0")
        drawer.visual(Box((0.014, 0.390, 0.145)), origin=Origin(xyz=(0.167, 0.0, -0.005)), material=clear, name="side_wall_1")
        drawer.visual(Box((0.240, 0.020, 0.025)), origin=Origin(xyz=(0.0, -0.212, 0.045)), material=trim, name="pull_lip")
        drawer.visual(Box((0.350, 0.018, 0.018)), origin=Origin(xyz=(0.0, 0.000, 0.076)), material=clear, name="top_rim")
        model.articulation(
            f"cabinet_to_crisper_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(x, 0.055, 0.445)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.320, effort=45.0, velocity=0.35),
        )

    thermostat = model.part("thermostat_dial")
    thermostat.visual(mesh_from_geometry(KnobGeometry(0.055, 0.022, body_style="faceted", edge_radius=0.002), "thermostat_dial"), origin=Origin(rpy=(pi / 2, 0.0, 0.0)), material=trim, name="dial_cap")
    model.articulation(
        "cabinet_to_thermostat_dial",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=thermostat,
        origin=Origin(xyz=(-0.205, -0.195, 1.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.35, upper=2.35, effort=0.8, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    fridge_door = object_model.get_part("fridge_door")
    freezer_door = object_model.get_part("freezer_door")
    crisper_0 = object_model.get_part("crisper_drawer_0")
    crisper_1 = object_model.get_part("crisper_drawer_1")
    fridge_hinge = object_model.get_articulation("cabinet_to_fridge_door")
    freezer_hinge = object_model.get_articulation("cabinet_to_freezer_door")
    crisper_slide_0 = object_model.get_articulation("cabinet_to_crisper_0")
    crisper_slide_1 = object_model.get_articulation("cabinet_to_crisper_1")

    # Closed doors sit in front of the cabinet aperture without penetrating it.
    ctx.expect_gap(cabinet, fridge_door, axis="y", min_gap=0.0, max_gap=0.020, positive_elem="side_wall_0", negative_elem="outer_panel", name="fresh food door closes against front plane")
    ctx.expect_gap(cabinet, freezer_door, axis="y", min_gap=0.0, max_gap=0.020, positive_elem="side_wall_0", negative_elem="outer_panel", name="freezer door closes against front plane")
    ctx.expect_overlap(fridge_door, cabinet, axes="xz", min_overlap=0.70, elem_a="outer_panel", name="fresh food door covers cabinet opening")
    ctx.expect_overlap(freezer_door, cabinet, axes="xz", min_overlap=0.45, elem_a="outer_panel", name="freezer door covers cabinet opening")

    # Hinges open both doors outward toward the viewer rather than into the case.
    fridge_closed = ctx.part_world_aabb(fridge_door)
    freezer_closed = ctx.part_world_aabb(freezer_door)
    with ctx.pose({fridge_hinge: 1.20, freezer_hinge: 1.20}):
        fridge_open = ctx.part_world_aabb(fridge_door)
        freezer_open = ctx.part_world_aabb(freezer_door)
        ctx.check(
            "doors swing outward",
            fridge_closed is not None
            and freezer_closed is not None
            and fridge_open is not None
            and freezer_open is not None
            and fridge_open[0][1] < fridge_closed[0][1] - 0.18
            and freezer_open[0][1] < freezer_closed[0][1] - 0.18,
            details=f"fridge closed={fridge_closed}, open={fridge_open}; freezer closed={freezer_closed}, open={freezer_open}",
        )

    crisper_0_closed = ctx.part_world_position(crisper_0)
    crisper_1_closed = ctx.part_world_position(crisper_1)
    with ctx.pose({crisper_slide_0: 0.28, crisper_slide_1: 0.28, fridge_hinge: 1.45}):
        crisper_0_open = ctx.part_world_position(crisper_0)
        crisper_1_open = ctx.part_world_position(crisper_1)
        ctx.check(
            "crisper drawers slide forward",
            crisper_0_closed is not None
            and crisper_1_closed is not None
            and crisper_0_open is not None
            and crisper_1_open is not None
            and crisper_0_open[1] < crisper_0_closed[1] - 0.20
            and crisper_1_open[1] < crisper_1_closed[1] - 0.20,
            details=f"drawer0 closed={crisper_0_closed}, open={crisper_0_open}; drawer1 closed={crisper_1_closed}, open={crisper_1_open}",
        )

    return ctx.report()


object_model = build_object_model()
