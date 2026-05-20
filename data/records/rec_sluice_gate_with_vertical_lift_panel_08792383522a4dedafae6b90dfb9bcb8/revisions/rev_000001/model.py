from __future__ import annotations

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _handwheel_rim_mesh():
    points = []
    radius = 0.24
    for i in range(25):
        angle = 2.0 * math.pi * i / 24.0
        # Ring lies in the local YZ plane so the gearbox input shaft is local +X.
        points.append((0.0, radius * math.cos(angle), radius * math.sin(angle)))
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=0.018,
            samples_per_segment=6,
            radial_segments=14,
        ),
        "sluice_handwheel_rim",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="masonry_sluice_gate")

    concrete = model.material("aged_concrete", rgba=(0.55, 0.54, 0.49, 1.0))
    mortar = model.material("dark_mortar", rgba=(0.30, 0.30, 0.28, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.52, 0.55, 0.56, 1.0))
    dark_steel = model.material("dark_oiled_steel", rgba=(0.12, 0.14, 0.15, 1.0))
    gate_green = model.material("weathered_green_gate", rgba=(0.12, 0.28, 0.24, 1.0))
    rib_green = model.material("dark_green_ribs", rgba=(0.07, 0.18, 0.16, 1.0))
    gearbox_paint = model.material("blue_grey_casting", rgba=(0.18, 0.28, 0.33, 1.0))
    rust = model.material("rust_stains", rgba=(0.56, 0.26, 0.12, 1.0))

    masonry = model.part("masonry")
    masonry.visual(Box((0.80, 0.90, 3.35)), origin=Origin(xyz=(-1.50, 0.0, 1.675)), material=concrete, name="pier_0")
    masonry.visual(Box((0.80, 0.90, 3.35)), origin=Origin(xyz=(1.50, 0.0, 1.675)), material=concrete, name="pier_1")
    masonry.visual(Box((3.80, 0.90, 0.35)), origin=Origin(xyz=(0.0, 0.0, 0.175)), material=concrete, name="sill")
    masonry.visual(Box((3.80, 0.90, 0.55)), origin=Origin(xyz=(0.0, 0.0, 3.625)), material=concrete, name="lintel")
    masonry.visual(Box((3.95, 1.02, 0.12)), origin=Origin(xyz=(0.0, 0.0, -0.06)), material=concrete, name="footing")

    # Recessed masonry courses on the visible downstream face.
    for i, z in enumerate((0.70, 1.20, 1.70, 2.20, 2.70, 3.28, 3.60)):
        masonry.visual(
            Box((3.78, 0.010, 0.035)),
            origin=Origin(xyz=(0.0, -0.449, z)),
            material=mortar,
            name=f"mortar_course_{i}",
        )
    for i, x in enumerate((-1.78, -1.48, -1.18, 1.18, 1.48, 1.78)):
        masonry.visual(
            Box((0.030, 0.012, 2.85)),
            origin=Origin(xyz=(x, -0.450, 1.75)),
            material=mortar,
            name=f"mortar_joint_{i}",
        )

    guide_z = 1.825
    guide_h = 2.95
    guide_names = (
        ("guide_web_0", "guide_lip_0_front", "guide_lip_0_back"),
        ("guide_web_1", "guide_lip_1_front", "guide_lip_1_back"),
    )
    for side, sx in enumerate((-1.0, 1.0)):
        web_x = sx * 1.06
        lip_x = sx * 0.93
        web_name, front_lip_name, back_lip_name = guide_names[side]
        masonry.visual(
            Box((0.08, 0.30, guide_h)),
            origin=Origin(xyz=(web_x, 0.0, guide_z)),
            material=galvanized,
            name=web_name,
        )
        masonry.visual(
            Box((0.18, 0.050, guide_h)),
            origin=Origin(xyz=(lip_x, -0.080, guide_z)),
            material=dark_steel,
            name=front_lip_name,
        )
        masonry.visual(
            Box((0.18, 0.050, guide_h)),
            origin=Origin(xyz=(lip_x, 0.080, guide_z)),
            material=dark_steel,
            name=back_lip_name,
        )
        for bolt_i, z in enumerate((0.68, 1.20, 1.72, 2.24, 2.76)):
            masonry.visual(
                Cylinder(radius=0.036, length=0.045),
                origin=Origin(xyz=(web_x, -0.172, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
                name=f"guide_bolt_{side}_{bolt_i}",
            )
    masonry.visual(Box((2.28, 0.32, 0.08)), origin=Origin(xyz=(0.0, 0.0, 3.34)), material=galvanized, name="guide_head")
    masonry.visual(Box((2.22, 0.06, 0.08)), origin=Origin(xyz=(0.0, -0.19, 3.34)), material=dark_steel, name="front_stop_bar")
    masonry.visual(Box((2.22, 0.06, 0.08)), origin=Origin(xyz=(0.0, 0.19, 3.34)), material=dark_steel, name="back_stop_bar")

    lift_panel = model.part("lift_panel")
    lift_panel.visual(Box((1.96, 0.110, 2.50)), origin=Origin(xyz=(0.0, 0.0, 1.60)), material=gate_green, name="panel_plate")
    for i, z in enumerate((0.66, 1.18, 1.70, 2.22, 2.72)):
        lift_panel.visual(
            Box((1.62, 0.050, 0.075)),
            origin=Origin(xyz=(0.0, -0.080, z)),
            material=rib_green,
            name=f"horizontal_rib_{i}",
        )
    lift_panel.visual(Box((0.12, 0.055, 2.18)), origin=Origin(xyz=(0.0, -0.083, 1.63)), material=rib_green, name="center_rib")
    lift_panel.visual(Box((0.30, 0.115, 0.14)), origin=Origin(xyz=(0.0, -0.110, 2.88)), material=dark_steel, name="lifting_lug")
    lift_panel.visual(Cylinder(radius=0.035, length=0.32), origin=Origin(xyz=(0.0, -0.125, 3.09)), material=dark_steel, name="lifting_screw")
    lift_panel.visual(Box((1.55, 0.018, 0.040)), origin=Origin(xyz=(0.0, -0.064, 1.05)), material=rust, name="rust_streak")

    gearbox = model.part("gearbox")
    gearbox.visual(Box((0.82, 0.58, 0.55)), origin=Origin(), material=gearbox_paint, name="housing")
    gearbox.visual(Box((0.94, 0.66, 0.08)), origin=Origin(xyz=(0.0, 0.0, -0.315)), material=dark_steel, name="base_plate")
    gearbox.visual(Box((0.62, 0.46, 0.075)), origin=Origin(xyz=(0.0, 0.0, 0.3125)), material=dark_steel, name="top_cover")
    gearbox.visual(
        Cylinder(radius=0.110, length=0.150),
        origin=Origin(xyz=(-0.485, 0.0, 0.04), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_boss",
    )
    gearbox.visual(
        Cylinder(radius=0.075, length=0.11),
        origin=Origin(xyz=(0.0, -0.345, -0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="output_boss",
    )
    gearbox.visual(Box((0.10, 0.035, 0.26)), origin=Origin(xyz=(-0.245, -0.299, 0.04)), material=dark_steel, name="door_hinge_leaf")

    handwheel = model.part("handwheel")
    handwheel.visual(_handwheel_rim_mesh(), material=dark_steel, name="rim")
    handwheel.visual(
        Cylinder(radius=0.074, length=0.090),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    handwheel.visual(
        Cylinder(radius=0.035, length=0.240),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="input_shaft",
    )
    for i in range(6):
        angle = 2.0 * math.pi * i / 6.0
        start = (0.0, 0.060 * math.cos(angle), 0.060 * math.sin(angle))
        end = (0.0, 0.225 * math.cos(angle), 0.225 * math.sin(angle))
        _add_member(handwheel, start, end, 0.011, galvanized, name=f"spoke_{i}")
    handwheel.visual(Cylinder(radius=0.030, length=0.11), origin=Origin(xyz=(-0.010, 0.0, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="spinner_knob")

    inspection_door = model.part("inspection_door")
    inspection_door.visual(Box((0.290, 0.034, 0.220)), origin=Origin(xyz=(0.145, -0.017, 0.0)), material=galvanized, name="door_leaf")
    inspection_door.visual(Cylinder(radius=0.018, length=0.255), origin=Origin(xyz=(0.0, -0.045, 0.0)), material=dark_steel, name="hinge_barrel")
    inspection_door.visual(Box((0.080, 0.018, 0.030)), origin=Origin(xyz=(0.245, -0.040, 0.0)), material=dark_steel, name="pull_tab")

    model.articulation(
        "panel_lift",
        ArticulationType.PRISMATIC,
        parent=masonry,
        child=lift_panel,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.18, lower=0.0, upper=0.40),
    )
    model.articulation(
        "gearbox_mount",
        ArticulationType.FIXED,
        parent=masonry,
        child=gearbox,
        origin=Origin(xyz=(0.0, -0.08, 4.255)),
    )
    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gearbox,
        child=handwheel,
        origin=Origin(xyz=(-0.660, 0.0, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=8.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=gearbox,
        child=inspection_door,
        origin=Origin(xyz=(-0.180, -0.290, 0.04)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    masonry = object_model.get_part("masonry")
    lift_panel = object_model.get_part("lift_panel")
    gearbox = object_model.get_part("gearbox")
    handwheel = object_model.get_part("handwheel")
    inspection_door = object_model.get_part("inspection_door")
    panel_lift = object_model.get_articulation("panel_lift")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        gearbox,
        handwheel,
        elem_a="bearing_boss",
        elem_b="input_shaft",
        reason="The handwheel input shaft is intentionally seated inside the gearbox bearing boss.",
    )
    ctx.expect_within(
        handwheel,
        gearbox,
        axes="yz",
        inner_elem="input_shaft",
        outer_elem="bearing_boss",
        margin=0.002,
        name="input shaft is centered in bearing boss",
    )
    ctx.expect_overlap(
        handwheel,
        gearbox,
        axes="x",
        elem_a="input_shaft",
        elem_b="bearing_boss",
        min_overlap=0.10,
        name="input shaft remains inserted in bearing boss",
    )

    ctx.expect_contact(gearbox, masonry, elem_a="base_plate", elem_b="lintel", contact_tol=1e-4, name="gearbox sits on lintel")
    ctx.expect_contact(lift_panel, masonry, elem_a="panel_plate", elem_b="sill", contact_tol=1e-4, name="closed panel seats on sill")
    ctx.expect_gap(
        lift_panel,
        masonry,
        axis="y",
        positive_elem="panel_plate",
        negative_elem="guide_lip_0_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="panel bears against front guide lip",
    )
    ctx.expect_gap(
        masonry,
        lift_panel,
        axis="y",
        positive_elem="guide_lip_0_back",
        negative_elem="panel_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="panel bears against back guide lip",
    )
    ctx.expect_overlap(lift_panel, masonry, axes="xz", elem_a="panel_plate", elem_b="guide_lip_0_front", min_overlap=0.12, name="panel edge remains under front lip")
    ctx.expect_overlap(lift_panel, masonry, axes="xz", elem_a="panel_plate", elem_b="guide_lip_1_back", min_overlap=0.12, name="opposite panel edge remains under back lip")

    rest_pos = ctx.part_world_position(lift_panel)
    with ctx.pose({panel_lift: 0.40}):
        ctx.expect_overlap(lift_panel, masonry, axes="xz", elem_a="panel_plate", elem_b="guide_lip_0_front", min_overlap=0.12, name="raised panel is still captured by guide")
        ctx.expect_overlap(lift_panel, masonry, axes="xz", elem_a="panel_plate", elem_b="guide_lip_1_back", min_overlap=0.12, name="raised opposite edge is still captured")
        raised_pos = ctx.part_world_position(lift_panel)
    ctx.check(
        "panel lift moves upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.35,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    ctx.expect_contact(inspection_door, gearbox, elem_a="door_leaf", elem_b="housing", contact_tol=1e-4, name="inspection door closes against housing")
    closed_aabb = ctx.part_element_world_aabb(inspection_door, elem="door_leaf")
    with ctx.pose({door_hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(inspection_door, elem="door_leaf")
    ctx.check(
        "inspection door swings outward",
        closed_aabb is not None and open_aabb is not None and open_aabb[0][1] < closed_aabb[0][1] - 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
