from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


DECK_TOP_Z = 0.9825
OPENING_Y = -0.08


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _rect_frame_shape(
    outer_size: tuple[float, float, float],
    inner_size: tuple[float, float],
    *,
    center: tuple[float, float, float],
) -> cq.Workplane:
    sx, sy, sz = outer_size
    ix, iy = inner_size
    outer = cq.Workplane("XY").box(sx, sy, sz)
    cutter = cq.Workplane("XY").box(ix, iy, sz * 3.0)
    return outer.cut(cutter).translate(center)


def _build_cabinet_shell() -> cq.Workplane:
    cab_w = 0.74
    cab_d = 0.72
    cab_h = 0.92
    wall_t = 0.035
    bottom_t = 0.070
    deck_w = 0.78
    deck_d = 0.76
    deck_t = 0.075

    left = _box_shape((wall_t, cab_d, cab_h), (-(cab_w - wall_t) / 2.0, 0.0, cab_h / 2.0))
    right = _box_shape((wall_t, cab_d, cab_h), ((cab_w - wall_t) / 2.0, 0.0, cab_h / 2.0))
    front = _box_shape((cab_w, wall_t, cab_h), (0.0, -(cab_d - wall_t) / 2.0, cab_h / 2.0))
    rear = _box_shape((cab_w, wall_t, cab_h), (0.0, (cab_d - wall_t) / 2.0, cab_h / 2.0))
    bottom = _box_shape((cab_w, cab_d, bottom_t), (0.0, 0.0, bottom_t / 2.0))

    deck = _box_shape((deck_w, deck_d, deck_t), (0.0, 0.0, DECK_TOP_Z - deck_t / 2.0))
    opening_cut = (
        cq.Workplane("XY")
        .circle(0.292)
        .extrude(deck_t * 3.0)
        .translate((0.0, OPENING_Y, DECK_TOP_Z - deck_t * 1.5))
    )
    deck = deck.cut(opening_cut)

    return left.union(right).union(front).union(rear).union(bottom).union(deck)


def _build_opening_rim() -> cq.Workplane:
    # A low raised porcelain lip around the cutout; it is a real ring, not a cap.
    return (
        cq.Workplane("XY")
        .circle(0.322)
        .circle(0.292)
        .extrude(0.020)
        .translate((0.0, OPENING_Y, DECK_TOP_Z - 0.010))
    )


def _build_deep_well() -> cq.Workplane:
    # Static dark outer tub / well walls extend well below the deck so the opening
    # reads as a deep hollow throat instead of a shallow depression.
    return (
        cq.Workplane("XY")
        .circle(0.292)
        .circle(0.252)
        .extrude(0.635)
        .translate((0.0, OPENING_Y, 0.285))
    )


def _build_tub_basket() -> cq.Workplane:
    wall = cq.Workplane("XY").circle(0.235).circle(0.213).extrude(0.545)
    bottom = cq.Workplane("XY").circle(0.235).extrude(0.030)
    rim = cq.Workplane("XY").circle(0.242).circle(0.203).extrude(0.035).translate((0.0, 0.0, 0.510))
    agitator = cq.Workplane("XY").circle(0.046).extrude(0.365).translate((0.0, 0.0, 0.030))
    vane_x = _box_shape((0.265, 0.030, 0.055), (0.0, 0.0, 0.080))
    vane_y = _box_shape((0.030, 0.265, 0.055), (0.0, 0.0, 0.080))
    return wall.union(bottom).union(rim).union(agitator).union(vane_x).union(vane_y)


def _build_lid_frame() -> cq.Workplane:
    # Child frame origin is on the rear hinge line.  The panel extends along -Y.
    return _rect_frame_shape((0.650, 0.560, 0.026), (0.500, 0.405), center=(0.0, -0.280, 0.018))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_top_load_washer")

    porcelain = model.material("porcelain_white", rgba=(0.92, 0.94, 0.93, 1.0))
    soft_gray = model.material("soft_gray_plastic", rgba=(0.58, 0.61, 0.62, 1.0))
    charcoal = model.material("charcoal_shadow", rgba=(0.025, 0.028, 0.030, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    black = model.material("black_gloss", rgba=(0.01, 0.012, 0.014, 1.0))
    glass = model.material("smoked_glass", rgba=(0.13, 0.22, 0.30, 0.42))
    blue = model.material("blue_start_button", rgba=(0.06, 0.38, 0.86, 1.0))

    body = model.part("cabinet")
    body.visual(
        mesh_from_cadquery(_build_cabinet_shell(), "cabinet_shell", tolerance=0.002),
        material=porcelain,
        name="cabinet_shell",
    )
    body.visual(
        mesh_from_cadquery(_build_opening_rim(), "opening_rim", tolerance=0.0015),
        material=porcelain,
        name="opening_rim",
    )
    body.visual(
        mesh_from_cadquery(_build_deep_well(), "deep_well", tolerance=0.002),
        material=charcoal,
        name="deep_well",
    )
    body.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .circle(0.055)
            .circle(0.030)
            .extrude(0.178)
            .translate((0.0, OPENING_Y, 0.070)),
            "bearing_pedestal",
            tolerance=0.0015,
        ),
        material=charcoal,
        name="bearing_pedestal",
    )
    body.visual(
        Cylinder(radius=0.033, length=0.070),
        origin=Origin(xyz=(0.0, OPENING_Y, 0.215)),
        material=black,
        name="bearing_bushing",
    )
    body.visual(
        Box((0.70, 0.030, 0.145)),
        origin=Origin(xyz=(0.0, 0.386, 1.052)),
        material=soft_gray,
        name="rear_console",
    )
    body.visual(
        Box((0.52, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, 0.367, 1.060)),
        material=black,
        name="control_strip",
    )
    body.visual(
        Box((0.055, 0.018, 0.008)),
        origin=Origin(xyz=(-0.275, 0.198, DECK_TOP_Z + 0.004)),
        material=soft_gray,
        name="hinge_boss_0",
    )
    body.visual(
        Box((0.055, 0.018, 0.008)),
        origin=Origin(xyz=(0.275, 0.198, DECK_TOP_Z + 0.004)),
        material=soft_gray,
        name="hinge_boss_1",
    )

    tub = model.part("wash_tub")
    tub.visual(
        mesh_from_cadquery(_build_tub_basket(), "wash_tub_basket", tolerance=0.0015),
        material=stainless,
        name="basket",
    )
    tub.visual(
        Cylinder(radius=0.023, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.0425)),
        material=stainless,
        name="drive_shaft",
    )

    main_lid = model.part("glass_lid")
    main_lid.visual(
        mesh_from_cadquery(_build_lid_frame(), "glass_lid_frame", tolerance=0.0015),
        material=soft_gray,
        name="lid_frame",
    )
    main_lid.visual(
        Box((0.535, 0.440, 0.010)),
        origin=Origin(xyz=(0.0, -0.280, 0.018)),
        material=glass,
        name="glass_panel",
    )
    main_lid.visual(
        Cylinder(radius=0.017, length=0.570),
        origin=Origin(xyz=(0.0, 0.004, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="hinge_barrel",
    )

    for idx, x in enumerate((-0.245, 0.245)):
        disp = model.part(f"dispenser_{idx}")
        disp.visual(
            Box((0.185, 0.112, 0.016)),
            # The child frame is on the rear hinge line; the lid leaf extends forward.
            origin=Origin(xyz=(0.0, -0.056, 0.010)),
            material=soft_gray,
            name="lid_panel",
        )
        disp.visual(
            Cylinder(radius=0.0065, length=0.145),
            origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=soft_gray,
            name="hinge_pin",
        )
        body.visual(
            Box((0.213, 0.010, 0.006)),
            origin=Origin(xyz=(x, 0.227, DECK_TOP_Z + 0.003)),
            material=black,
            name=f"dispenser_pocket_front_{idx}",
        )
        body.visual(
            Box((0.010, 0.130, 0.006)),
            origin=Origin(xyz=(x - 0.106, 0.289, DECK_TOP_Z + 0.003)),
            material=black,
            name=f"dispenser_pocket_side_{idx}_0",
        )
        body.visual(
            Box((0.010, 0.130, 0.006)),
            origin=Origin(xyz=(x + 0.106, 0.289, DECK_TOP_Z + 0.003)),
            material=black,
            name=f"dispenser_pocket_side_{idx}_1",
        )

    selector = model.part("selector_knob")
    selector_knob = KnobGeometry(
        0.082,
        0.045,
        body_style="skirted",
        top_diameter=0.062,
        edge_radius=0.002,
        skirt=KnobSkirt(0.098, 0.010, flare=0.04, chamfer=0.002),
        grip=KnobGrip(style="fluted", count=28, depth=0.0015),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    selector.visual(
        mesh_from_geometry(selector_knob, "selector_knob"),
        material=soft_gray,
        name="knob_cap",
    )

    start = model.part("start_button")
    start.visual(
        Cylinder(radius=0.027, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=blue,
        name="button_cap",
    )

    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_Y, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=7.0),
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=main_lid,
        origin=Origin(xyz=(0.0, 0.183, DECK_TOP_Z + 0.005)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    for idx, x in enumerate((-0.245, 0.245)):
        model.articulation(
            f"cabinet_to_dispenser_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=f"dispenser_{idx}",
            origin=Origin(xyz=(x, 0.344, DECK_TOP_Z + 0.004)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=1.25),
        )
    model.articulation(
        "cabinet_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(-0.085, 0.270, DECK_TOP_Z + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )
    model.articulation(
        "cabinet_to_start",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start,
        origin=Origin(xyz=(0.095, 0.270, DECK_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.15, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    tub = object_model.get_part("wash_tub")
    lid = object_model.get_part("glass_lid")
    disp_0 = object_model.get_part("dispenser_0")
    disp_1 = object_model.get_part("dispenser_1")
    start = object_model.get_part("start_button")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    disp_hinge_0 = object_model.get_articulation("cabinet_to_dispenser_0")
    disp_hinge_1 = object_model.get_articulation("cabinet_to_dispenser_1")
    start_push = object_model.get_articulation("cabinet_to_start")

    ctx.allow_overlap(
        cabinet,
        tub,
        elem_a="bearing_bushing",
        elem_b="drive_shaft",
        reason="The rotating basket's drive shaft is intentionally captured inside the fixed center bushing.",
    )
    ctx.expect_within(
        tub,
        cabinet,
        axes="xy",
        inner_elem="drive_shaft",
        outer_elem="bearing_bushing",
        margin=0.0,
        name="drive shaft stays centered in the bushing",
    )
    ctx.expect_overlap(
        tub,
        cabinet,
        axes="z",
        elem_a="drive_shaft",
        elem_b="bearing_bushing",
        min_overlap=0.060,
        name="drive shaft remains captured by the bushing",
    )

    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="lid_frame",
        negative_elem="opening_rim",
        name="closed glass lid sits just above the deck rim",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="xy",
        min_overlap=0.40,
        elem_a="glass_panel",
        elem_b="opening_rim",
        name="glass lid spans the washer opening",
    )
    ctx.expect_within(
        tub,
        cabinet,
        axes="xy",
        margin=0.010,
        inner_elem="basket",
        outer_elem="deep_well",
        name="rotating basket is centered inside the deep well",
    )

    cab_aabb = ctx.part_element_world_aabb(cabinet, elem="opening_rim")
    tub_aabb = ctx.part_element_world_aabb(tub, elem="basket")
    ctx.check(
        "drum opening remains visibly deep",
        cab_aabb is not None and tub_aabb is not None and (cab_aabb[1][2] - tub_aabb[1][2]) > 0.16,
        details=f"rim={cab_aabb}, tub={tub_aabb}",
    )

    for disp in (disp_0, disp_1):
        ctx.expect_gap(
            disp,
            cabinet,
            axis="z",
            min_gap=0.001,
            max_gap=0.020,
            positive_elem="lid_panel",
            negative_elem="cabinet_shell",
            name=f"{disp.name} lies flush above its rear deck pocket",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "main lid rotates upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.23,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_disp_aabb = ctx.part_world_aabb(disp_0)
    with ctx.pose({disp_hinge_0: 0.9, disp_hinge_1: 0.9}):
        open_disp_aabb = ctx.part_world_aabb(disp_0)
    ctx.check(
        "dispenser lids flip upward from rear-corner hinges",
        closed_disp_aabb is not None
        and open_disp_aabb is not None
        and open_disp_aabb[1][2] > closed_disp_aabb[1][2] + 0.05,
        details=f"closed={closed_disp_aabb}, open={open_disp_aabb}",
    )

    rest_button = ctx.part_world_position(start)
    with ctx.pose({start_push: 0.012}):
        pushed_button = ctx.part_world_position(start)
    ctx.check(
        "start button pushes downward",
        rest_button is not None and pushed_button is not None and pushed_button[2] < rest_button[2] - 0.010,
        details=f"rest={rest_button}, pushed={pushed_button}",
    )

    return ctx.report()


object_model = build_object_model()
