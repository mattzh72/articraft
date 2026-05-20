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
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


FASCIA_W = 0.72
FASCIA_H = 0.38
FASCIA_T = 0.036
FASCIA_Z = -0.060

OPENING_W = 0.500
OPENING_H = 0.260
OPENING_TOP_Z = 0.070
OPENING_Z = OPENING_TOP_Z - OPENING_H / 2.0

DOOR_W = 0.486
DOOR_H = 0.248
DOOR_T = 0.018
HINGE_Y = -0.010
HINGE_Z = OPENING_TOP_Z

STAY_X = 0.228
STAY_PIVOT_Y = HINGE_Y + 0.030
STAY_PIVOT_Z = HINGE_Z - 0.018
STAY_LEN = 0.130
STAY_DOOR_Z = STAY_PIVOT_Z - HINGE_Z - STAY_LEN


def _fascia_frame_shape() -> cq.Workplane:
    """Dashboard fascia with a glove-box opening cut through it."""
    frame = (
        cq.Workplane("XY")
        .box(FASCIA_W, FASCIA_T, FASCIA_H)
        .translate((0.0, 0.0, FASCIA_Z))
    )
    opening_cutter = (
        cq.Workplane("XY")
        .box(OPENING_W, FASCIA_T + 0.012, OPENING_H)
        .translate((0.0, 0.0, OPENING_Z))
    )
    return frame.cut(opening_cutter)


def _storage_tub_shape() -> cq.Workplane:
    """Thin-walled fixed storage tub, open toward the flush door."""
    wall = 0.018
    tub_depth = 0.230
    y0 = FASCIA_T / 2.0 - 0.006
    y1 = y0 + tub_depth
    yc = (y0 + y1) / 2.0
    zc = OPENING_Z

    back = (
        cq.Workplane("XY")
        .box(OPENING_W, wall, OPENING_H)
        .translate((0.0, y1, zc))
    )
    top = (
        cq.Workplane("XY")
        .box(OPENING_W + 2.0 * wall, tub_depth, wall)
        .translate((0.0, yc, OPENING_TOP_Z + wall / 2.0))
    )
    bottom = (
        cq.Workplane("XY")
        .box(OPENING_W + 2.0 * wall, tub_depth, wall)
        .translate((0.0, yc, OPENING_TOP_Z - OPENING_H - wall / 2.0))
    )
    side_a = (
        cq.Workplane("XY")
        .box(wall, tub_depth, OPENING_H)
        .translate((OPENING_W / 2.0 + wall / 2.0, yc, zc))
    )
    side_b = (
        cq.Workplane("XY")
        .box(wall, tub_depth, OPENING_H)
        .translate((-OPENING_W / 2.0 - wall / 2.0, yc, zc))
    )
    return back.union(top).union(bottom).union(side_a).union(side_b)


def _door_panel_shape() -> cq.Workplane:
    """A slightly rounded molded plastic door, expressed in the door frame."""
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((0.0, 0.0, -DOOR_H / 2.0))
    )
    try:
        panel = panel.edges().fillet(0.004)
    except Exception:
        # Small cosmetic fillets are not worth making the authored mechanism
        # brittle if a CadQuery kernel build rejects one edge.
        pass
    return panel


def _make_stay(part, material: Material) -> None:
    part.visual(
        Box((0.012, 0.005, STAY_LEN)),
        origin=Origin(xyz=(0.0, 0.0, -STAY_LEN / 2.0)),
        material=material,
        name="stay_bar",
    )
    part.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="pivot_bushing",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -STAY_LEN), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="end_bushing",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upper_hinged_dashboard_glove_box")

    fascia_mat = model.material("grained_charcoal_fascia", rgba=(0.10, 0.105, 0.11, 1.0))
    tub_mat = model.material("matte_black_tub", rgba=(0.015, 0.016, 0.018, 1.0))
    door_mat = model.material("slightly_lighter_door", rgba=(0.13, 0.135, 0.14, 1.0))
    reveal_mat = model.material("black_shadow_reveals", rgba=(0.004, 0.004, 0.005, 1.0))
    metal_mat = model.material("dark_phosphate_metal", rgba=(0.055, 0.055, 0.052, 1.0))
    knob_mat = model.material("satin_latch_knob", rgba=(0.025, 0.026, 0.028, 1.0))

    fascia = model.part("fascia_tub")
    rail_overlap = 0.003
    outer_top = FASCIA_Z + FASCIA_H / 2.0
    outer_bottom = FASCIA_Z - FASCIA_H / 2.0
    upper_rail_bottom = HINGE_Z + 0.012
    lower_rail_top = OPENING_TOP_Z - OPENING_H - 0.004
    side_w = (FASCIA_W - OPENING_W) / 2.0
    fascia.visual(
        Box((FASCIA_W, FASCIA_T, outer_top - upper_rail_bottom + rail_overlap)),
        origin=Origin(xyz=(0.0, 0.0, (outer_top + upper_rail_bottom) / 2.0)),
        material=fascia_mat,
        name="upper_rail",
    )
    fascia.visual(
        Box((FASCIA_W, FASCIA_T, lower_rail_top - outer_bottom + rail_overlap)),
        origin=Origin(xyz=(0.0, 0.0, (lower_rail_top + outer_bottom) / 2.0)),
        material=fascia_mat,
        name="lower_rail",
    )
    for idx, sx in enumerate((-1.0, 1.0)):
        fascia.visual(
            Box((side_w + rail_overlap, FASCIA_T, OPENING_H + 2.0 * rail_overlap)),
            origin=Origin(xyz=(sx * (OPENING_W / 2.0 + side_w / 2.0), 0.0, OPENING_Z)),
            material=fascia_mat,
            name=f"side_rail_{idx}",
        )
    fascia.visual(
        Box((OPENING_W + 0.024, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -FASCIA_T / 2.0 - 0.0045, HINGE_Z + 0.015)),
        material=reveal_mat,
        name="top_shadow_lip",
    )
    fascia.visual(
        mesh_from_cadquery(_storage_tub_shape(), "storage_tub", tolerance=0.001),
        material=tub_mat,
        name="storage_tub",
    )
    # Metal hinge pin and compact side-stay pins are fixed to the fascia/tub
    # assembly.  The moving child bushings are intentionally captured around
    # these pin proxies.
    fascia.visual(
        Cylinder(radius=0.0045, length=0.560),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="upper_pin",
    )
    for idx, sx in enumerate((-STAY_X, STAY_X)):
        fascia.visual(
            Cylinder(radius=0.0040, length=0.030),
            origin=Origin(
                xyz=(sx, STAY_PIVOT_Y, STAY_PIVOT_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=metal_mat,
            name=f"stay_pin_{idx}",
        )
        fascia.visual(
            Box((0.050, 0.034, 0.052)),
            origin=Origin(xyz=(sx, STAY_PIVOT_Y + 0.032, STAY_PIVOT_Z + 0.002)),
            material=tub_mat,
            name=f"stay_mount_{idx}",
        )
        for cheek in (-1.0, 1.0):
            fascia.visual(
                Box((0.006, 0.018, 0.026)),
                origin=Origin(
                    xyz=(sx + cheek * 0.017, STAY_PIVOT_Y + 0.010, STAY_PIVOT_Z)
                ),
                material=tub_mat,
                name=f"stay_cheek_{idx}_{0 if cheek < 0.0 else 1}",
            )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_panel_shape(), "door_panel", tolerance=0.0008),
        material=door_mat,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=door_mat,
        name="hinge_clip",
    )
    # Thin black molded reveal strips make the closed panel read as a flush
    # glove-box door with a small uniform seam.
    door.visual(
        Box((DOOR_W - 0.050, 0.0015, 0.004)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0 - 0.0004, -0.028)),
        material=reveal_mat,
        name="upper_reveal",
    )
    door.visual(
        Box((DOOR_W - 0.070, 0.0015, 0.004)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0 - 0.0004, -DOOR_H + 0.045)),
        material=reveal_mat,
        name="lower_reveal",
    )
    for idx, sx in enumerate((-STAY_X, STAY_X)):
        door.visual(
            Box((0.022, 0.040, 0.040)),
            origin=Origin(xyz=(sx, 0.026, STAY_DOOR_Z)),
            material=door_mat,
            name=f"side_slot_{idx}",
        )
        door.visual(
            Box((0.016, 0.003, 0.052)),
            origin=Origin(xyz=(sx, 0.047, STAY_DOOR_Z)),
            material=reveal_mat,
            name=f"slot_shadow_{idx}",
        )

    knob = model.part("latch_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.016,
                body_style="faceted",
                top_diameter=0.038,
                edge_radius=0.0012,
                grip=KnobGrip(style="ribbed", count=14, depth=0.0007),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "latch_knob_cap",
        ),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_cap",
    )
    knob.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="knob_shaft",
    )
    knob.visual(
        Box((0.030, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, -0.027, 0.0)),
        material=fascia_mat,
        name="pointer_bar",
    )

    stay_0 = model.part("stay_0")
    _make_stay(stay_0, metal_mat)
    stay_1 = model.part("stay_1")
    _make_stay(stay_1, metal_mat)

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=fascia,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.12),
        motion_properties=MotionProperties(damping=0.15, friction=0.35),
    )
    model.articulation(
        "latch_axis",
        ArticulationType.REVOLUTE,
        parent=door,
        child=knob,
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0, -DOOR_H + 0.046)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-0.80, upper=0.80),
        motion_properties=MotionProperties(damping=0.03, friction=0.08),
    )
    for idx, (stay, sx) in enumerate(((stay_0, -STAY_X), (stay_1, STAY_X))):
        model.articulation(
            f"stay_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=fascia,
            child=stay,
            origin=Origin(xyz=(sx, STAY_PIVOT_Y, STAY_PIVOT_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.6, lower=0.0, upper=1.12),
            motion_properties=MotionProperties(damping=0.30, friction=0.90),
            mimic=Mimic(joint="door_hinge", multiplier=1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fascia = object_model.get_part("fascia_tub")
    door = object_model.get_part("door")
    knob = object_model.get_part("latch_knob")
    stay_0 = object_model.get_part("stay_0")
    stay_1 = object_model.get_part("stay_1")
    hinge = object_model.get_articulation("door_hinge")
    latch = object_model.get_articulation("latch_axis")

    ctx.allow_overlap(
        fascia,
        door,
        elem_a="upper_pin",
        elem_b="hinge_clip",
        reason="The fixed hinge pin is intentionally captured inside the molded upper door clip.",
    )
    for idx, stay in enumerate((stay_0, stay_1)):
        ctx.allow_overlap(
            fascia,
            stay,
            elem_a=f"stay_pin_{idx}",
            elem_b="pivot_bushing",
            reason="Each friction stay pivots around a fixed support pin modeled as a captured shaft.",
        )
        ctx.allow_overlap(
            door,
            stay,
            elem_a=f"side_slot_{idx}",
            elem_b="end_bushing",
            reason="The stay end bushing is intentionally captured in the door-side friction-stay slot.",
        )
    ctx.allow_overlap(
        door,
        knob,
        elem_a="door_panel",
        elem_b="knob_shaft",
        reason="The rotary latch shaft intentionally passes through the door panel.",
    )

    with ctx.pose({hinge: 0.0, latch: 0.0}):
        ctx.expect_overlap(
            fascia,
            door,
            axes="xyz",
            elem_a="upper_pin",
            elem_b="hinge_clip",
            min_overlap=0.002,
            name="upper hinge pin is captured by the door clip",
        )
        for idx, stay in enumerate((stay_0, stay_1)):
            ctx.expect_overlap(
                fascia,
                stay,
                axes="xyz",
                elem_a=f"stay_pin_{idx}",
                elem_b="pivot_bushing",
                min_overlap=0.002,
                name=f"stay {idx} support pin is captured",
            )
            ctx.expect_overlap(
                door,
                stay,
                axes="xyz",
                elem_a=f"side_slot_{idx}",
                elem_b="end_bushing",
                min_overlap=0.002,
                name=f"stay {idx} end bushing is in the door slot",
            )
        ctx.expect_overlap(
            door,
            knob,
            axes="xyz",
            elem_a="door_panel",
            elem_b="knob_shaft",
            min_overlap=0.001,
            name="rotary latch shaft passes through the door",
        )

        door_box = ctx.part_element_world_aabb(door, elem="door_panel")
        fascia_box = ctx.part_element_world_aabb(fascia, elem="upper_rail")
        if door_box is not None and fascia_box is not None:
            door_front_y = door_box[0][1]
            fascia_front_y = fascia_box[0][1]
            ctx.check(
                "door face sits flush with fascia face",
                abs(door_front_y - fascia_front_y) <= 0.003,
                details=f"door_front_y={door_front_y}, fascia_front_y={fascia_front_y}",
            )
            ctx.check(
                "door panel fits inside the dashboard opening",
                door_box[0][0] > -OPENING_W / 2.0
                and door_box[1][0] < OPENING_W / 2.0
                and door_box[0][2] > OPENING_TOP_Z - OPENING_H
                and door_box[1][2] <= OPENING_TOP_Z + 0.012,
                details=f"door_aabb={door_box}",
            )

        closed_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        closed_stay = ctx.part_element_world_aabb(stay_0, elem="end_bushing")

    with ctx.pose({hinge: 1.05, latch: 0.0}):
        open_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        open_stay = ctx.part_element_world_aabb(stay_0, elem="end_bushing")
        if closed_panel is not None and open_panel is not None:
            closed_center_y = (closed_panel[0][1] + closed_panel[1][1]) / 2.0
            open_center_y = (open_panel[0][1] + open_panel[1][1]) / 2.0
            ctx.check(
                "upper-hinged door swings outward and upward",
                open_center_y < closed_center_y - 0.050
                and open_panel[0][2] > closed_panel[0][2] + 0.045,
                details=f"closed={closed_panel}, open={open_panel}",
            )
        if closed_stay is not None and open_stay is not None:
            closed_stay_y = (closed_stay[0][1] + closed_stay[1][1]) / 2.0
            open_stay_y = (open_stay[0][1] + open_stay[1][1]) / 2.0
            ctx.check(
                "side friction stays rotate outward with the door",
                open_stay_y < closed_stay_y - 0.050,
                details=f"closed_stay={closed_stay}, open_stay={open_stay}",
            )

    with ctx.pose({latch: 0.0}):
        pointer_0 = ctx.part_element_world_aabb(knob, elem="pointer_bar")
    with ctx.pose({latch: 0.75}):
        pointer_1 = ctx.part_element_world_aabb(knob, elem="pointer_bar")
    if pointer_0 is not None and pointer_1 is not None:
        z_span_0 = pointer_0[1][2] - pointer_0[0][2]
        z_span_1 = pointer_1[1][2] - pointer_1[0][2]
        ctx.check(
            "latch knob rotates on its own axis",
            z_span_1 > z_span_0 + 0.010,
            details=f"pointer_closed={pointer_0}, pointer_rotated={pointer_1}",
        )

    return ctx.report()


object_model = build_object_model()
