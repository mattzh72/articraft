from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small CadQuery rounded rectangular appliance panel."""
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _ridged_plate(
    length_x: float,
    width_y: float,
    thickness: float,
    *,
    ridge_count: int = 7,
    ridge_width: float = 0.022,
    ridge_height: float = 0.012,
) -> cq.Workplane:
    """Cast grill plate with raised parallel ribs in one connected mesh."""
    plate = _rounded_box((length_x, width_y, thickness), 0.012)
    usable_width = width_y * 0.76
    start = -usable_width / 2.0
    step = usable_width / float(ridge_count - 1)
    for i in range(ridge_count):
        y = start + i * step
        ridge = (
            cq.Workplane("XY")
            .box(length_x * 0.86, ridge_width, ridge_height)
            .translate((0.0, y, thickness / 2.0 + ridge_height / 2.0))
        )
        plate = plate.union(ridge)
    return plate


def _grease_drawer_pan() -> cq.Workplane:
    """Thin, shallow pull-out grease tray; local origin is the front slot plane."""
    bottom = cq.Workplane("XY").box(0.320, 0.400, 0.006).translate((-0.140, 0.0, -0.019))
    side_a = cq.Workplane("XY").box(0.320, 0.012, 0.038).translate((-0.140, 0.206, -0.004))
    side_b = cq.Workplane("XY").box(0.320, 0.012, 0.038).translate((-0.140, -0.206, -0.004))
    back = cq.Workplane("XY").box(0.012, 0.412, 0.038).translate((-0.296, 0.0, -0.004))
    front = cq.Workplane("XY").box(0.014, 0.445, 0.044).translate((0.017, 0.0, -0.004))
    return bottom.union(side_a).union(side_b).union(back).union(front)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deli_counter_grill_press")

    stainless = model.material("brushed_stainless", rgba=(0.62, 0.62, 0.58, 1.0))
    dark_iron = model.material("seasoned_cast_iron", rgba=(0.045, 0.047, 0.043, 1.0))
    black = model.material("black_insulated_polymer", rgba=(0.015, 0.014, 0.012, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    warm_tick = model.material("heat_mark_orange", rgba=(1.0, 0.33, 0.04, 1.0))
    label_white = model.material("etched_white", rgba=(0.95, 0.92, 0.82, 1.0))

    # Static countertop base: a stainless service-grade body with a real front
    # drawer slot, heavy lower cooking plate, rear tower, and side controls.
    base = model.part("base")
    base.visual(Box((0.540, 0.620, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=stainless, name="bottom_plinth")
    base.visual(Box((0.540, 0.030, 0.100)), origin=Origin(xyz=(0.0, 0.295, 0.070)), material=stainless, name="side_wall_left")
    base.visual(Box((0.540, 0.030, 0.100)), origin=Origin(xyz=(0.0, -0.295, 0.070)), material=stainless, name="side_wall_right")
    base.visual(Box((0.030, 0.620, 0.100)), origin=Origin(xyz=(-0.255, 0.0, 0.070)), material=stainless, name="rear_wall")
    base.visual(Box((0.030, 0.620, 0.044)), origin=Origin(xyz=(0.255, 0.0, 0.103)), material=stainless, name="front_fascia")
    base.visual(Box((0.540, 0.620, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.130)), material=stainless, name="top_deck")

    lower_plate = _ridged_plate(0.420, 0.520, 0.035)
    base.visual(
        mesh_from_cadquery(lower_plate, "lower_grill_plate"),
        origin=Origin(xyz=(0.020, 0.0, 0.1575)),
        material=dark_iron,
        name="lower_grill_plate",
    )

    for index, (x, y) in enumerate(((-0.190, -0.235), (-0.190, 0.235), (0.190, -0.235), (0.190, 0.235))):
        base.visual(
            Cylinder(radius=0.032, length=0.030),
            origin=Origin(xyz=(x, y, -0.012), rpy=(0.0, 0.0, 0.0)),
            material=rubber,
            name=f"foot_{index}",
        )

    # Tall rear hinge tower and a visible stout horizontal axle.
    base.visual(Box((0.080, 0.060, 0.310)), origin=Origin(xyz=(-0.230, 0.340, 0.275)), material=stainless, name="tower_post_0")
    base.visual(Box((0.080, 0.060, 0.310)), origin=Origin(xyz=(-0.230, -0.340, 0.275)), material=stainless, name="tower_post_1")
    base.visual(Box((0.055, 0.460, 0.165)), origin=Origin(xyz=(-0.263, 0.0, 0.220)), material=stainless, name="tower_web")
    base.visual(
        Cylinder(radius=0.020, length=0.740),
        origin=Origin(xyz=(-0.230, 0.0, 0.360), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_pin",
    )
    base.visual(Box((0.345, 0.030, 0.018)), origin=Origin(xyz=(0.0725, 0.150, 0.029)), material=stainless, name="drawer_rail_0")
    base.visual(Box((0.345, 0.030, 0.018)), origin=Origin(xyz=(0.0725, -0.150, 0.029)), material=stainless, name="drawer_rail_1")

    # Left-side heat control surround and fixed scale ticks.
    base.visual(Box((0.170, 0.006, 0.105)), origin=Origin(xyz=(0.080, 0.313, 0.085)), material=black, name="control_panel")
    base.visual(
        Cylinder(radius=0.052, length=0.007),
        origin=Origin(xyz=(0.080, 0.3195, 0.085), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="knob_bezel",
    )
    for index, (dx, dz, height) in enumerate(((-0.050, -0.026, 0.014), (-0.028, 0.025, 0.017), (0.000, 0.043, 0.022), (0.032, 0.025, 0.017), (0.052, -0.026, 0.014))):
        base.visual(
            Box((0.004, 0.001, height)),
            origin=Origin(xyz=(0.080 + dx, 0.31635, 0.085 + dz)),
            material=label_white if index != 4 else warm_tick,
            name=f"heat_tick_{index}",
        )

    # Hinged upper lid.  Its part frame is exactly the rear horizontal hinge
    # axis, so the revolute joint rotates the whole press head realistically.
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rounded_box((0.440, 0.580, 0.075), 0.018), "lid_shell"),
        origin=Origin(xyz=(0.230, 0.0, -0.073)),
        material=stainless,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_ridged_plate(0.390, 0.500, 0.035), "upper_grill_plate"),
        origin=Origin(xyz=(0.235, 0.0, -0.128), rpy=(math.pi, 0.0, 0.0)),
        material=dark_iron,
        name="upper_grill_plate",
    )
    for index, y in enumerate((-0.180, 0.180)):
        lid.visual(
            Cylinder(radius=0.035, length=0.130),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"hinge_barrel_{index}",
        )
        lid.visual(
            Box((0.070, 0.044, 0.090)),
            origin=Origin(xyz=(0.060, y, -0.040)),
            material=stainless,
            name=f"hinge_lug_{index}",
        )

    # A broad insulated handle spans the whole front edge and is carried by
    # two stout standoffs bonded to the lid casting.
    lid.visual(
        Cylinder(radius=0.025, length=0.540),
        origin=Origin(xyz=(0.425, 0.0, 0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="front_handle",
    )
    for index, y in enumerate((-0.190, 0.190)):
        lid.visual(
            Cylinder(radius=0.014, length=0.075),
            origin=Origin(xyz=(0.425, y, 0.002), rpy=(0.0, 0.0, 0.0)),
            material=black,
            name=f"handle_standoff_{index}",
        )
        lid.visual(
            Box((0.070, 0.055, 0.010)),
            origin=Origin(xyz=(0.425, y, -0.035)),
            material=black,
            name=f"handle_mount_{index}",
        )

    # Pull-out grease drawer.  Child origin is the base-front slot; positive
    # prismatic travel pulls the tray toward the operator.
    grease_drawer = model.part("grease_drawer")
    grease_drawer.visual(
        mesh_from_cadquery(_grease_drawer_pan(), "tray_pan"),
        material=stainless,
        name="tray_pan",
    )
    grease_drawer.visual(
        Box((0.022, 0.180, 0.018)),
        origin=Origin(xyz=(0.033, 0.0, -0.002)),
        material=black,
        name="drawer_pull",
    )

    # Rotary heat-control knob on the left side.  The stem is intentionally
    # captured inside the side-panel boss while the cap remains proud and free.
    heat_knob = model.part("heat_knob")
    knob_geom = KnobGeometry(
        0.074,
        0.035,
        body_style="skirted",
        top_diameter=0.060,
        skirt=KnobSkirt(0.082, 0.006, flare=0.06, chamfer=0.0015),
        grip=KnobGrip(style="fluted", count=20, depth=0.0016),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        bore=KnobBore(style="round", diameter=0.010),
        center=False,
    )
    heat_knob.visual(
        mesh_from_geometry(knob_geom, "knob_cap"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_cap",
    )
    heat_knob.visual(
        Cylinder(radius=0.008, length=0.011),
        origin=Origin(xyz=(0.0, -0.0055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="knob_stem",
    )
    heat_knob.visual(
        Box((0.006, 0.003, 0.026)),
        origin=Origin(xyz=(0.0, 0.036, 0.012)),
        material=warm_tick,
        name="knob_indicator",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.230, 0.0, 0.360)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=grease_drawer,
        origin=Origin(xyz=(0.250, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.180),
    )
    model.articulation(
        "heat_knob_turn",
        ArticulationType.REVOLUTE,
        parent=base,
        child=heat_knob,
        origin=Origin(xyz=(0.080, 0.323, 0.085)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-2.60, upper=2.60),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("grease_drawer")
    knob = object_model.get_part("heat_knob")
    lid_hinge = object_model.get_articulation("lid_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    heat_knob_turn = object_model.get_articulation("heat_knob_turn")

    # Captured, coaxial mechanical interfaces deliberately share volume.
    for barrel in ("hinge_barrel_0", "hinge_barrel_1"):
        ctx.allow_overlap(
            base,
            lid,
            elem_a="hinge_pin",
            elem_b=barrel,
            reason="The lid hinge barrel is intentionally modeled around the stout rear axle.",
        )
        ctx.expect_overlap(base, lid, axes="y", elem_a="hinge_pin", elem_b=barrel, min_overlap=0.10, name=f"{barrel} retained on hinge pin")
        ctx.expect_within(base, lid, axes="xz", inner_elem="hinge_pin", outer_elem=barrel, margin=0.002, name=f"{barrel} surrounds hinge pin")

    for boss in ("control_panel", "knob_bezel"):
        ctx.allow_overlap(
            base,
            knob,
            elem_a=boss,
            elem_b="knob_stem",
            reason="The control knob stem is intentionally captured in the side-panel boss.",
        )
        ctx.expect_overlap(base, knob, axes="y", elem_a=boss, elem_b="knob_stem", min_overlap=0.002, name=f"knob stem retained in {boss}")

    with ctx.pose({lid_hinge: 0.0, drawer_slide: 0.0, heat_knob_turn: 0.0}):
        ctx.expect_gap(lid, base, axis="z", positive_elem="upper_grill_plate", negative_elem="lower_grill_plate", min_gap=0.010, max_gap=0.030, name="closed press plate clearance")
        ctx.expect_overlap(lid, base, axes="xy", elem_a="upper_grill_plate", elem_b="lower_grill_plate", min_overlap=0.34, name="upper plate covers lower plate")
        ctx.expect_within(drawer, base, axes="y", inner_elem="tray_pan", outer_elem="top_deck", margin=0.002, name="grease drawer centered in base slot")
        ctx.expect_gap(base, drawer, axis="z", positive_elem="top_deck", negative_elem="tray_pan", min_gap=0.035, max_gap=0.060, name="drawer clears underside of deck")
        ctx.expect_overlap(drawer, base, axes="x", elem_a="tray_pan", elem_b="top_deck", min_overlap=0.28, name="closed drawer remains deeply inserted")
        ctx.expect_gap(knob, base, axis="y", positive_elem="knob_cap", negative_elem="knob_bezel", min_gap=0.0, max_gap=0.010, name="knob cap sits proud of side bezel")
        rest_handle_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
        rest_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({lid_hinge: 1.05}):
        raised_handle_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
        ctx.check(
            "lid opens upward on rear tower hinge",
            rest_handle_aabb is not None
            and raised_handle_aabb is not None
            and raised_handle_aabb[0][2] > rest_handle_aabb[0][2] + 0.20,
            details=f"rest_handle_aabb={rest_handle_aabb}, raised_handle_aabb={raised_handle_aabb}",
        )

    with ctx.pose({drawer_slide: 0.180}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(drawer, base, axes="x", elem_a="tray_pan", elem_b="top_deck", min_overlap=0.10, name="extended drawer retains insertion")
        ctx.expect_within(drawer, base, axes="y", inner_elem="tray_pan", outer_elem="top_deck", margin=0.002, name="extended drawer stays in slide channel")
        ctx.check(
            "grease drawer slides out the front",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.15,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    ctx.check(
        "heat knob has bounded rotary control travel",
        heat_knob_turn.motion_limits is not None
        and heat_knob_turn.motion_limits.lower is not None
        and heat_knob_turn.motion_limits.upper is not None
        and heat_knob_turn.motion_limits.lower < 0.0
        and heat_knob_turn.motion_limits.upper > 0.0,
        details=f"limits={heat_knob_turn.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
