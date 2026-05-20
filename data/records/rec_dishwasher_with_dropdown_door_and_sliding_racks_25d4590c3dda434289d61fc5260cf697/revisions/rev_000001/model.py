from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
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


BODY_W = 0.62
BODY_D = 0.62
BODY_H = 0.84
FRONT_Y = -BODY_D / 2.0


def _rounded_box(width: float, depth: float, height: float, radius: float):
    """CadQuery rounded box centered at its local origin."""
    shape = cq.Workplane("XY").box(width, depth, height)
    if radius > 0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _tub_shell():
    """One-piece cream outer shell with a visible open-front wash chamber."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(0.035)
    )
    # The cutter runs through the front face and stops before the rear wall,
    # leaving a real open hollow chamber rather than a solid dark insert.
    cavity = (
        cq.Workplane("XY")
        .box(0.52, 0.62, 0.60)
        .translate((0.0, -0.065, 0.43))
    )
    return outer.cut(cavity)


def _door_panel():
    return (
        cq.Workplane("XY")
        .box(0.56, 0.035, 0.62)
        .translate((0.0, -0.0175, 0.328))
        .edges("|Z")
        .fillet(0.012)
    )


def _add_rack(part, *, prefix: str, material: Material) -> None:
    """Connected welded-wire rack: perimeter, grid, and short dish tines."""
    rod = 0.007
    w = 0.468
    d = 0.48
    # Perimeter rail.
    grid_z = -0.018
    part.visual(Box((rod, d, rod)), origin=Origin(xyz=(-w / 2, 0.0, grid_z)), material=material, name=f"{prefix}_side_0")
    part.visual(Box((rod, d, rod)), origin=Origin(xyz=(w / 2, 0.0, grid_z)), material=material, name=f"{prefix}_side_1")
    part.visual(Box((w + rod, rod, rod)), origin=Origin(xyz=(0.0, -d / 2, grid_z)), material=material, name=f"{prefix}_front")
    part.visual(Box((w + rod, rod, rod)), origin=Origin(xyz=(0.0, d / 2, grid_z)), material=material, name=f"{prefix}_rear")

    # Bottom grid. Each wire overlaps the perimeter frame.
    for i, y in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        part.visual(Box((w, 0.0045, 0.0045)), origin=Origin(xyz=(0.0, y, grid_z)), material=material, name=f"{prefix}_cross_{i}")
    for i, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        part.visual(Box((0.0045, d, 0.0045)), origin=Origin(xyz=(x, 0.0, grid_z)), material=material, name=f"{prefix}_long_{i}")

    # Two rows of short upward tines welded into the grid.
    tine_h = 0.075
    for row, y in enumerate((-0.09, 0.09)):
        for col, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
            part.visual(
                Box((0.005, 0.005, tine_h)),
                origin=Origin(xyz=(x, y, 0.020)),
                material=material,
                name=f"{prefix}_tine_{row}_{col}",
            )


def _add_spray_arm(part, *, name_prefix: str, material: Material) -> None:
    part.visual(Box((0.42, 0.034, 0.018)), origin=Origin(), material=material, name=f"{name_prefix}_flat_arm")
    part.visual(Cylinder(radius=0.035, length=0.030), origin=Origin(), material=material, name=f"{name_prefix}_hub")
    # Small raised spray slots make the rotor read as a dishwasher spray arm,
    # and overlap the arm body so they are welded-on details, not floating tabs.
    for i, x in enumerate((-0.155, -0.075, 0.075, 0.155)):
        part.visual(
            Box((0.042, 0.006, 0.004)),
            origin=Origin(xyz=(x, 0.010 if i % 2 == 0 else -0.010, 0.010), rpy=(0.0, 0.0, 0.35 if i % 2 == 0 else -0.35)),
            material=material,
            name=f"{name_prefix}_spray_slot_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_dishwasher")

    cream = model.material("warm_ivory_enamel", rgba=(0.86, 0.78, 0.62, 1.0))
    mint = model.material("rounded_mint_fascia", rgba=(0.38, 0.72, 0.68, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.68, 0.69, 0.66, 1.0))
    dark_stainless = model.material("shadowed_stainless_tub", rgba=(0.38, 0.39, 0.38, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.88, 0.88, 0.84, 1.0))
    rack_wire = model.material("coated_wire", rgba=(0.92, 0.93, 0.90, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    red = model.material("red_square_button", rgba=(0.72, 0.10, 0.08, 1.0))
    blue = model.material("blue_square_button", rgba=(0.10, 0.24, 0.62, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_tub_shell(), "tub_shell"), material=cream, name="tub_shell")
    # Stainless liner surfaces, all slightly let into the cream shell so the
    # open wash chamber reads as a real metal tub with depth.
    body.visual(Box((0.515, 0.010, 0.585)), origin=Origin(xyz=(0.0, 0.240, 0.430)), material=dark_stainless, name="rear_liner")
    body.visual(Box((0.010, 0.545, 0.585)), origin=Origin(xyz=(-0.262, -0.035, 0.430)), material=dark_stainless, name="side_liner_0")
    body.visual(Box((0.010, 0.545, 0.585)), origin=Origin(xyz=(0.262, -0.035, 0.430)), material=dark_stainless, name="side_liner_1")
    body.visual(Box((0.515, 0.545, 0.010)), origin=Origin(xyz=(0.0, -0.035, 0.135)), material=dark_stainless, name="floor_liner")
    body.visual(Box((0.515, 0.545, 0.010)), origin=Origin(xyz=(0.0, -0.035, 0.725)), material=dark_stainless, name="ceiling_liner")

    fascia = _rounded_box(0.59, 0.035, 0.115, 0.012).translate((0.0, FRONT_Y - 0.017, 0.775))
    body.visual(mesh_from_cadquery(fascia, "rounded_fascia"), material=mint, name="rounded_fascia")
    body.visual(Box((0.58, 0.018, 0.060)), origin=Origin(xyz=(0.0, FRONT_Y - 0.0085, 0.038)), material=cream, name="lower_kickplate")
    body.visual(Cylinder(radius=0.032, length=0.055), origin=Origin(xyz=(0.0, -0.035, 0.1625)), material=dark_stainless, name="lower_spray_pedestal")
    body.visual(Box((0.035, 0.285, 0.012)), origin=Origin(xyz=(0.0, 0.102, 0.511)), material=dark_stainless, name="upper_spray_feed")
    body.visual(Cylinder(radius=0.006, length=0.62), origin=Origin(xyz=(0.0, FRONT_Y - 0.037, 0.062), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name="door_hinge_pin")
    for i, x in enumerate((-0.300, 0.300)):
        body.visual(Box((0.022, 0.052, 0.026)), origin=Origin(xyz=(x, FRONT_Y - 0.023, 0.062)), material=chrome, name=f"hinge_bracket_{i}")

    # Side runners in the chamber for both sliding wire racks.
    for level, z in enumerate((0.300, 0.560)):
        for side, x in enumerate((-0.250, 0.250)):
            body.visual(
                Box((0.026, 0.500, 0.012)),
                origin=Origin(xyz=(x, -0.035, z)),
                material=chrome,
                name=f"runner_{level}_{side}",
            )

    # Door and bottom hinge.
    door = model.part("door")
    door.visual(mesh_from_cadquery(_door_panel(), "brushed_door_panel"), material=stainless, name="brushed_panel")
    for i, z in enumerate((0.16, 0.25, 0.34, 0.43, 0.52, 0.61)):
        door.visual(Box((0.46, 0.0025, 0.0022)), origin=Origin(xyz=(0.0, -0.036, z)), material=chrome, name=f"brush_line_{i}")
    door.visual(Cylinder(radius=0.012, length=0.50), origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name="hinge_barrel")
    for i, x in enumerate((-0.18, 0.18)):
        door.visual(Box((0.040, 0.008, 0.012)), origin=Origin(xyz=(x, -0.028, 0.016)), material=chrome, name=f"hinge_leaf_{i}")
    door.visual(Cylinder(radius=0.012, length=0.42), origin=Origin(xyz=(0.0, -0.068, 0.555), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name="front_handle")
    for i, x in enumerate((-0.17, 0.17)):
        door.visual(Box((0.025, 0.040, 0.040)), origin=Origin(xyz=(x, -0.050, 0.555)), material=chrome, name=f"handle_standoff_{i}")

    door_joint = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0190, 0.062)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    # Two dishwasher racks sliding out toward the user on side runners.
    lower_rack = model.part("lower_rack")
    _add_rack(lower_rack, prefix="lower", material=rack_wire)
    lower_slide = model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.035, 0.320)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    upper_rack = model.part("upper_rack")
    _add_rack(upper_rack, prefix="upper", material=rack_wire)
    # Parent-side pivot ears for the cup shelf.
    for i, y in enumerate((-0.125, 0.125)):
        upper_rack.visual(Box((0.018, 0.012, 0.085)), origin=Origin(xyz=(-0.230, y, 0.020)), material=chrome, name=f"shelf_pivot_ear_{i}")
    upper_slide = model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -0.035, 0.585)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.30),
    )

    cup_shelf = model.part("cup_shelf")
    cup_shelf.visual(Box((0.008, 0.235, 0.008)), origin=Origin(), material=chrome, name="pivot_rod")
    cup_shelf.visual(Box((0.180, 0.006, 0.006)), origin=Origin(xyz=(0.090, -0.110, 0.0)), material=rack_wire, name="shelf_front")
    cup_shelf.visual(Box((0.180, 0.006, 0.006)), origin=Origin(xyz=(0.090, 0.110, 0.0)), material=rack_wire, name="shelf_rear")
    cup_shelf.visual(Box((0.006, 0.220, 0.006)), origin=Origin(xyz=(0.180, 0.0, 0.0)), material=rack_wire, name="shelf_outer")
    for i, x in enumerate((0.045, 0.090, 0.135)):
        cup_shelf.visual(Box((0.005, 0.220, 0.005)), origin=Origin(xyz=(x, 0.0, 0.0)), material=rack_wire, name=f"shelf_wire_{i}")
    shelf_joint = model.articulation(
        "upper_rack_to_cup_shelf",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=cup_shelf,
        origin=Origin(xyz=(-0.230, 0.0, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.30),
    )

    # Continuously rotating upper and lower spray arms in the wash chamber.
    lower_arm = model.part("lower_spray_arm")
    _add_spray_arm(lower_arm, name_prefix="lower_arm", material=stainless)
    lower_arm_joint = model.articulation(
        "body_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_arm,
        origin=Origin(xyz=(0.0, -0.035, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    upper_arm = model.part("upper_spray_arm")
    _add_spray_arm(upper_arm, name_prefix="upper_arm", material=stainless)
    upper_arm_joint = model.articulation(
        "body_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=upper_arm,
        origin=Origin(xyz=(0.0, -0.035, 0.490)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    # Right-front cluster: two rotary chrome dials and two separate push buttons.
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.026,
            body_style="skirted",
            skirt=KnobSkirt(0.060, 0.006, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        ),
        "chrome_dial",
    )
    dial_positions = ((0.175, 0.795), (0.250, 0.795))
    for i, (x, z) in enumerate(dial_positions):
        dial = model.part(f"dial_{i}")
        dial.visual(dial_mesh, origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=chrome, name="dial_cap")
        model.articulation(
            f"body_to_dial_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(xyz=(x, FRONT_Y - 0.0395, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    button_specs = ((0.178, 0.735, red, "button_0"), (0.248, 0.735, blue, "button_1"))
    for x, z, mat, name in button_specs:
        button = model.part(name)
        button.visual(Box((0.040, 0.018, 0.032)), origin=Origin(xyz=(0.0, -0.009, 0.0)), material=mat, name="button_cap")
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.0345, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.12, lower=0.0, upper=0.012),
        )

    # Keep references alive for prompt-specific tests via articulation names.
    _ = (door_joint, lower_slide, upper_slide, shelf_joint, lower_arm_joint, upper_arm_joint)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    cup_shelf = object_model.get_part("cup_shelf")
    button_0 = object_model.get_part("button_0")

    door_joint = object_model.get_articulation("body_to_door")
    lower_slide = object_model.get_articulation("body_to_lower_rack")
    upper_slide = object_model.get_articulation("body_to_upper_rack")
    shelf_joint = object_model.get_articulation("upper_rack_to_cup_shelf")
    button_joint = object_model.get_articulation("body_to_button_0")

    ctx.allow_overlap(
        body,
        door,
        elem_a="door_hinge_pin",
        elem_b="hinge_barrel",
        reason="The visible lower hinge uses a fixed chrome pin captured inside the rotating door barrel.",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="x",
        elem_a="door_hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.45,
        name="door barrel is retained along the hinge pin",
    )
    ctx.expect_within(
        body,
        door,
        axes="yz",
        inner_elem="door_hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="hinge pin sits inside the door barrel",
    )

    ctx.expect_within(lower_rack, body, axes="x", margin=0.010, name="lower rack fits between tub sides")
    ctx.expect_within(upper_rack, body, axes="x", margin=0.010, name="upper rack fits between tub sides")
    ctx.expect_overlap(lower_rack, body, axes="y", min_overlap=0.30, name="lower rack retained in tub at rest")
    ctx.expect_overlap(upper_rack, body, axes="y", min_overlap=0.30, name="upper rack retained in tub at rest")

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door drops outward on bottom hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.15
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.18,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({lower_slide: 0.28, upper_slide: 0.25}):
        lower_out = ctx.part_world_position(lower_rack)
        upper_out = ctx.part_world_position(upper_rack)
    ctx.check(
        "both racks slide toward the open front",
        lower_rest is not None
        and upper_rest is not None
        and lower_out is not None
        and upper_out is not None
        and lower_out[1] < lower_rest[1] - 0.20
        and upper_out[1] < upper_rest[1] - 0.18,
        details=f"lower {lower_rest}->{lower_out}, upper {upper_rest}->{upper_out}",
    )

    shelf_rest = ctx.part_world_aabb(cup_shelf)
    with ctx.pose({shelf_joint: 1.10}):
        shelf_folded = ctx.part_world_aabb(cup_shelf)
    ctx.check(
        "cup shelf folds upward on side pivots",
        shelf_rest is not None and shelf_folded is not None and shelf_folded[1][2] > shelf_rest[1][2] + 0.08,
        details=f"rest={shelf_rest}, folded={shelf_folded}",
    )

    button_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_joint: 0.010}):
        button_in = ctx.part_world_position(button_0)
    ctx.check(
        "square button pushes inward",
        button_rest is not None and button_in is not None and button_in[1] > button_rest[1] + 0.008,
        details=f"button {button_rest}->{button_in}",
    )

    return ctx.report()


object_model = build_object_model()
