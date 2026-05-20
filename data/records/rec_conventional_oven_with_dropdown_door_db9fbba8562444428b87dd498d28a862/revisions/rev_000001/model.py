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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.48
BODY_D = 0.34
BODY_H = 0.26
FRONT_Y = -BODY_D / 2.0

WALL = 0.018
BACK_WALL = 0.020
BOTTOM_WALL = 0.045
TOP_WALL = 0.035

OPENING_X = -0.035
OPENING_W = 0.360
OPENING_Z = BOTTOM_WALL
OPENING_H = BODY_H - BOTTOM_WALL - TOP_WALL

DOOR_W = 0.382
DOOR_H = 0.205
DOOR_T = 0.016
HINGE_Y = FRONT_Y - 0.010
HINGE_Z = 0.038

CONTROL_X = 0.185


def _body_shell_mesh():
    """Continuous chrome oven shell with an open cooking chamber."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
    )
    front_cut_min_y = FRONT_Y - 0.055
    inner_back_y = BODY_D / 2.0 - BACK_WALL
    cutter = (
        cq.Workplane("XY")
        .box(
            OPENING_W,
            inner_back_y - front_cut_min_y,
            OPENING_H,
        )
        .translate(
            (
                OPENING_X,
                (front_cut_min_y + inner_back_y) / 2.0,
                OPENING_Z + OPENING_H / 2.0,
            )
        )
    )
    shell = outer.cut(cutter)
    # Small corner radii keep the chrome toaster-oven body from reading as a raw box.
    try:
        shell = shell.edges("|Z").fillet(0.006)
    except Exception:
        pass
    return mesh_from_cadquery(shell, "chrome_body_shell", tolerance=0.0015)


def _control_knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.024,
            body_style="skirted",
            top_diameter=0.032,
            skirt=KnobSkirt(0.050, 0.005, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0013),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    chrome = model.material("soft_chrome", rgba=(0.76, 0.78, 0.76, 1.0))
    dark = model.material("black_cavity", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("smoky_blue_glass", rgba=(0.28, 0.56, 0.72, 0.38))
    black = model.material("matte_black", rgba=(0.03, 0.03, 0.032, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.01, 0.01, 0.01, 1.0))
    heat = model.material("warm_heating_element", rgba=(1.0, 0.32, 0.07, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=chrome, name="body_shell")
    body.visual(
        Box((0.082, 0.004, 0.185)),
        origin=Origin(xyz=(CONTROL_X, FRONT_Y - 0.0015, 0.142)),
        material=black,
        name="control_faceplate",
    )
    body.visual(
        Box((OPENING_W - 0.030, 0.006, OPENING_H - 0.030)),
        origin=Origin(
            xyz=(
                OPENING_X,
                BODY_D / 2.0 - BACK_WALL - 0.003,
                OPENING_Z + OPENING_H / 2.0,
            )
        ),
        material=dark,
        name="dark_rear_liner",
    )
    body.visual(
        Box((OPENING_W - 0.020, 0.230, 0.004)),
        origin=Origin(xyz=(OPENING_X, -0.012, OPENING_Z + 0.002)),
        material=dark,
        name="crumb_tray_liner",
    )
    # Two visible warm quartz rods span the chamber and are anchored into side walls.
    for z, name in ((0.085, "lower_heater"), (0.190, "upper_heater")):
        body.visual(
            Cylinder(radius=0.0055, length=OPENING_W + 0.008),
            origin=Origin(xyz=(OPENING_X, -0.020, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=heat,
            name=name,
        )
    # Four squat rubber feet slightly tuck under the chrome base.
    for ix, x in enumerate((-0.175, 0.175)):
        for iy, y in enumerate((-0.105, 0.105)):
            body.visual(
                Cylinder(radius=0.025, length=0.018),
                origin=Origin(xyz=(x, y, -0.008)),
                material=rubber,
                name=f"foot_{ix}_{iy}",
            )

    left_pin_x = OPENING_X - DOOR_W / 2.0 + 0.014
    right_pin_x = OPENING_X + DOOR_W / 2.0 - 0.014
    body.visual(
        Cylinder(radius=0.0052, length=0.040),
        origin=Origin(xyz=(left_pin_x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_pin_0",
    )
    body.visual(
        Box((0.012, 0.012, 0.020)),
        origin=Origin(xyz=(left_pin_x - 0.022, FRONT_Y - 0.002, HINGE_Z)),
        material=chrome,
        name="hinge_support_0",
    )
    body.visual(
        Cylinder(radius=0.0052, length=0.040),
        origin=Origin(xyz=(right_pin_x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_pin_1",
    )
    body.visual(
        Box((0.012, 0.012, 0.020)),
        origin=Origin(xyz=(right_pin_x + 0.022, FRONT_Y - 0.002, HINGE_Z)),
        material=chrome,
        name="hinge_support_1",
    )

    door = model.part("door")
    rail_h = 0.030
    stile_w = 0.030
    bottom_rail_z0 = 0.012
    bottom_rail_z1 = bottom_rail_z0 + rail_h
    top_rail_z0 = DOOR_H - rail_h
    stile_h = top_rail_z0 - bottom_rail_z1
    stile_z = bottom_rail_z1 + stile_h / 2.0
    # Chrome perimeter frame around an inset transparent pane.
    door.visual(
        Box((DOOR_W, DOOR_T, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, bottom_rail_z0 + rail_h / 2.0)),
        material=chrome,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_W, DOOR_T, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, DOOR_H - rail_h / 2.0)),
        material=chrome,
        name="top_rail",
    )
    for side, x in (("0", -DOOR_W / 2.0 + stile_w / 2.0), ("1", DOOR_W / 2.0 - stile_w / 2.0)):
        door.visual(
            Box((stile_w, DOOR_T, stile_h)),
            origin=Origin(xyz=(x, 0.0, stile_z)),
            material=chrome,
            name=f"side_stile_{side}",
        )
    door.visual(
        Box((DOOR_W - 2.0 * stile_w + 0.006, 0.006, DOOR_H - 2.0 * rail_h + 0.006)),
        origin=Origin(xyz=(0.0, 0.001, (bottom_rail_z1 + top_rail_z0) / 2.0)),
        material=glass,
        name="glass_panel",
    )
    # A cool-touch handle, carried by two posts, sits proud of the glass door.
    door.visual(
        Cylinder(radius=0.007, length=0.245),
        origin=Origin(xyz=(0.0, -0.044, 0.155), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="handle_bar",
    )
    for i, x in enumerate((-0.105, 0.105)):
        door.visual(
            Cylinder(radius=0.0045, length=0.046),
            origin=Origin(xyz=(x, -0.024, 0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"handle_post_{i}",
        )
    door.visual(
        Box((0.030, DOOR_T, 0.034)),
        origin=Origin(xyz=(-DOOR_W / 2.0 + 0.014, 0.0, 0.0245)),
        material=chrome,
        name="hinge_strap_0",
    )
    door.visual(
        Box((0.030, DOOR_T, 0.034)),
        origin=Origin(xyz=(DOOR_W / 2.0 - 0.014, 0.0, 0.0245)),
        material=chrome,
        name="hinge_strap_1",
    )
    door.visual(
        Cylinder(radius=0.0082, length=0.028),
        origin=Origin(xyz=(-DOOR_W / 2.0 + 0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.0082, length=0.028),
        origin=Origin(xyz=(DOOR_W / 2.0 - 0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel_1",
    )

    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(OPENING_X, HINGE_Y, HINGE_Z)),
        # Closed door rises along +Z from the bottom hinge; +X opens it down/out.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.75),
    )
    door_hinge.meta["qc_samples"] = [0.0, 0.9, 1.55]

    # Three separate appliance controls are rotatable, not painted-on circles.
    for i, z in enumerate((0.195, 0.140, 0.085)):
        knob = model.part(f"knob_{i}")
        knob.visual(
            _control_knob_mesh(f"control_knob_{i}"),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="knob_shell",
        )
        joint = model.articulation(
            f"body_to_knob_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(CONTROL_X, FRONT_Y - 0.0035, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-2.6, upper=2.6),
        )
        joint.meta["qc_samples"] = [-1.2, 0.0, 1.2]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_pin_0",
        elem_b="hinge_barrel_0",
        reason="The side hinge pin is intentionally captured inside the simplified door hinge barrel.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_pin_1",
        elem_b="hinge_barrel_1",
        reason="The side hinge pin is intentionally captured inside the simplified door hinge barrel.",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="xyz",
        min_overlap=0.003,
        elem_a="hinge_pin_0",
        elem_b="hinge_barrel_0",
        name="left hinge pin is captured by barrel",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="xyz",
        min_overlap=0.003,
        elem_a="hinge_pin_1",
        elem_b="hinge_barrel_1",
        name="right hinge pin is captured by barrel",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=0.0005,
            max_gap=0.006,
            positive_elem="body_shell",
            negative_elem="top_rail",
            name="closed door sits just in front of chrome body",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.12,
            elem_a="glass_panel",
            elem_b="body_shell",
            name="glass door covers the oven opening",
        )
        closed_aabb = ctx.part_world_aabb(door)

    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(door)

    if closed_aabb is not None and open_aabb is not None:
        closed_min, closed_max = closed_aabb
        open_min, open_max = open_aabb
        ctx.check(
            "door opens downward from bottom hinge",
            open_max[2] < closed_max[2] - 0.045 and open_min[1] < closed_min[1] - 0.060,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
    else:
        ctx.fail("door pose aabbs available", "Expected closed and open door AABBs.")

    for i in range(3):
        knob = object_model.get_part(f"knob_{i}")
        joint = object_model.get_articulation(f"body_to_knob_{i}")
        ctx.check(f"knob_{i}_is_articulated", knob is not None and joint is not None)

    return ctx.report()


object_model = build_object_model()
