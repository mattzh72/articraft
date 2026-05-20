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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


STAINLESS = Material("brushed_stainless", color=(0.72, 0.72, 0.68, 1.0))
DARK_STEEL = Material("dark_steel", color=(0.06, 0.065, 0.07, 1.0))
BLACK_RUBBER = Material("black_rubber", color=(0.005, 0.005, 0.004, 1.0))
SMOKED_GLASS = Material("smoked_glass", color=(0.25, 0.45, 0.55, 0.38))
WHITE_MARK = Material("white_marking", color=(0.92, 0.92, 0.86, 1.0))


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(radius: float, length: float):
    """CadQuery cylinder with its axis on local Y and centered on the origin."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90)
    )


def _annulus_y(outer_radius: float, inner_radius: float, thickness: float):
    """Annular disk in the local XZ plane, with thickness along local Y."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90)
    )


def _dryer_body_shell():
    width = 1.05
    depth = 0.86
    height = 1.42
    bottom_z = 0.08
    wall = 0.035
    front_y = -depth / 2
    rear_y = depth / 2
    center_z = bottom_z + height / 2
    opening_z = 0.80
    opening_radius = 0.375

    left = _box_at((wall, depth, height), (-width / 2 + wall / 2, 0.0, center_z))
    right = _box_at((wall, depth, height), (width / 2 - wall / 2, 0.0, center_z))
    top = _box_at((width, depth, wall), (0.0, 0.0, bottom_z + height - wall / 2))
    bottom = _box_at((width, depth, wall), (0.0, 0.0, bottom_z + wall / 2))
    rear = _box_at((width, wall, height), (0.0, rear_y - wall / 2, center_z))

    front_plate = _box_at((width, wall, height), (0.0, front_y + wall / 2, center_z))
    cutter = _cylinder_y(opening_radius, wall * 4.0).translate(
        (0.0, front_y + wall / 2, opening_z)
    )
    front_plate = front_plate.cut(cutter)

    # A raised stainless flange around the porthole makes the sheet-metal cutout
    # read as a reinforced commercial dryer front rather than a flat decal.
    porthole_flange = _annulus_y(0.435, opening_radius, 0.024).translate(
        (0.0, front_y - 0.012, opening_z)
    )

    return (
        left.union(right)
        .union(top)
        .union(bottom)
        .union(rear)
        .union(front_plate)
        .union(porthole_flange)
    )


def _drum_geometry():
    radius = 0.335
    wall = 0.018
    length = 0.60

    outer = _cylinder_y(radius, length)
    inner = _cylinder_y(radius - wall, length + 0.04)
    shell = outer.cut(inner)
    rear_disk = _cylinder_y(radius - wall * 0.7, wall).translate(
        (0.0, length / 2 - wall / 2, 0.0)
    )
    front_lip = _annulus_y(radius + 0.010, radius - 0.040, 0.035).translate(
        (0.0, -length / 2 + 0.018, 0.0)
    )
    hub = _cylinder_y(0.055, 0.070).translate((0.0, length / 2 + 0.020, 0.0))

    drum = shell.union(rear_disk).union(front_lip).union(hub)

    # Three raised lifters are fused into the rotating drum so the cylinder reads
    # as a real laundry drum, not just a smooth pipe.
    for angle_deg in (18.0, 138.0, 258.0):
        lifter = _box_at(
            (0.040, length * 0.78, 0.055),
            (radius - wall - 0.020, -0.025, 0.0),
        )
        lifter = lifter.rotate((0, 0, 0), (0, 1, 0), angle_deg)
        drum = drum.union(lifter)

    return drum


def _door_frame_geometry():
    center_x = 0.445
    frame = _annulus_y(0.375, 0.245, 0.075).translate((center_x, 0.0, 0.0))
    inner_gasket = _annulus_y(0.267, 0.232, 0.028).translate((center_x, -0.006, 0.0))
    outer_bead = _annulus_y(0.392, 0.368, 0.032).translate((center_x, -0.002, 0.0))
    return frame.union(inner_gasket).union(outer_bead)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_laundry_dryer")

    width = 1.05
    depth = 0.86
    front_y = -depth / 2
    opening_z = 0.80
    hinge_x = -0.445
    hinge_y = front_y - 0.065

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_dryer_body_shell(), "stainless_body_shell"),
        material=STAINLESS,
        name="body_shell",
    )
    body.visual(
        Box((0.88, 0.038, 0.20)),
        origin=Origin(xyz=(0.0, front_y - 0.018, 1.31)),
        material=DARK_STEEL,
        name="control_panel",
    )
    body.visual(
        Box((0.19, 0.008, 0.060)),
        origin=Origin(xyz=(-0.20, front_y - 0.040, 1.335)),
        material=BLACK_RUBBER,
        name="display_window",
    )
    body.visual(
        Box((0.12, 0.010, 0.020)),
        origin=Origin(xyz=(-0.205, front_y - 0.040, 1.275)),
        material=STAINLESS,
        name="coin_slot",
    )
    body.visual(
        Box((0.055, 0.009, 0.012)),
        origin=Origin(xyz=(-0.20, front_y - 0.040, 1.365)),
        material=WHITE_MARK,
        name="status_label",
    )
    # Alternating hinge knuckles and wide leaves are fixed to the cabinet; the
    # moving companion knuckles live on the door part.
    body.visual(
        Box((0.050, 0.055, 0.825)),
        origin=Origin(xyz=(hinge_x - 0.088, front_y - 0.025, opening_z)),
        material=STAINLESS,
        name="hinge_reinforcement",
    )
    for index, local_z in enumerate((-0.290, 0.0, 0.290)):
        body.visual(
            Cylinder(radius=0.026, length=0.150),
            origin=Origin(xyz=(hinge_x, hinge_y, opening_z + local_z)),
            material=DARK_STEEL,
            name=f"body_hinge_knuckle_{index}",
        )
        body.visual(
            Box((0.105, 0.055, 0.130)),
            origin=Origin(
                xyz=(hinge_x - 0.048, front_y - 0.025, opening_z + local_z)
            ),
            material=STAINLESS,
            name=f"body_hinge_leaf_{index}",
        )

    # A fixed rear bearing boss visually supports the drum axle at the back of
    # the dryer cabinet.
    body.visual(
        Box((0.160, 0.070, 0.160)),
        origin=Origin(xyz=(0.0, depth / 2 - 0.035, opening_z)),
        material=DARK_STEEL,
        name="rear_bearing_block",
    )
    body.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(
            xyz=(0.0, depth / 2 - 0.055, opening_z),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        material=DARK_STEEL,
        name="rear_bearing",
    )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_geometry(), "rotating_drum"),
        material=STAINLESS,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.034, length=0.145),
        origin=Origin(xyz=(0.0, 0.3425, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=DARK_STEEL,
        name="axle_stub",
    )
    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, -0.055, opening_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=18.0),
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame_geometry(), "round_porthole_door"),
        material=STAINLESS,
        name="door_frame",
    )
    door.visual(
        Cylinder(radius=0.232, length=0.018),
        origin=Origin(xyz=(0.445, -0.004, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=SMOKED_GLASS,
        name="glass_porthole",
    )
    door.visual(
        Box((0.050, 0.060, 0.170)),
        origin=Origin(xyz=(0.780, -0.045, 0.0)),
        material=DARK_STEEL,
        name="latch_handle",
    )
    for index, local_z in enumerate((-0.145, 0.145)):
        door.visual(
            Cylinder(radius=0.023, length=0.140),
            origin=Origin(xyz=(0.0, 0.0, local_z)),
            material=DARK_STEEL,
            name=f"door_hinge_knuckle_{index}",
        )
        door.visual(
            Box((0.155, 0.026, 0.060)),
            origin=Origin(xyz=(0.080, -0.006, local_z)),
            material=STAINLESS,
            name=f"door_hinge_leaf_{index}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, opening_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.85),
    )

    knob = model.part("cycle_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.085,
            0.040,
            body_style="skirted",
            top_diameter=0.066,
            edge_radius=0.002,
            grip=KnobGrip(style="fluted", count=22, depth=0.002),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "cycle_selector_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=DARK_STEEL,
        name="knob_cap",
    )
    model.articulation(
        "body_to_cycle_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.270, front_y - 0.037, 1.325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=4.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    knob = object_model.get_part("cycle_knob")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    knob_joint = object_model.get_articulation("body_to_cycle_knob")

    ctx.expect_within(
        drum,
        body,
        axes="xz",
        inner_elem="drum_shell",
        outer_elem="body_shell",
        margin=0.010,
        name="drum is centered inside stainless cabinet opening",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        max_penetration=0.0,
        positive_elem="body_shell",
        negative_elem="door_frame",
        name="closed porthole door stays just in front of cabinet",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.30,
        elem_a="door_frame",
        elem_b="body_shell",
        name="round door covers the porthole opening",
    )
    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_cap",
        elem_b="control_panel",
        contact_tol=0.004,
        name="cycle knob is mounted on the control panel",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=0.005,
            positive_elem="body_shell",
            negative_elem="door_frame",
            name="open door swings outward on left hinge",
        )
    ctx.check(
        "door hinge moves the handle forward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.18,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    with ctx.pose({drum_joint: math.pi / 2.0, knob_joint: 1.0}):
        ctx.expect_within(
            drum,
            body,
            axes="xz",
            inner_elem="drum_shell",
            outer_elem="body_shell",
            margin=0.010,
            name="spinning drum remains captured by the cabinet shell",
        )

    return ctx.report()


object_model = build_object_model()
