from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.68
DEPTH = 0.72
HEIGHT = 0.90
WALL = 0.040
FRONT_Y = -DEPTH / 2.0
DRUM_Z = 0.445
DOOR_Z = DRUM_Z
DOOR_HINGE_X = -0.305
DOOR_HINGE_Y = FRONT_Y - 0.035
DOOR_CENTER_X = 0.300


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _circular_front_panel() -> cq.Workplane:
    panel = _box_at((WIDTH, WALL, HEIGHT), (0.0, FRONT_Y + WALL / 2.0, HEIGHT / 2.0))
    cutter = (
        cq.Workplane("XZ")
        .center(0.0, DRUM_Z)
        .circle(0.285)
        .extrude(WALL + 0.020, both=True)
        .translate((0.0, FRONT_Y + WALL / 2.0, 0.0))
    )
    return panel.cut(cutter)


def _washer_cabinet_shape() -> cq.Workplane:
    bottom = _box_at((WIDTH, DEPTH, WALL), (0.0, 0.0, WALL / 2.0))
    top = _box_at((WIDTH, DEPTH, WALL), (0.0, 0.0, HEIGHT - WALL / 2.0))
    side_0 = _box_at((WALL, DEPTH, HEIGHT), (-WIDTH / 2.0 + WALL / 2.0, 0.0, HEIGHT / 2.0))
    side_1 = _box_at((WALL, DEPTH, HEIGHT), (WIDTH / 2.0 - WALL / 2.0, 0.0, HEIGHT / 2.0))
    back = _box_at((WIDTH, WALL, HEIGHT), (0.0, DEPTH / 2.0 - WALL / 2.0, HEIGHT / 2.0))
    front = _circular_front_panel()

    shell = bottom.union(top).union(side_0).union(side_1).union(back).union(front)
    # Small exterior corner radii make the charcoal appliance body look molded
    # rather than like a raw rectangular block.  The fillet is intentionally
    # restrained so the front circular cutout remains crisp.
    return shell.edges("|Z").fillet(0.012)


def _drum_shape() -> cq.Workplane:
    # A front-load washer drum is a shallow open stainless cylinder.  This mesh
    # includes a thin perforated-looking shell, three lifting ribs, and a rear
    # hub so spinning the revolute axle has a visible orientation.
    outer = cq.Workplane("XZ").circle(0.238).extrude(0.2025, both=True)
    inner = cq.Workplane("XZ").circle(0.215).extrude(0.2150, both=True)
    shell = outer.cut(inner)
    rear_disk = cq.Workplane("XZ").circle(0.215).extrude(0.030).translate((0.0, 0.190, 0.0))
    hub = cq.Workplane("XZ").circle(0.060).extrude(0.075).translate((0.0, 0.205, 0.0))
    drum = shell.union(rear_disk).union(hub)

    for angle in (0.0, 120.0, 240.0):
        rib = (
            _box_at((0.032, 0.330, 0.045), (0.0, -0.020, 0.198))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        )
        drum = drum.union(rib)
    return drum


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_front_load_washer")

    charcoal = model.material("dark_charcoal", rgba=(0.025, 0.028, 0.030, 1.0))
    charcoal_2 = model.material("satin_charcoal", rgba=(0.060, 0.065, 0.070, 1.0))
    black_glass = model.material("black_glass", rgba=(0.005, 0.009, 0.012, 0.92))
    display_blue = model.material("cool_display", rgba=(0.050, 0.180, 0.360, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.690, 0.680, 0.630, 1.0))
    drum_metal = model.material("drum_stainless", rgba=(0.780, 0.780, 0.740, 1.0))
    rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    glass = model.material("smoked_glass", rgba=(0.040, 0.095, 0.125, 0.42))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_circular_front_panel(), "front_panel", tolerance=0.0015),
        material=charcoal,
        name="front_panel",
    )
    body.visual(
        Box((WIDTH, DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=charcoal,
        name="bottom_panel",
    )
    body.visual(
        Box((WIDTH, DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - WALL / 2.0)),
        material=charcoal,
        name="top_panel",
    )
    body.visual(
        Box((WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + WALL / 2.0, 0.0, HEIGHT / 2.0)),
        material=charcoal,
        name="side_panel_0",
    )
    body.visual(
        Box((WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - WALL / 2.0, 0.0, HEIGHT / 2.0)),
        material=charcoal,
        name="side_panel_1",
    )
    body.visual(
        Box((WIDTH, WALL, HEIGHT)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - WALL / 2.0, HEIGHT / 2.0)),
        material=charcoal,
        name="rear_panel",
    )

    body.visual(
        Box((0.640, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.006, 0.795)),
        material=charcoal_2,
        name="control_panel",
    )
    body.visual(
        Box((0.300, 0.006, 0.058)),
        origin=Origin(xyz=(-0.115, FRONT_Y - 0.015, 0.800)),
        material=black_glass,
        name="touch_display",
    )
    body.visual(
        Box((0.185, 0.003, 0.011)),
        origin=Origin(xyz=(-0.115, FRONT_Y - 0.0180, 0.804)),
        material=display_blue,
        name="display_readout",
    )
    body.visual(
        Box((0.130, 0.010, 0.028)),
        origin=Origin(xyz=(-0.245, FRONT_Y - 0.005, 0.728)),
        material=charcoal_2,
        name="detergent_drawer",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.500, 0.500),
                (0.610, 0.610),
                0.014,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "recess_gasket",
        ),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.007, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_glass,
        name="recess_gasket",
    )
    body.visual(
        Box((0.040, 0.010, 0.455)),
        origin=Origin(xyz=(DOOR_HINGE_X, FRONT_Y - 0.005, DOOR_Z)),
        material=stainless,
        name="hinge_backplate",
    )
    body.visual(
        Box((0.065, 0.026, 0.050)),
        origin=Origin(xyz=(DOOR_HINGE_X + 0.020, FRONT_Y - 0.013, DOOR_Z - 0.160)),
        material=stainless,
        name="fixed_hinge_leaf_0",
    )
    body.visual(
        Box((0.065, 0.026, 0.050)),
        origin=Origin(xyz=(DOOR_HINGE_X + 0.020, FRONT_Y - 0.013, DOOR_Z + 0.160)),
        material=stainless,
        name="fixed_hinge_leaf_1",
    )
    body.visual(
        Cylinder(radius=0.045, length=0.095),
        origin=Origin(xyz=(0.0, 0.2925, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="bearing_sleeve",
    )
    for idx, (x, y) in enumerate(((-0.255, -0.255), (0.255, -0.255), (-0.255, 0.255), (0.255, 0.255))):
        body.visual(
            Cylinder(radius=0.040, length=0.030),
            origin=Origin(xyz=(x, y, -0.015)),
            material=rubber,
            name=f"foot_{idx}",
        )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_shape(), "drum_shell", tolerance=0.0012),
        material=drum_metal,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.035, length=0.120),
        origin=Origin(xyz=(0.0, 0.255, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="axle_stub",
    )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.398, 0.398),
                (0.582, 0.582),
                0.050,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "door_stainless_bezel",
        ),
        origin=Origin(xyz=(DOOR_CENTER_X, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="stainless_bezel",
    )
    door.visual(
        Cylinder(radius=0.203, length=0.018),
        origin=Origin(xyz=(DOOR_CENTER_X, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="round_glass",
    )
    door.visual(
        Cylinder(radius=0.019, length=0.430),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    for idx, z_offset in enumerate((-0.150, 0.150)):
        door.visual(
            Box((0.145, 0.020, 0.052)),
            origin=Origin(xyz=(0.073, -0.014, z_offset)),
            material=stainless,
            name=f"hinge_strap_{idx}",
        )
    door.visual(
        Box((0.035, 0.022, 0.168)),
        origin=Origin(xyz=(0.557, -0.045, 0.0)),
        material=rubber,
        name="door_handle",
    )

    cycle_knob = model.part("cycle_knob")
    cycle_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.092,
                0.032,
                body_style="skirted",
                top_diameter=0.074,
                grip=KnobGrip(style="fluted", count=24, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", depth=0.0009),
                center=False,
            ),
            "cycle_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="knob_cap",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=display_blue,
        name="button_cap",
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, -0.070, DRUM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=25.0),
        motion_properties=MotionProperties(damping=0.02),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.95, effort=18.0, velocity=1.2),
        motion_properties=MotionProperties(damping=0.08),
    )
    model.articulation(
        "body_to_cycle_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cycle_knob,
        origin=Origin(xyz=(0.215, FRONT_Y - 0.012, 0.800)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(0.083, FRONT_Y - 0.012, 0.800)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.004, effort=4.0, velocity=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    button = object_model.get_part("start_button")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    knob_joint = object_model.get_articulation("body_to_cycle_knob")
    button_joint = object_model.get_articulation("body_to_start_button")

    ctx.check(
        "drum is on a continuous revolute axle",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(drum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={drum_joint.articulation_type}, axis={drum_joint.axis}",
    )
    ctx.check(
        "door hinge is on the left vertical edge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_joint.axis) == (0.0, 0.0, -1.0)
        and door_joint.origin.xyz[0] < -0.25,
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}, origin={door_joint.origin.xyz}",
    )
    ctx.check(
        "smart controls are articulated",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and button_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"knob={knob_joint.articulation_type}, button={button_joint.articulation_type}",
    )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_panel",
        negative_elem="stainless_bezel",
        min_gap=0.010,
        max_gap=0.035,
        name="closed door bezel sits proud of the front panel",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="stainless_bezel",
        elem_b="front_panel",
        min_overlap=0.35,
        name="large round door covers the circular washer opening",
    )
    ctx.expect_gap(
        drum,
        door,
        axis="y",
        positive_elem="drum_shell",
        negative_elem="stainless_bezel",
        min_gap=0.070,
        name="rotating drum is recessed behind the closed door",
    )
    ctx.expect_contact(
        drum,
        body,
        elem_a="axle_stub",
        elem_b="bearing_sleeve",
        contact_tol=0.0015,
        name="drum axle is seated in the rear bearing sleeve",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="hinge_barrel",
        elem_b="fixed_hinge_leaf_0",
        contact_tol=0.002,
        name="door hinge barrel is carried by the body hinge leaf",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    rest_drum_pos = ctx.part_world_position(drum)
    rest_button_pos = ctx.part_world_position(button)
    with ctx.pose({door_joint: 1.25}):
        open_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({drum_joint: math.pi / 2.0}):
        spun_drum_pos = ctx.part_world_position(drum)
    with ctx.pose({button_joint: 0.004}):
        pressed_button_pos = ctx.part_world_position(button)

    ctx.check(
        "door swings outward from the left hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )
    ctx.check(
        "drum spin keeps the axle center fixed",
        rest_drum_pos is not None
        and spun_drum_pos is not None
        and abs(rest_drum_pos[0] - spun_drum_pos[0]) < 1e-6
        and abs(rest_drum_pos[1] - spun_drum_pos[1]) < 1e-6
        and abs(rest_drum_pos[2] - spun_drum_pos[2]) < 1e-6,
        details=f"rest={rest_drum_pos}, spun={spun_drum_pos}",
    )
    ctx.check(
        "start button presses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
