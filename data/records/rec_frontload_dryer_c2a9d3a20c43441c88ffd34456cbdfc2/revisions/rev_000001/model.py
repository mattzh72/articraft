from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _drum_shell_geometry(
    *,
    radius: float = 0.245,
    wall: float = 0.012,
    length: float = 0.440,
    segments: int = 72,
) -> MeshGeometry:
    """A thin, open-front dryer drum shell oriented along local X."""

    geom = MeshGeometry()
    inner_radius = radius - wall
    x_front = -length / 2.0
    x_back = length / 2.0

    outer_front = []
    outer_back = []
    inner_front = []
    inner_back = []

    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        outer_front.append(geom.add_vertex(x_front, radius * ca, radius * sa))
        outer_back.append(geom.add_vertex(x_back, radius * ca, radius * sa))
        inner_front.append(geom.add_vertex(x_front, inner_radius * ca, inner_radius * sa))
        inner_back.append(geom.add_vertex(x_back, inner_radius * ca, inner_radius * sa))

    back_center = geom.add_vertex(x_back, 0.0, 0.0)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        j = (i + 1) % segments
        # Outer cylindrical wall.
        quad(outer_front[i], outer_back[i], outer_back[j], outer_front[j])
        # Inner cylindrical wall, reversed so the visible inside shades correctly.
        quad(inner_front[j], inner_back[j], inner_back[i], inner_front[i])
        # Rolled front lip and closed rear wall.
        quad(outer_front[j], inner_front[j], inner_front[i], outer_front[i])
        quad(outer_back[i], inner_back[i], inner_back[j], outer_back[j])
        geom.add_face(back_center, inner_back[j], inner_back[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drop_down_tumble_dryer")

    white = Material("warm_white_enamel", rgba=(0.92, 0.90, 0.84, 1.0))
    off_white = Material("slightly_glossy_door", rgba=(0.98, 0.97, 0.93, 1.0))
    dark = Material("black_recessed_plastic", rgba=(0.015, 0.017, 0.018, 1.0))
    graphite = Material("graphite_trim", rgba=(0.18, 0.19, 0.19, 1.0))
    steel = Material("brushed_stainless_steel", rgba=(0.64, 0.66, 0.64, 1.0))
    rubber = Material("dark_rubber_gasket", rgba=(0.02, 0.02, 0.018, 1.0))
    glass = Material("smoked_transparent_glass", rgba=(0.20, 0.35, 0.45, 0.36))
    blue = Material("cool_display_blue", rgba=(0.05, 0.20, 0.55, 1.0))

    body = model.part("body")
    # Compact under-counter cabinet: width Y, depth X, height Z.
    body.visual(Box((0.640, 0.040, 0.800)), origin=Origin(xyz=(0.000, -0.390, 0.440)), material=white, name="side_panel_0")
    body.visual(Box((0.640, 0.040, 0.800)), origin=Origin(xyz=(0.000, 0.390, 0.440)), material=white, name="side_panel_1")
    body.visual(Box((0.660, 0.820, 0.045)), origin=Origin(xyz=(0.000, 0.000, 0.842)), material=white, name="top_panel")
    body.visual(Box((0.660, 0.820, 0.055)), origin=Origin(xyz=(0.000, 0.000, 0.068)), material=white, name="bottom_plinth")
    body.visual(Box((0.040, 0.820, 0.775)), origin=Origin(xyz=(0.320, 0.000, 0.455)), material=white, name="back_panel")
    body.visual(
        Cylinder(radius=0.034, length=0.040),
        origin=Origin(xyz=(0.280, 0.000, 0.460), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_bearing",
    )
    body.visual(Box((0.036, 0.820, 0.170)), origin=Origin(xyz=(-0.322, 0.000, 0.746)), material=white, name="control_band")
    body.visual(Box((0.036, 0.820, 0.190)), origin=Origin(xyz=(-0.322, 0.000, 0.170)), material=white, name="lower_apron")
    body.visual(Box((0.036, 0.145, 0.520)), origin=Origin(xyz=(-0.322, -0.337, 0.460)), material=white, name="front_stile_0")
    body.visual(Box((0.036, 0.145, 0.520)), origin=Origin(xyz=(-0.322, 0.337, 0.460)), material=white, name="front_stile_1")
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.385, 0.385),
                (0.610, 0.610),
                0.030,
                opening_shape="circle",
                outer_shape="circle",
                center=True,
            ),
            "front_round_bezel",
        ),
        origin=Origin(xyz=(-0.345, 0.000, 0.460), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="front_bezel",
    )
    body.visual(Box((0.018, 0.230, 0.034)), origin=Origin(xyz=(-0.352, -0.205, 0.760)), material=blue, name="display_window")
    body.visual(Box((0.018, 0.120, 0.018)), origin=Origin(xyz=(-0.340, 0.195, 0.760)), material=dark, name="start_button_socket")
    body.visual(Box((0.016, 0.090, 0.024)), origin=Origin(xyz=(-0.352, 0.000, 0.705)), material=dark, name="top_latch_catch")
    # Alternating fixed hinge barrels on the cabinet side of the bottom hinge.
    for idx, y in enumerate((-0.225, 0.000, 0.225)):
        body.visual(
            Box((0.060, 0.110, 0.020)),
            origin=Origin(xyz=(-0.370, y, 0.170)),
            material=steel,
            name=f"hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=0.014, length=0.090),
            origin=Origin(xyz=(-0.395, y, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_barrel_{idx}",
        )
    for idx, y in enumerate((-0.285, 0.285)):
        body.visual(Box((0.125, 0.078, 0.030)), origin=Origin(xyz=(0.205, y, 0.035)), material=dark, name=f"rubber_foot_{idx}")

    drum = model.part("drum")
    drum.visual(
        mesh_from_geometry(_drum_shell_geometry(), "drum_shell"),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.025, length=0.040),
        origin=Origin(xyz=(0.220, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_axle_stub",
    )
    # Three inward lifters are attached to the shell so the drum reads as a real tumbler.
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        y = 0.225 * math.cos(angle)
        z = 0.225 * math.sin(angle)
        drum.visual(
            Box((0.330, 0.052, 0.046)),
            origin=Origin(xyz=(0.000, y, z), rpy=(angle, 0.0, 0.0)),
            material=steel,
            name=f"drum_lifter_{idx}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.335, 0.335),
                (0.520, 0.520),
                0.045,
                opening_shape="circle",
                outer_shape="circle",
                center=True,
            ),
            "door_frame_mesh",
        ),
        origin=Origin(xyz=(0.005, 0.000, 0.275), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="door_frame",
    )
    door.visual(
        Cylinder(radius=0.176, length=0.008),
        origin=Origin(xyz=(-0.023, 0.000, 0.275), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="porthole_glass",
    )
    door.visual(
        Cylinder(radius=0.187, length=0.012),
        origin=Origin(xyz=(-0.015, 0.000, 0.275), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="glass_gasket",
    )
    # The inner flat region becomes the loading shelf when the door drops down.
    door.visual(Box((0.016, 0.505, 0.390)), origin=Origin(xyz=(0.015, 0.000, 0.195)), material=off_white, name="loading_shelf")
    door.visual(Box((0.030, 0.070, 0.030)), origin=Origin(xyz=(0.019, 0.000, 0.525)), material=dark, name="top_latch_tongue")
    # Side-pull bar handle with two stand-offs bonded into a vertical mounting strip.
    door.visual(Box((0.055, 0.050, 0.330)), origin=Origin(xyz=(-0.030, 0.245, 0.275)), material=off_white, name="handle_mount_strip")
    door.visual(
        Cylinder(radius=0.016, length=0.285),
        origin=Origin(xyz=(-0.051, 0.302, 0.275)),
        material=graphite,
        name="side_pull_bar",
    )
    for idx, z in enumerate((0.165, 0.385)):
        door.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(-0.045, 0.276, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=f"handle_standoff_{idx}",
        )
    for idx, y in enumerate((-0.105, 0.105)):
        door.visual(
            Box((0.055, 0.100, 0.018)),
            origin=Origin(xyz=(-0.005, y, 0.005)),
            material=steel,
            name=f"door_hinge_leaf_{idx}",
        )
        door.visual(
            Cylinder(radius=0.014, length=0.100),
            origin=Origin(xyz=(0.000, y, -0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_knuckle_{idx}",
        )

    cycle_dial = model.part("cycle_dial")
    cycle_dial.visual(
        Cylinder(radius=0.046, length=0.034),
        origin=Origin(xyz=(-0.017, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="dial_cap",
    )
    cycle_dial.visual(Box((0.006, 0.010, 0.045)), origin=Origin(xyz=(-0.036, 0.000, 0.022)), material=white, name="dial_pointer")

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.020, 0.000, 0.460)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=9.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.395, 0.000, 0.195)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.62),
    )
    model.articulation(
        "body_to_cycle_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cycle_dial,
        origin=Origin(xyz=(-0.340, 0.320, 0.760)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    dial = object_model.get_part("cycle_dial")
    door_hinge = object_model.get_articulation("body_to_door")
    drum_spin = object_model.get_articulation("body_to_drum")
    dial_turn = object_model.get_articulation("body_to_cycle_dial")

    ctx.expect_within(drum, body, axes="yz", margin=0.020, name="drum fits inside cabinet width and height")
    ctx.expect_overlap(drum, body, axes="x", min_overlap=0.400, name="drum sits deep in cabinet")
    ctx.expect_gap(
        body,
        door,
        axis="x",
        positive_elem="front_bezel",
        negative_elem="door_frame",
        min_gap=0.006,
        max_gap=0.040,
        name="closed door is seated just proud of front bezel",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        elem_a="door_frame",
        elem_b="front_bezel",
        min_overlap=0.45,
        name="closed door covers the round loading opening",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="x",
        positive_elem="control_band",
        negative_elem="dial_cap",
        min_gap=0.000,
        max_gap=0.020,
        name="cycle dial is mounted on the front control band",
    )

    closed_shelf = ctx.part_element_world_aabb(door, elem="loading_shelf")
    with ctx.pose({door_hinge: 1.56, drum_spin: 1.2, dial_turn: 0.7}):
        opened_shelf = ctx.part_element_world_aabb(door, elem="loading_shelf")
        opened_frame = ctx.part_element_world_aabb(door, elem="door_frame")

    ctx.check(
        "drop down door becomes a shelf",
        closed_shelf is not None
        and opened_shelf is not None
        and opened_shelf[1][0] - opened_shelf[0][0] > 0.38
        and opened_shelf[1][2] - opened_shelf[0][2] < 0.08
        and opened_shelf[0][0] < closed_shelf[0][0] - 0.15,
        details=f"closed={closed_shelf}, opened={opened_shelf}",
    )
    ctx.check(
        "door top swings downward and outward",
        closed_shelf is not None
        and opened_frame is not None
        and opened_frame[1][2] < 0.55
        and opened_frame[0][0] < -0.55,
        details=f"opened_frame={opened_frame}",
    )

    return ctx.report()


object_model = build_object_model()
