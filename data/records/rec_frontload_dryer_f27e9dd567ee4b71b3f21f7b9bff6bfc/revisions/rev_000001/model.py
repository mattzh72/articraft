from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
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


def _open_drum_mesh(radius: float, length: float, wall: float, *, segments: int = 72) -> MeshGeometry:
    """Thin-walled front-load dryer drum, open at the front and capped at the rear."""
    inner = radius - wall
    y_front = -length / 2.0
    y_back = length / 2.0
    geom = MeshGeometry()

    outer_front: list[int] = []
    outer_back: list[int] = []
    inner_front: list[int] = []
    inner_back: list[int] = []

    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        outer_front.append(geom.add_vertex(radius * ca, y_front, radius * sa))
        outer_back.append(geom.add_vertex(radius * ca, y_back, radius * sa))
        inner_front.append(geom.add_vertex(inner * ca, y_front, inner * sa))
        inner_back.append(geom.add_vertex(inner * ca, y_back, inner * sa))

    rear_center = geom.add_vertex(0.0, y_back, 0.0)

    for i in range(segments):
        j = (i + 1) % segments

        # Outer cylindrical wall.
        geom.add_face(outer_front[i], outer_back[i], outer_back[j])
        geom.add_face(outer_front[i], outer_back[j], outer_front[j])

        # Inner cylindrical wall.
        geom.add_face(inner_front[j], inner_back[j], inner_back[i])
        geom.add_face(inner_front[j], inner_back[i], inner_front[i])

        # Rounded-looking front lip annulus around the open mouth.
        geom.add_face(outer_front[j], inner_front[j], inner_front[i])
        geom.add_face(outer_front[j], inner_front[i], outer_front[i])

        # Rear annulus and rear drum plate.
        geom.add_face(outer_back[i], inner_back[i], inner_back[j])
        geom.add_face(outer_back[i], inner_back[j], outer_back[j])
        geom.add_face(rear_center, inner_back[j], inner_back[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_front_load_dryer")

    white = model.material("warm_white_enamel", rgba=(0.94, 0.94, 0.90, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.02, 0.022, 0.025, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.69, 1.0))
    dark_steel = model.material("dark_perforated_steel", rgba=(0.33, 0.35, 0.36, 1.0))
    glass = model.material("smoked_glass", rgba=(0.20, 0.32, 0.42, 0.42))
    panel = model.material("control_panel_grey", rgba=(0.77, 0.78, 0.76, 1.0))

    width = 0.74
    depth = 0.72
    height = 1.02
    wall = 0.035
    drum_z = 0.54
    drum_y = -0.02
    front_y = -depth / 2.0

    body = model.part("body")

    # Cabinet shell panels leave a real open cavity for the rotating drum.
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_0",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_1",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=white,
        name="top_panel",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=white,
        name="bottom_panel",
    )
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=white,
        name="rear_panel",
    )

    front_panel_y = front_y + wall / 2.0
    body.visual(
        Box((0.12, wall, 0.72)),
        origin=Origin(xyz=(-0.31, front_panel_y, 0.52)),
        material=white,
        name="front_jamb_0",
    )
    body.visual(
        Box((0.12, wall, 0.72)),
        origin=Origin(xyz=(0.31, front_panel_y, 0.52)),
        material=white,
        name="front_jamb_1",
    )
    body.visual(
        Box((width, wall, 0.22)),
        origin=Origin(xyz=(0.0, front_panel_y, 0.91)),
        material=white,
        name="front_header",
    )
    body.visual(
        Box((width, wall, 0.20)),
        origin=Origin(xyz=(0.0, front_panel_y, 0.10)),
        material=white,
        name="front_sill",
    )

    front_trim = BezelGeometry(
        opening_size=(0.50, 0.50),
        outer_size=(0.63, 0.63),
        depth=0.026,
        opening_shape="circle",
        outer_shape="circle",
        face=BezelFace(style="radiused_step", front_lip=0.005, fillet=0.004),
    )
    body.visual(
        mesh_from_geometry(front_trim, "front_trim"),
        origin=Origin(xyz=(0.0, front_y - 0.013, drum_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="front_trim",
    )
    body.visual(
        Cylinder(radius=0.255, length=0.010),
        origin=Origin(xyz=(0.0, front_y - 0.030, drum_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="door_gasket",
    )

    # A modest raised console and a single rotary timer control keep the appliance readable.
    body.visual(
        Box((0.52, 0.034, 0.12)),
        origin=Origin(xyz=(0.02, front_y - 0.016, 0.925)),
        material=panel,
        name="control_console",
    )
    body.visual(
        Box((0.18, 0.008, 0.035)),
        origin=Origin(xyz=(-0.16, front_y - 0.033, 0.925)),
        material=shadow,
        name="display_slot",
    )
    body.visual(
        Box((0.040, 0.057, 0.60)),
        origin=Origin(xyz=(-0.325, -0.3885, drum_z)),
        material=white,
        name="hinge_receiver",
    )
    body.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall - 0.012, drum_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bearing",
    )

    # Short feet are integrated into the root body, visibly lifting the white cabinet.
    for idx, x in enumerate((-0.27, 0.27)):
        for idy, y in enumerate((-0.24, 0.24)):
            body.visual(
                Box((0.10, 0.10, 0.035)),
                origin=Origin(xyz=(x, y, -0.0175)),
                material=rubber,
                name=f"foot_{idx}_{idy}",
            )

    drum = model.part("drum")
    drum.visual(
        mesh_from_geometry(_open_drum_mesh(0.245, 0.50, 0.018), "drum_shell"),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.035, length=0.642),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="central_axle",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        x = 0.1995 * math.cos(angle)
        z = 0.1995 * math.sin(angle)
        drum.visual(
            Box((0.055, 0.38, 0.030)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, -angle, 0.0)),
            material=dark_steel,
            name=f"baffle_{i}",
        )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, drum_y, drum_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    door = model.part("door")
    door_center_offset = 0.325
    door_y = front_y - 0.075
    door_ring = BezelGeometry(
        opening_size=(0.405, 0.405),
        outer_size=(0.570, 0.570),
        depth=0.050,
        opening_shape="circle",
        outer_shape="circle",
        face=BezelFace(style="radiused_step", front_lip=0.006, fillet=0.006),
    )
    door.visual(
        mesh_from_geometry(door_ring, "door_ring"),
        origin=Origin(xyz=(door_center_offset, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.207, length=0.010),
        origin=Origin(xyz=(door_center_offset, -0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_window",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=white,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.060, 0.025, 0.46)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=white,
        name="hinge_leaf",
    )
    door.visual(
        Box((0.055, 0.034, 0.090)),
        origin=Origin(xyz=(door_center_offset + 0.265, -0.012, 0.0)),
        material=white,
        name="pull_lip",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-door_center_offset, door_y, drum_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.9),
    )

    knob = model.part("timer_knob")
    knob.visual(
        Cylinder(radius=0.043, length=0.032),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="knob_cap",
    )
    knob.visual(
        Box((0.010, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        material=shadow,
        name="pointer_mark",
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.205, front_y - 0.045, 0.925)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0, lower=-2.6, upper=2.6),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    knob = object_model.get_part("timer_knob")
    drum_spin = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    knob_turn = object_model.get_articulation("body_to_timer_knob")

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="door_ring",
        elem_b="front_trim",
        min_overlap=0.45,
        name="closed circular door is centered over the porthole trim",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_trim",
        negative_elem="door_ring",
        min_gap=0.004,
        max_gap=0.040,
        name="closed door sits just proud of the front trim",
    )
    ctx.expect_overlap(
        drum,
        door,
        axes="xz",
        elem_a="drum_shell",
        elem_b="glass_window",
        min_overlap=0.35,
        name="drum is visible through the glass porthole",
    )
    ctx.expect_gap(
        body,
        drum,
        axis="y",
        positive_elem="rear_bearing",
        negative_elem="central_axle",
        max_gap=0.050,
        max_penetration=0.001,
        name="central axle terminates at the rear bearing",
    )

    rest_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    rest_door_center = _aabb_center(rest_door_aabb)
    open_door_center = _aabb_center(open_door_aabb)
    ctx.check(
        "door hinge swings the porthole outward",
        rest_door_center is not None
        and open_door_center is not None
        and open_door_center[1] < rest_door_center[1] - 0.18,
        details=f"rest={rest_door_center}, open={open_door_center}",
    )

    rest_baffle = ctx.part_element_world_aabb(drum, elem="baffle_0")
    with ctx.pose({drum_spin: 1.10}):
        spun_baffle = ctx.part_element_world_aabb(drum, elem="baffle_0")
    rest_baffle_center = _aabb_center(rest_baffle)
    spun_baffle_center = _aabb_center(spun_baffle)
    ctx.check(
        "drum revolute axle carries the internal baffles around the central axis",
        rest_baffle_center is not None
        and spun_baffle_center is not None
        and abs(spun_baffle_center[2] - rest_baffle_center[2]) > 0.10,
        details=f"rest={rest_baffle_center}, spun={spun_baffle_center}",
    )

    rest_knob = ctx.part_element_world_aabb(knob, elem="pointer_mark")
    with ctx.pose({knob_turn: 1.2}):
        turned_knob = ctx.part_element_world_aabb(knob, elem="pointer_mark")
    ctx.check(
        "timer knob has a visible rotary control articulation",
        rest_knob is not None and turned_knob is not None,
        details=f"rest={rest_knob}, turned={turned_knob}",
    )

    return ctx.report()


object_model = build_object_model()
