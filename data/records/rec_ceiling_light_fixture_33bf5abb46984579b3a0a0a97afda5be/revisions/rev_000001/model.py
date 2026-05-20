from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

HOUSING_W = 0.21
HOUSING_D = 0.21
HOUSING_H = 0.31
POST = 0.018
TOP_RAIL = 0.024
BOTTOM_RAIL = 0.028

ROOF_MAIN = 0.028
ROOF_CROWN = 0.018
STEM_LEN = 0.038
CANOPY_W = 0.145
CANOPY_H = 0.022

BOTTOM_CAP_W = 0.174
BOTTOM_CAP_H = 0.010
FINIAL_STEM = 0.020
FINIAL_BALL = 0.012

SOCKET_RADIUS = 0.012
SOCKET_LEN = 0.110
BULB_RADIUS = 0.040
BULB_CENTER_Z = 0.015

DOOR_DEPTH = 0.012
DOOR_BAR = 0.012
GLASS_DEPTH = 0.004
HINGE_RADIUS = 0.0035
HINGE_SEGMENT = 0.052
DOOR_EDGE_GAP = 0.004
DOOR_Z_GAP = 0.003
DOOR_OPEN = 1.35

OPENING_W = HOUSING_W - 2.0 * POST
OPENING_H = HOUSING_H - TOP_RAIL - BOTTOM_RAIL
DOOR_W = OPENING_W - 2.0 * DOOR_EDGE_GAP
DOOR_H = OPENING_H - 2.0 * DOOR_Z_GAP
FRAME_FACE = HOUSING_W / 2.0 - POST / 2.0
HINGE_OFFSET = OPENING_W / 2.0 - DOOR_EDGE_GAP + 0.0005

DOOR_SPECS = (
    {
        "door": "door_0",
        "joint": "body_to_door_0",
        "origin": (HINGE_OFFSET, FRAME_FACE, 0.0),
        "yaw": 0.0,
        "axis_name": "y",
        "axis_sign": 1.0,
    },
    {
        "door": "door_1",
        "joint": "body_to_door_1",
        "origin": (FRAME_FACE, -HINGE_OFFSET, 0.0),
        "yaw": -pi / 2.0,
        "axis_name": "x",
        "axis_sign": 1.0,
    },
    {
        "door": "door_2",
        "joint": "body_to_door_2",
        "origin": (-HINGE_OFFSET, -FRAME_FACE, 0.0),
        "yaw": pi,
        "axis_name": "y",
        "axis_sign": -1.0,
    },
    {
        "door": "door_3",
        "joint": "body_to_door_3",
        "origin": (-FRAME_FACE, HINGE_OFFSET, 0.0),
        "yaw": pi / 2.0,
        "axis_name": "x",
        "axis_sign": -1.0,
    },
)


def _add_housing(body) -> None:
    post_offsets = (
        (HOUSING_W / 2.0 - POST / 2.0, HOUSING_D / 2.0 - POST / 2.0),
        (-HOUSING_W / 2.0 + POST / 2.0, HOUSING_D / 2.0 - POST / 2.0),
        (-HOUSING_W / 2.0 + POST / 2.0, -HOUSING_D / 2.0 + POST / 2.0),
        (HOUSING_W / 2.0 - POST / 2.0, -HOUSING_D / 2.0 + POST / 2.0),
    )
    for index, (x_pos, y_pos) in enumerate(post_offsets):
        body.visual(
            Box((POST, POST, HOUSING_H)),
            origin=Origin(xyz=(x_pos, y_pos, 0.0)),
            material="frame_metal",
            name=f"post_{index}",
        )

    top_z = HOUSING_H / 2.0 - TOP_RAIL / 2.0
    bottom_z = -HOUSING_H / 2.0 + BOTTOM_RAIL / 2.0
    face_y = HOUSING_D / 2.0 - POST / 2.0
    face_x = HOUSING_W / 2.0 - POST / 2.0

    rail_specs = (
        ("top_rail_0", (OPENING_W, POST, TOP_RAIL), (0.0, face_y, top_z)),
        ("top_rail_1", (POST, OPENING_W, TOP_RAIL), (face_x, 0.0, top_z)),
        ("top_rail_2", (OPENING_W, POST, TOP_RAIL), (0.0, -face_y, top_z)),
        ("top_rail_3", (POST, OPENING_W, TOP_RAIL), (-face_x, 0.0, top_z)),
        ("bottom_rail_0", (OPENING_W, POST, BOTTOM_RAIL), (0.0, face_y, bottom_z)),
        ("bottom_rail_1", (POST, OPENING_W, BOTTOM_RAIL), (face_x, 0.0, bottom_z)),
        ("bottom_rail_2", (OPENING_W, POST, BOTTOM_RAIL), (0.0, -face_y, bottom_z)),
        ("bottom_rail_3", (POST, OPENING_W, BOTTOM_RAIL), (-face_x, 0.0, bottom_z)),
    )
    for name, size, xyz in rail_specs:
        body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material="frame_metal",
            name=name,
        )

    roof_z = HOUSING_H / 2.0 + ROOF_MAIN / 2.0
    crown_z = HOUSING_H / 2.0 + ROOF_MAIN + ROOF_CROWN / 2.0
    stem_z = HOUSING_H / 2.0 + ROOF_MAIN + ROOF_CROWN + STEM_LEN / 2.0
    canopy_z = HOUSING_H / 2.0 + ROOF_MAIN + ROOF_CROWN + STEM_LEN + CANOPY_H / 2.0

    body.visual(
        Box((HOUSING_W + 0.038, HOUSING_D + 0.038, ROOF_MAIN)),
        origin=Origin(xyz=(0.0, 0.0, roof_z)),
        material="frame_metal",
        name="roof",
    )
    body.visual(
        Box((HOUSING_W - 0.040, HOUSING_D - 0.040, ROOF_CROWN)),
        origin=Origin(xyz=(0.0, 0.0, crown_z)),
        material="frame_trim",
        name="roof_crown",
    )
    body.visual(
        Cylinder(radius=0.018, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, stem_z)),
        material="frame_metal",
        name="stem",
    )
    body.visual(
        Box((CANOPY_W, CANOPY_W, CANOPY_H)),
        origin=Origin(xyz=(0.0, 0.0, canopy_z)),
        material="frame_metal",
        name="canopy",
    )

    cap_z = -HOUSING_H / 2.0 - BOTTOM_CAP_H / 2.0
    finial_stem_z = -HOUSING_H / 2.0 - BOTTOM_CAP_H - FINIAL_STEM / 2.0
    finial_ball_z = -HOUSING_H / 2.0 - BOTTOM_CAP_H - FINIAL_STEM - FINIAL_BALL
    body.visual(
        Box((BOTTOM_CAP_W, BOTTOM_CAP_W, BOTTOM_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, cap_z)),
        material="frame_metal",
        name="bottom_cap",
    )
    body.visual(
        Cylinder(radius=0.008, length=FINIAL_STEM),
        origin=Origin(xyz=(0.0, 0.0, finial_stem_z)),
        material="frame_trim",
        name="finial_stem",
    )
    body.visual(
        Sphere(radius=FINIAL_BALL),
        origin=Origin(xyz=(0.0, 0.0, finial_ball_z)),
        material="frame_trim",
        name="finial_ball",
    )

    socket_z = HOUSING_H / 2.0 - SOCKET_LEN / 2.0
    body.visual(
        Cylinder(radius=SOCKET_RADIUS, length=SOCKET_LEN),
        origin=Origin(xyz=(0.0, 0.0, socket_z)),
        material="lamp_metal",
        name="socket",
    )
    body.visual(
        Sphere(radius=BULB_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, BULB_CENTER_Z)),
        material="lamp_glow",
        name="bulb",
    )


def _add_door_visuals(door) -> None:
    door.visual(
        Box((DOOR_BAR, DOOR_DEPTH, DOOR_H)),
        origin=Origin(xyz=(-DOOR_BAR / 2.0, 0.0, 0.0)),
        material="frame_metal",
        name="hinge_stile",
    )
    door.visual(
        Box((DOOR_BAR, DOOR_DEPTH, DOOR_H)),
        origin=Origin(xyz=(-(DOOR_W - DOOR_BAR / 2.0), 0.0, 0.0)),
        material="frame_metal",
        name="free_stile",
    )
    door.visual(
        Box((DOOR_W, DOOR_DEPTH, DOOR_BAR)),
        origin=Origin(xyz=(-DOOR_W / 2.0, 0.0, DOOR_H / 2.0 - DOOR_BAR / 2.0)),
        material="frame_metal",
        name="top_rail",
    )
    door.visual(
        Box((DOOR_W, DOOR_DEPTH, DOOR_BAR)),
        origin=Origin(xyz=(-DOOR_W / 2.0, 0.0, -DOOR_H / 2.0 + DOOR_BAR / 2.0)),
        material="frame_metal",
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_W - 2.0 * DOOR_BAR + 0.004, GLASS_DEPTH, DOOR_H - 2.0 * DOOR_BAR + 0.004)),
        origin=Origin(xyz=(-DOOR_W / 2.0, 0.0, 0.0)),
        material="glass",
        name="pane",
    )

    hinge_positions = (DOOR_H * 0.32, 0.0, -DOOR_H * 0.32)
    for index, z_pos in enumerate(hinge_positions):
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_SEGMENT),
            origin=Origin(xyz=(0.0, DOOR_DEPTH / 2.0 - HINGE_RADIUS, z_pos)),
            material="frame_trim",
            name=f"hinge_barrel_{index}",
        )

    door.visual(
        Box((0.008, 0.005, 0.040)),
        origin=Origin(xyz=(-(DOOR_W - DOOR_BAR / 2.0), DOOR_DEPTH / 2.0 + 0.0015, 0.0)),
        material="frame_trim",
        name="latch",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) / 2.0 for index in range(3))


def _axis_value(vec, axis_name: str) -> float | None:
    if vec is None:
        return None
    index = {"x": 0, "y": 1, "z": 2}[axis_name]
    return vec[index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_carriage_lantern")

    model.material("frame_metal", rgba=(0.17, 0.15, 0.13, 1.0))
    model.material("frame_trim", rgba=(0.32, 0.26, 0.18, 1.0))
    model.material("glass", rgba=(0.86, 0.91, 0.96, 0.30))
    model.material("lamp_metal", rgba=(0.58, 0.49, 0.34, 1.0))
    model.material("lamp_glow", rgba=(0.98, 0.90, 0.68, 0.55))

    body = model.part("body")
    _add_housing(body)

    for spec in DOOR_SPECS:
        door = model.part(spec["door"])
        _add_door_visuals(door)
        model.articulation(
            spec["joint"],
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=spec["origin"], rpy=(0.0, 0.0, spec["yaw"])),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.5,
                lower=0.0,
                upper=1.55,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for spec in DOOR_SPECS:
        door = object_model.get_part(spec["door"])
        joint = object_model.get_articulation(spec["joint"])

        closed_aabb = ctx.part_element_world_aabb(door, elem="free_stile")
        closed_center = _aabb_center(closed_aabb)
        closed_axis = _axis_value(closed_center, spec["axis_name"])
        expected_closed = spec["axis_sign"] * FRAME_FACE

        ctx.check(
            f"{spec['door']} sits within its lantern face",
            closed_axis is not None and abs(closed_axis - expected_closed) <= 0.010,
            details=f"free_stile_axis={closed_axis}, expected={expected_closed}",
        )

        with ctx.pose({joint: DOOR_OPEN}):
            open_aabb = ctx.part_element_world_aabb(door, elem="free_stile")
            open_center = _aabb_center(open_aabb)

        open_axis = _axis_value(open_center, spec["axis_name"])
        opens_outward = (
            closed_axis is not None
            and open_axis is not None
            and spec["axis_sign"] * (open_axis - closed_axis) >= 0.050
        )
        ctx.check(
            f"{spec['door']} opens outward",
            opens_outward,
            details=f"closed_axis={closed_axis}, open_axis={open_axis}",
        )

    return ctx.report()


object_model = build_object_model()
