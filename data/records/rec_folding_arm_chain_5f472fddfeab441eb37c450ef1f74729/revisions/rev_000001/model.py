from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


LINK_LENGTH = 0.38
LINK_WIDTH = 0.090
LINK_THICKNESS = 0.028
LAYER_STEP = 0.050
PIN_RADIUS = 0.014
PIN_HOLE_RADIUS = 0.026


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (cx + radius * cos(2.0 * pi * index / segments), cy + radius * sin(2.0 * pi * index / segments))
        for index in range(segments)
    ]


def _link_mesh(name: str):
    """A single flat boxed arm link with rounded pivot eyes and through holes."""
    outer = _translate_profile(
        rounded_rect_profile(
            LINK_LENGTH + LINK_WIDTH,
            LINK_WIDTH,
            LINK_WIDTH * 0.5,
            corner_segments=14,
        ),
        LINK_LENGTH * 0.5,
        0.0,
    )
    center_slot = _translate_profile(
        rounded_rect_profile(
            LINK_LENGTH * 0.46,
            LINK_WIDTH * 0.34,
            LINK_WIDTH * 0.13,
            corner_segments=8,
        ),
        LINK_LENGTH * 0.5,
        0.0,
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [
            _circle_profile(PIN_HOLE_RADIUS, center=(0.0, 0.0), segments=48),
            _circle_profile(PIN_HOLE_RADIUS, center=(LINK_LENGTH, 0.0), segments=48),
            center_slot,
        ],
        LINK_THICKNESS,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def _add_boxed_link_visuals(
    part,
    *,
    mesh,
    layer_z: float,
    body_material,
    rib_material,
    pin_material,
    distal_pin: bool,
    terminal_pin: bool = False,
) -> None:
    part.visual(
        mesh,
        origin=Origin(xyz=(0.0, 0.0, layer_z)),
        material=body_material,
        name="main_plate",
    )
    rail_z = layer_z + LINK_THICKNESS * 0.5 + 0.004
    for y, visual_name in ((LINK_WIDTH * 0.39, "outer_rail"), (-LINK_WIDTH * 0.39, "inner_rail")):
        part.visual(
            Box((LINK_LENGTH * 0.70, 0.012, 0.012)),
            origin=Origin(xyz=(LINK_LENGTH * 0.5, y, rail_z)),
            material=rib_material,
            name=visual_name,
        )

    if distal_pin:
        # This pin is owned by the lower link and passes through the next link's
        # generous pivot hole; the raised collars make it visibly retained while
        # staying below the next link layer.
        pin_center_z = layer_z + LAYER_STEP * 0.5
        pin_length = LAYER_STEP + LINK_THICKNESS * 0.82
        part.visual(
            Cylinder(radius=PIN_RADIUS, length=pin_length),
            origin=Origin(xyz=(LINK_LENGTH, 0.0, pin_center_z)),
            material=pin_material,
            name="distal_pin",
        )
        for dz, collar_name in ((LINK_THICKNESS * 0.5 + 0.002, "upper_collar"), (-LINK_THICKNESS * 0.5 - 0.002, "lower_collar")):
            part.visual(
                Cylinder(radius=0.035, length=0.006),
                origin=Origin(xyz=(LINK_LENGTH, 0.0, layer_z + dz)),
                material=pin_material,
                name=collar_name,
            )

    if terminal_pin:
        part.visual(
            Cylinder(radius=PIN_RADIUS, length=LINK_THICKNESS * 1.45),
            origin=Origin(xyz=(LINK_LENGTH, 0.0, layer_z)),
            material=pin_material,
            name="end_pin",
        )
        for dz, collar_name in ((LINK_THICKNESS * 0.5 + 0.002, "end_upper_collar"), (-LINK_THICKNESS * 0.5 - 0.002, "end_lower_collar")):
            part.visual(
                Cylinder(radius=0.035, length=0.006),
                origin=Origin(xyz=(LINK_LENGTH, 0.0, layer_z + dz)),
                material=pin_material,
                name=collar_name,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_chain")

    bracket_paint = model.material("bracket_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    link_yellow = model.material("link_yellow", rgba=(0.92, 0.67, 0.12, 1.0))
    link_amber = model.material("link_amber", rgba=(0.84, 0.50, 0.10, 1.0))
    dark_ribs = model.material("dark_ribs", rgba=(0.22, 0.23, 0.24, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.045, 0.045, 0.045, 1.0))

    link_mesh = _link_mesh("boxed_arm_link")

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.030, 0.220, 0.170)),
        origin=Origin(xyz=(-0.086, 0.0, 0.0)),
        material=bracket_paint,
        name="wall_plate",
    )
    bracket.visual(
        Box((0.118, 0.150, 0.012)),
        origin=Origin(xyz=(-0.025, 0.0, -0.030)),
        material=bracket_paint,
        name="lower_cheek",
    )
    bracket.visual(
        Box((0.118, 0.150, 0.012)),
        origin=Origin(xyz=(-0.025, 0.0, 0.030)),
        material=bracket_paint,
        name="upper_cheek",
    )
    bracket.visual(
        Box((0.030, 0.150, 0.070)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0)),
        material=bracket_paint,
        name="back_web",
    )
    bracket.visual(
        Cylinder(radius=PIN_RADIUS, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_steel,
        name="base_pin",
    )
    for z, visual_name in ((0.020, "top_washer"), (-0.020, "bottom_washer")):
        bracket.visual(
            Cylinder(radius=0.037, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=pin_steel,
            name=visual_name,
        )
    for y in (-0.075, 0.075):
        for z in (-0.055, 0.055):
            bracket.visual(
                Cylinder(radius=0.012, length=0.007),
                origin=Origin(xyz=(-0.102, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=pin_steel,
                name=f"mount_bolt_{len(bracket.visuals)}",
            )
    bracket.inertial = Inertial.from_geometry(
        Box((0.13, 0.22, 0.17)),
        mass=1.2,
        origin=Origin(xyz=(-0.046, 0.0, 0.0)),
    )

    link_0 = model.part("link_0")
    _add_boxed_link_visuals(
        link_0,
        mesh=link_mesh,
        layer_z=0.0,
        body_material=link_yellow,
        rib_material=dark_ribs,
        pin_material=pin_steel,
        distal_pin=True,
    )
    link_0.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH + LINK_WIDTH, LINK_WIDTH, LINK_THICKNESS)),
        mass=0.55,
        origin=Origin(xyz=(LINK_LENGTH * 0.5, 0.0, 0.0)),
    )

    link_1 = model.part("link_1")
    _add_boxed_link_visuals(
        link_1,
        mesh=link_mesh,
        layer_z=LAYER_STEP,
        body_material=link_amber,
        rib_material=dark_ribs,
        pin_material=pin_steel,
        distal_pin=True,
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH + LINK_WIDTH, LINK_WIDTH, LINK_THICKNESS)),
        mass=0.52,
        origin=Origin(xyz=(LINK_LENGTH * 0.5, 0.0, LAYER_STEP)),
    )

    link_2 = model.part("link_2")
    _add_boxed_link_visuals(
        link_2,
        mesh=link_mesh,
        layer_z=LAYER_STEP * 2.0,
        body_material=link_yellow,
        rib_material=dark_ribs,
        pin_material=pin_steel,
        distal_pin=False,
        terminal_pin=True,
    )
    link_2.visual(
        Box((0.048, 0.074, 0.014)),
        origin=Origin(xyz=(LINK_LENGTH + 0.020, 0.0, LAYER_STEP * 2.0 + 0.025)),
        material=black_rubber,
        name="end_bumper",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH + LINK_WIDTH, LINK_WIDTH, LINK_THICKNESS)),
        mass=0.50,
        origin=Origin(xyz=(LINK_LENGTH * 0.5, 0.0, LAYER_STEP * 2.0)),
    )

    model.articulation(
        "bracket_to_link_0",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=0.0, upper=pi),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-pi, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")

    base_joint = object_model.get_articulation("bracket_to_link_0")
    elbow_joint = object_model.get_articulation("link_0_to_link_1")
    wrist_joint = object_model.get_articulation("link_1_to_link_2")

    ctx.allow_overlap(
        bracket,
        link_0,
        elem_a="base_pin",
        elem_b="main_plate",
        reason="The bracket pin is intentionally captured through the first link's pivot eye.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="main_plate",
        reason="The first link's distal pin intentionally passes through the second link's pivot eye.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="distal_pin",
        elem_b="main_plate",
        reason="The second link's distal pin intentionally passes through the third link's pivot eye.",
    )

    joints = (base_joint, elbow_joint, wrist_joint)
    ctx.check(
        "three serial revolute joints",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints),
        details=f"joint types={[joint.articulation_type for joint in joints]}",
    )
    ctx.check(
        "parallel vertical hinge axes",
        all(tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0) for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    for carrier, follower, pin_name, check_name in (
        (bracket, link_0, "base_pin", "base pin retained in first link eye"),
        (link_0, link_1, "distal_pin", "first distal pin retained in second link eye"),
        (link_1, link_2, "distal_pin", "second distal pin retained in third link eye"),
    ):
        ctx.expect_within(
            carrier,
            follower,
            axes="xy",
            inner_elem=pin_name,
            outer_elem="main_plate",
            margin=0.0,
            name=f"{check_name} xy",
        )
        ctx.expect_overlap(
            carrier,
            follower,
            axes="z",
            min_overlap=LINK_THICKNESS * 0.70,
            elem_a=pin_name,
            elem_b="main_plate",
            name=f"{check_name} thickness",
        )

    with ctx.pose({base_joint: 0.0, elbow_joint: 0.0, wrist_joint: 0.0}):
        link_2_aabb = ctx.part_world_aabb(link_2)
        ctx.check(
            "straight pose reaches outward",
            link_2_aabb is not None and link_2_aabb[1][0] > (3.0 * LINK_LENGTH),
            details=f"link_2_aabb={link_2_aabb}",
        )

    with ctx.pose({base_joint: 0.0, elbow_joint: pi, wrist_joint: -pi}):
        folded_pos = ctx.part_world_position(link_2)
        folded_aabb = ctx.part_world_aabb(link_2)
        ctx.check(
            "folded pose returns distal link to bracket",
            folded_pos is not None and abs(folded_pos[0]) < 0.025 and abs(folded_pos[1]) < 0.025,
            details=f"folded_pos={folded_pos}",
        )
        ctx.check(
            "folded pose stays compact",
            folded_aabb is not None and (folded_aabb[1][0] - folded_aabb[0][0]) < 0.52,
            details=f"folded_aabb={folded_aabb}",
        )
        ctx.expect_gap(
            link_1,
            link_0,
            axis="z",
            min_gap=0.015,
            positive_elem="main_plate",
            negative_elem="main_plate",
            name="folded middle link stacks above first link",
        )
        ctx.expect_gap(
            link_2,
            link_1,
            axis="z",
            min_gap=0.015,
            positive_elem="main_plate",
            negative_elem="main_plate",
            name="folded outer link stacks above middle link",
        )

    return ctx.report()


object_model = build_object_model()
