from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _slot_profile(
    center_x: float,
    center_z: float,
    *,
    length: float,
    width: float,
    segments: int = 10,
) -> list[tuple[float, float]]:
    """Rounded vertical slot profile in an X/Z plate drawing."""
    radius = width * 0.5
    half_straight = max(0.0, (length - width) * 0.5)
    top_z = center_z + half_straight
    bottom_z = center_z - half_straight
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        angle = (math.pi * index) / segments
        points.append((center_x + radius * math.cos(angle), top_z + radius * math.sin(angle)))
    for index in range(segments + 1):
        angle = math.pi + (math.pi * index) / segments
        points.append(
            (center_x + radius * math.cos(angle), bottom_z + radius * math.sin(angle))
        )
    return points


def _yoke_arm_geometry(y_offset: float):
    outer_profile = [
        (-0.185, 0.030),
        (0.195, 0.030),
        (0.230, 0.105),
        (0.230, 0.565),
        (0.145, 0.655),
        (-0.125, 0.655),
        (-0.185, 0.570),
    ]
    trunnion_slot = _slot_profile(0.020, 0.420, length=0.220, width=0.082)
    arm = ExtrudeWithHolesGeometry(
        outer_profile,
        [trunnion_slot],
        0.055,
        center=True,
    )
    # Profile XY becomes world XZ; extrusion thickness becomes world Y.
    arm.rotate_x(math.pi / 2.0).translate(0.0, y_offset, 0.0)
    return arm


def _bearing_sleeve_geometry():
    return LatheGeometry.from_shell_profiles(
        [(0.145, 0.000), (0.145, 0.170)],
        [(0.055, 0.000), (0.055, 0.170)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _barrel_geometry():
    # Lathe profile is authored along local +Z, then rotated so its axis is +X.
    barrel = LatheGeometry(
        [
            (0.000, -0.540),
            (0.035, -0.535),
            (0.064, -0.515),
            (0.081, -0.485),
            (0.075, -0.445),
            (0.045, -0.415),
            (0.055, -0.385),
            (0.105, -0.355),
            (0.130, -0.325),
            (0.132, -0.260),
            (0.118, -0.235),
            (0.118, -0.155),
            (0.136, -0.145),
            (0.136, -0.105),
            (0.113, -0.095),
            (0.108, 0.120),
            (0.123, 0.130),
            (0.123, 0.175),
            (0.104, 0.190),
            (0.092, 0.520),
            (0.084, 0.650),
            (0.100, 0.675),
            (0.125, 0.715),
            (0.137, 0.760),
            (0.137, 0.820),
            # The return path forms a short visible bore rather than a flat cap.
            (0.055, 0.820),
            (0.055, 0.645),
            (0.000, 0.605),
            (0.000, -0.540),
        ],
        segments=88,
    )
    barrel.rotate_y(math.pi / 2.0)
    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_swivel_deck_cannon")

    black_iron = model.material("black_iron", rgba=(0.035, 0.037, 0.040, 1.0))
    rubbed_iron = model.material("rubbed_iron", rgba=(0.18, 0.18, 0.17, 1.0))
    dark_bore = model.material("dark_bore", rgba=(0.002, 0.002, 0.002, 1.0))
    worn_edges = model.material("worn_edges", rgba=(0.33, 0.31, 0.27, 1.0))

    base_post = model.part("post")
    base_post.visual(
        Cylinder(radius=0.305, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=black_iron,
        name="flanged_base",
    )
    base_post.visual(
        Cylinder(radius=0.210, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=rubbed_iron,
        name="raised_flange",
    )
    base_post.visual(
        Cylinder(radius=0.090, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=black_iron,
        name="iron_column",
    )
    base_post.visual(
        Cylinder(radius=0.128, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=rubbed_iron,
        name="lower_collar",
    )
    base_post.visual(
        Cylinder(radius=0.132, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=rubbed_iron,
        name="bearing_washer",
    )
    base_post.visual(
        Cylinder(radius=0.045, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.750)),
        material=worn_edges,
        name="post_pin",
    )
    for index in range(6):
        angle = (math.tau * index) / 6.0
        base_post.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(
                xyz=(0.245 * math.cos(angle), 0.245 * math.sin(angle), 0.069)
            ),
            material=worn_edges,
            name=f"base_bolt_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_geometry(_bearing_sleeve_geometry(), "bearing_sleeve"),
        material=black_iron,
        name="bearing_sleeve",
    )
    arm_names = ("slotted_arm_0", "slotted_arm_1")
    boss_names = ("slot_boss_0", "slot_boss_1")
    lug_names = ("side_lug_0", "side_lug_1")
    for side, y_offset in enumerate((-0.160, 0.160)):
        yoke.visual(
            Box((0.340, 0.100, 0.120)),
            origin=Origin(xyz=(0.030, y_offset * 0.72, 0.070)),
            material=black_iron,
            name=lug_names[side],
        )
        yoke.visual(
            mesh_from_geometry(_yoke_arm_geometry(y_offset), arm_names[side]),
            material=black_iron,
            name=arm_names[side],
        )
        yoke.visual(
            Cylinder(radius=0.060, length=0.018),
            origin=Origin(
                xyz=(0.020, y_offset + (0.031 if y_offset > 0 else -0.031), 0.420),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=worn_edges,
            name=boss_names[side],
        )
    yoke.visual(
        Box((0.100, 0.390, 0.075)),
        origin=Origin(xyz=(-0.170, 0.0, 0.135)),
        material=black_iron,
        name="rear_tie",
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_barrel_geometry(), "tapered_barrel"),
        material=black_iron,
        name="tapered_barrel",
    )
    barrel.visual(
        Cylinder(radius=0.028, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edges,
        name="trunnion_pin",
    )
    for side, y_offset in enumerate((-0.214, 0.214)):
        barrel.visual(
            Cylinder(radius=0.046, length=0.025),
            origin=Origin(xyz=(0.0, y_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_edges,
            name=f"trunnion_head_{side}",
        )
    barrel.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.816, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_bore,
        name="muzzle_bore",
    )

    model.articulation(
        "post_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=base_post,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.2),
    )
    model.articulation(
        "yoke_to_barrel",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=barrel,
        origin=Origin(xyz=(0.020, 0.0, 0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=-0.22, upper=0.62),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    yoke = object_model.get_part("yoke")
    barrel = object_model.get_part("barrel")
    swivel = object_model.get_articulation("post_to_yoke")
    elevation = object_model.get_articulation("yoke_to_barrel")

    ctx.allow_overlap(
        "barrel",
        "yoke",
        elem_a="trunnion_pin",
        elem_b="slotted_arm_0",
        reason="The trunnion pin is intentionally represented as captured through the slotted yoke cheek.",
    )
    ctx.allow_overlap(
        "barrel",
        "yoke",
        elem_a="trunnion_pin",
        elem_b="slotted_arm_1",
        reason="The trunnion pin is intentionally represented as captured through the opposite slotted yoke cheek.",
    )
    ctx.allow_overlap(
        "barrel",
        "yoke",
        elem_a="trunnion_pin",
        elem_b="slot_boss_0",
        reason="The trunnion pin intentionally runs inside the cheek boss that reinforces the slot.",
    )
    ctx.allow_overlap(
        "barrel",
        "yoke",
        elem_a="trunnion_pin",
        elem_b="slot_boss_1",
        reason="The trunnion pin intentionally runs inside the opposite cheek boss that reinforces the slot.",
    )
    ctx.expect_contact(
        yoke,
        post,
        elem_a="bearing_sleeve",
        elem_b="bearing_washer",
        contact_tol=0.002,
        name="swivel sleeve rests on post washer",
    )
    ctx.expect_within(
        "barrel",
        "yoke",
        axes="z",
        inner_elem="trunnion_pin",
        outer_elem="slotted_arm_0",
        margin=0.003,
        name="trunnion pin height is captured in the yoke slot",
    )
    ctx.expect_within(
        "barrel",
        "yoke",
        axes="z",
        inner_elem="trunnion_pin",
        outer_elem="slotted_arm_1",
        margin=0.003,
        name="trunnion pin height is captured in the opposite slot",
    )
    ctx.expect_overlap(
        "barrel",
        "yoke",
        axes="y",
        elem_a="trunnion_pin",
        elem_b="slotted_arm_0",
        min_overlap=0.035,
        name="trunnion pin passes through a slotted yoke arm",
    )
    ctx.expect_overlap(
        "barrel",
        "yoke",
        axes="y",
        elem_a="trunnion_pin",
        elem_b="slotted_arm_1",
        min_overlap=0.035,
        name="trunnion pin passes through the opposite yoke arm",
    )
    ctx.expect_overlap(
        "barrel",
        "yoke",
        axes="yz",
        elem_a="trunnion_pin",
        elem_b="slot_boss_0",
        min_overlap=0.018,
        name="trunnion pin is centered through a cheek boss",
    )
    ctx.expect_overlap(
        "barrel",
        "yoke",
        axes="yz",
        elem_a="trunnion_pin",
        elem_b="slot_boss_1",
        min_overlap=0.018,
        name="trunnion pin is centered through the opposite cheek boss",
    )

    rest_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_bore")
    with ctx.pose({elevation: 0.50}):
        raised_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_bore")
    ctx.check(
        "barrel elevates upward in the yoke",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[0][2] > rest_aabb[0][2] + 0.25,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    rest_pos = ctx.part_world_position(yoke)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(yoke)
    ctx.check(
        "fork yoke swivels on the vertical post pin",
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
