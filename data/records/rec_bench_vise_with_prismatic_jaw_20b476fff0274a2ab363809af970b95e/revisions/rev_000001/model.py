from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _jaw_with_round_holes(
    *,
    thickness_x: float,
    width_y: float,
    height_z: float,
    hole_specs: tuple[tuple[float, float, float], ...],
    chamfer: float = 0.003,
):
    """CadQuery jaw blank centered at the origin, with through-holes along X.

    hole_specs are (local_y, local_z, radius).
    """
    body = cq.Workplane("XY").box(thickness_x, width_y, height_z)
    for y, z, radius in hole_specs:
        cutter = cq.Workplane("YZ").center(y, z).circle(radius).extrude(
            thickness_x * 3.0, both=True
        )
        body = body.cut(cutter)
    if chamfer > 0.0:
        return body.edges().chamfer(chamfer)
    return body


def _tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    tube = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(tube, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_release_woodworking_vise")

    wood = model.material("oiled_beech", rgba=(0.72, 0.48, 0.25, 1.0))
    endgrain = model.material("dark_endgrain", rgba=(0.45, 0.27, 0.12, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    cast_iron = model.material("blue_gray_cast_iron", rgba=(0.16, 0.19, 0.22, 1.0))
    brass = model.material("worn_brass", rgba=(0.76, 0.58, 0.24, 1.0))

    jaw_holes = (
        (-0.20, -0.055, 0.025),
        (0.20, -0.055, 0.025),
        (0.0, -0.030, 0.030),
    )

    frame = model.part("frame")
    rear_jaw_mesh = mesh_from_cadquery(
        _jaw_with_round_holes(
            thickness_x=0.050,
            width_y=0.580,
            height_z=0.180,
            hole_specs=jaw_holes,
        ),
        "rear_jaw",
        tolerance=0.0008,
    )
    frame.visual(
        rear_jaw_mesh,
        origin=Origin(xyz=(0.025, 0.0, 0.160)),
        material=wood,
        name="rear_jaw",
    )
    # Bench-edge mounting mass behind the fixed jaw.
    frame.visual(
        Box((0.230, 0.650, 0.055)),
        origin=Origin(xyz=(0.100, 0.0, 0.275)),
        material=wood,
        name="bench_top",
    )
    backplate_holes = (
        (-0.20, -0.065, 0.025),
        (0.20, -0.065, 0.025),
        (0.0, -0.040, 0.030),
    )
    frame.visual(
        mesh_from_cadquery(
            _jaw_with_round_holes(
                thickness_x=0.030,
                width_y=0.620,
                height_z=0.170,
                hole_specs=backplate_holes,
                chamfer=0.0,
            ),
            "rear_backplate",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=(0.105, 0.0, 0.170)),
        material=cast_iron,
        name="rear_backplate",
    )

    # Wood grain strips are slightly proud and embedded so they read as applied
    # facing detail without becoming disconnected islands.
    for i, z in enumerate((0.105, 0.150, 0.205)):
        frame.visual(
            Box((0.003, 0.525, 0.006)),
            origin=Origin(xyz=(-0.001, 0.0, z)),
            material=endgrain,
            name=f"rear_grain_{i}",
        )

    # Twin guide bars fixed to the rear casting.
    for i, y in enumerate((-0.20, 0.20)):
        frame.visual(
            Cylinder(radius=0.018, length=0.440),
            origin=Origin(
                xyz=(-0.120, y, 0.105),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"guide_bar_{i}",
        )
        frame.visual(
            Cylinder(radius=0.031, length=0.048),
            origin=Origin(
                xyz=(0.020, y, 0.105),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=cast_iron,
            name=f"bar_socket_{i}",
        )

    frame.visual(
        _tube_mesh("screw_nut_tube", outer_radius=0.044, inner_radius=0.028, length=0.120),
        origin=Origin(xyz=(0.060, 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="screw_nut",
    )
    frame.visual(
        Box((0.090, 0.120, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, 0.068)),
        material=cast_iron,
        name="lower_tie_plate",
    )

    front_jaw = model.part("front_jaw")
    front_jaw_mesh = mesh_from_cadquery(
        _jaw_with_round_holes(
            thickness_x=0.080,
            width_y=0.580,
            height_z=0.180,
            hole_specs=jaw_holes,
        ),
        "front_jaw_body",
        tolerance=0.0008,
    )
    front_jaw.visual(front_jaw_mesh, material=wood, name="jaw_body")
    for i, z in enumerate((-0.050, 0.010, 0.070)):
        front_jaw.visual(
            Box((0.003, 0.520, 0.006)),
            origin=Origin(xyz=(-0.041, 0.0, z)),
            material=endgrain,
            name=f"front_grain_{i}",
        )
    # Quick-release lever clevis lugs and their retained pivot pin.
    for i, y in enumerate((-0.044, 0.044)):
        front_jaw.visual(
            Box((0.034, 0.020, 0.052)),
            origin=Origin(xyz=(-0.053, y, -0.092)),
            material=cast_iron,
            name=f"lever_lug_{i}",
        )
    front_jaw.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(-0.060, 0.0, -0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lever_pin",
    )

    jaw_slide = model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=front_jaw,
        origin=Origin(xyz=(-0.046, 0.0, 0.160)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    jaw_slide.meta["description"] = "front jaw slides outward on the two fixed guide bars"

    screw = model.part("screw")
    screw.visual(
        Cylinder(radius=0.017, length=0.500),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="screw_shaft",
    )
    for i, x in enumerate([(-0.175 + 0.025 * n) for n in range(16)]):
        screw.visual(
            Cylinder(radius=0.0215, length=0.005),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"thread_crest_{i}",
        )
    screw.visual(
        Cylinder(radius=0.039, length=0.034),
        origin=Origin(xyz=(-0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="front_collar",
    )
    screw.visual(
        Cylinder(radius=0.012, length=0.240),
        origin=Origin(xyz=(-0.214, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="handle_bar",
    )
    for i, y in enumerate((-0.132, 0.132)):
        screw.visual(
            Sphere(radius=0.027),
            origin=Origin(xyz=(-0.214, y, 0.0)),
            material=wood,
            name=f"handle_knob_{i}",
        )

    model.articulation(
        "screw_turn",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=screw,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=5.0),
    )

    release_lever = model.part("release_lever")
    release_lever.visual(
        Cylinder(radius=0.014, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_hub",
    )
    release_lever.visual(
        Box((0.020, 0.030, 0.154)),
        origin=Origin(xyz=(-0.014, 0.0, -0.070), rpy=(0.0, -0.10, 0.0)),
        material=dark_steel,
        name="lever_blade",
    )
    release_lever.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(-0.033, 0.0, -0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="release_grip",
    )
    release_lever.visual(
        Box((0.024, 0.018, 0.030)),
        origin=Origin(xyz=(0.012, 0.0, -0.022)),
        material=dark_steel,
        name="release_pawl",
    )

    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=front_jaw,
        child=release_lever,
        origin=Origin(xyz=(-0.060, 0.0, -0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_jaw = object_model.get_part("front_jaw")
    screw = object_model.get_part("screw")
    release_lever = object_model.get_part("release_lever")
    jaw_slide = object_model.get_articulation("jaw_slide")
    screw_turn = object_model.get_articulation("screw_turn")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.allow_overlap(
        front_jaw,
        release_lever,
        elem_a="lever_pin",
        elem_b="pivot_hub",
        reason="The steel pivot pin is intentionally captured inside the rapid-release lever hub.",
    )
    ctx.expect_overlap(
        front_jaw,
        release_lever,
        axes="yz",
        elem_a="lever_pin",
        elem_b="pivot_hub",
        min_overlap=0.010,
        name="release lever hub is retained by the pivot pin",
    )

    ctx.expect_gap(
        frame,
        front_jaw,
        axis="x",
        positive_elem="rear_jaw",
        negative_elem="jaw_body",
        min_gap=0.004,
        max_gap=0.010,
        name="closed vise leaves a narrow jaw clearance",
    )
    for bar_name in ("guide_bar_0", "guide_bar_1"):
        ctx.expect_within(
            frame,
            front_jaw,
            axes="yz",
            inner_elem=bar_name,
            outer_elem="jaw_body",
            margin=0.0,
            name=f"{bar_name} passes through the moving jaw guide hole",
        )
        ctx.expect_overlap(
            frame,
            front_jaw,
            axes="x",
            elem_a=bar_name,
            elem_b="jaw_body",
            min_overlap=0.065,
            name=f"{bar_name} retains the moving jaw at rest",
        )

    rest_pos = ctx.part_world_position(front_jaw)
    with ctx.pose({jaw_slide: 0.180}):
        extended_pos = ctx.part_world_position(front_jaw)
        ctx.expect_gap(
            frame,
            front_jaw,
            axis="x",
            positive_elem="rear_jaw",
            negative_elem="jaw_body",
            min_gap=0.175,
            max_gap=0.195,
            name="front jaw opens on the slide",
        )
        for bar_name in ("guide_bar_0", "guide_bar_1"):
            ctx.expect_overlap(
                frame,
                front_jaw,
                axes="x",
                elem_a=bar_name,
                elem_b="jaw_body",
                min_overlap=0.065,
                name=f"{bar_name} still supports the fully opened jaw",
            )
    ctx.check(
        "jaw slide moves outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    bar_rest = ctx.part_element_world_aabb(screw, elem="handle_bar")
    with ctx.pose({screw_turn: math.pi / 2.0}):
        bar_turned = ctx.part_element_world_aabb(screw, elem="handle_bar")
    if bar_rest is not None and bar_turned is not None:
        rest_dy = bar_rest[1][1] - bar_rest[0][1]
        rest_dz = bar_rest[1][2] - bar_rest[0][2]
        turned_dy = bar_turned[1][1] - bar_turned[0][1]
        turned_dz = bar_turned[1][2] - bar_turned[0][2]
        screw_rotates = rest_dy > rest_dz * 2.5 and turned_dz > turned_dy * 2.5
    else:
        screw_rotates = False
    ctx.check(
        "fine adjust screw rotates the T handle",
        screw_rotates,
        details=f"rest_aabb={bar_rest}, turned_aabb={bar_turned}",
    )

    grip_rest = ctx.part_element_world_aabb(release_lever, elem="release_grip")
    with ctx.pose({lever_pivot: 0.75}):
        grip_pulled = ctx.part_element_world_aabb(release_lever, elem="release_grip")
    if grip_rest is not None and grip_pulled is not None:
        rest_x = (grip_rest[0][0] + grip_rest[1][0]) * 0.5
        pulled_x = (grip_pulled[0][0] + grip_pulled[1][0]) * 0.5
        lever_moves = pulled_x < rest_x - 0.030
    else:
        lever_moves = False
    ctx.check(
        "rapid release lever pulls outward",
        lever_moves,
        details=f"rest_aabb={grip_rest}, pulled_aabb={grip_pulled}",
    )

    return ctx.report()


object_model = build_object_model()
