from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PIN_RADIUS = 0.0055
PIN_CLEARANCE_RADIUS = 0.0072


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 40,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * cos(2.0 * pi * index / segments),
            cy + radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _capsule_profile(
    length: float,
    radius: float,
    *,
    segments: int = 20,
) -> list[tuple[float, float]]:
    """Closed 2-D capsule profile in the X/Z side plane."""

    points: list[tuple[float, float]] = []
    # Distal semicircle, bottom tangent to top tangent.
    for index in range(segments + 1):
        angle = -pi / 2.0 + pi * index / segments
        points.append((length + radius * cos(angle), radius * sin(angle)))
    # Proximal semicircle, top tangent back to bottom tangent.
    for index in range(segments + 1):
        angle = pi / 2.0 + pi * index / segments
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _lever_plate_mesh(
    length: float,
    eye_radius: float,
    hole_radius: float,
    thickness: float,
    name: str,
):
    # The mesh starts as an X/Y 2-D plate extruded along local Z, then rotates so
    # the through-bores and pin axes run along the linkage's local Y direction.
    geometry = ExtrudeWithHolesGeometry(
        _capsule_profile(length, eye_radius),
        [
            _circle_profile(0.0, 0.0, hole_radius),
            _circle_profile(length, 0.0, hole_radius),
        ],
        height=thickness,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(geometry, name)


def _terminal_tab_mesh(
    length: float,
    eye_radius: float,
    hole_radius: float,
    thickness: float,
    name: str,
):
    # A shorter output member with a slightly flattened nose so the last piece
    # reads as a compact terminal tab rather than another long link.
    outer = _capsule_profile(length, eye_radius, segments=18)
    geometry = ExtrudeWithHolesGeometry(
        outer,
        [
            _circle_profile(0.0, 0.0, hole_radius),
            _circle_profile(length, 0.0, hole_radius * 0.78, segments=32),
        ],
        height=thickness,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(geometry, name)


def _clevis_cheek_mesh(name: str):
    geometry = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.082, 0.105, 0.010, corner_segments=8),
        [_circle_profile(0.0, 0.0, PIN_CLEARANCE_RADIUS)],
        height=0.010,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_clevis_lever_linkage")

    parkerized_steel = model.material("parkerized_steel", rgba=(0.17, 0.18, 0.18, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.48, 0.49, 0.47, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.03, 0.035, 0.04, 1.0))
    brass_bushing = model.material("brass_bushing", rgba=(0.86, 0.64, 0.28, 1.0))

    root = model.part("root_clevis")
    root.visual(
        Box((0.112, 0.072, 0.018)),
        origin=Origin(xyz=(-0.005, 0.0, -0.057)),
        material=parkerized_steel,
        name="mount_base",
    )
    root.visual(
        Box((0.020, 0.058, 0.062)),
        origin=Origin(xyz=(-0.041, 0.0, -0.022)),
        material=parkerized_steel,
        name="rear_bridge",
    )
    root.visual(
        _clevis_cheek_mesh("clevis_cheek_neg"),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=parkerized_steel,
        name="cheek_neg",
    )
    root.visual(
        _clevis_cheek_mesh("clevis_cheek_pos"),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=parkerized_steel,
        name="cheek_pos",
    )
    root.visual(
        Cylinder(radius=PIN_RADIUS, length=0.064),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_steel,
        name="root_pin",
    )
    root.visual(
        Cylinder(radius=0.0125, length=0.004),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_steel,
        name="root_pin_head",
    )
    root.visual(
        Cylinder(radius=0.0105, length=0.003),
        origin=Origin(xyz=(0.0, 0.0335, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="root_pin_clip",
    )
    for index, bolt_x in enumerate((-0.036, 0.030)):
        root.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(bolt_x, 0.0, -0.046)),
            material=black_oxide,
            name=f"base_bolt_{index}",
        )

    lever_0 = model.part("lever_0")
    lever_0.visual(
        _lever_plate_mesh(0.280, 0.025, PIN_CLEARANCE_RADIUS, 0.010, "lever_0_plate"),
        material=warm_steel,
        name="lever_plate",
    )
    lever_0.visual(
        Cylinder(radius=0.0155, length=0.004),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass_bushing,
        name="root_bushing",
    )
    lever_0.visual(
        Cylinder(radius=PIN_RADIUS, length=0.054),
        origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_steel,
        name="middle_pin",
    )
    lever_0.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.280, -0.029, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="middle_pin_head",
    )
    lever_0.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.280, 0.029, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="middle_pin_clip",
    )

    lever_1 = model.part("lever_1")
    lever_1.visual(
        _lever_plate_mesh(0.200, 0.022, PIN_CLEARANCE_RADIUS, 0.010, "lever_1_plate"),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=warm_steel,
        name="lever_plate",
    )
    lever_1.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass_bushing,
        name="middle_bushing",
    )
    lever_1.visual(
        Cylinder(radius=PIN_RADIUS, length=0.060),
        origin=Origin(xyz=(0.200, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_steel,
        name="terminal_pin",
    )
    lever_1.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.200, -0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="terminal_pin_head",
    )
    lever_1.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.200, 0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="terminal_pin_clip",
    )

    terminal_tab = model.part("terminal_tab")
    terminal_tab.visual(
        _terminal_tab_mesh(0.125, 0.019, PIN_CLEARANCE_RADIUS, 0.010, "terminal_tab_plate"),
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        material=warm_steel,
        name="tab_plate",
    )
    terminal_tab.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass_bushing,
        name="terminal_bushing",
    )
    terminal_tab.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.125, -0.018, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="output_stud",
    )

    pin_limits = MotionLimits(effort=18.0, velocity=3.0, lower=-0.85, upper=0.85)
    model.articulation(
        "root_pin",
        ArticulationType.REVOLUTE,
        parent=root,
        child=lever_0,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=pin_limits,
    )
    model.articulation(
        "middle_pin",
        ArticulationType.REVOLUTE,
        parent=lever_0,
        child=lever_1,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=3.0, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "terminal_pin",
        ArticulationType.REVOLUTE,
        parent=lever_1,
        child=terminal_tab,
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=-1.30, upper=1.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_clevis")
    lever_0 = object_model.get_part("lever_0")
    lever_1 = object_model.get_part("lever_1")
    terminal_tab = object_model.get_part("terminal_tab")
    root_joint = object_model.get_articulation("root_pin")
    middle_joint = object_model.get_articulation("middle_pin")
    terminal_joint = object_model.get_articulation("terminal_pin")

    ctx.allow_overlap(
        root,
        lever_0,
        elem_a="root_pin",
        elem_b="lever_plate",
        reason="The root pin shaft is intentionally captured through the first lever eye.",
    )
    ctx.allow_overlap(
        lever_0,
        lever_1,
        elem_a="middle_pin",
        elem_b="lever_plate",
        reason="The middle pin shaft is intentionally captured through the offset second lever eye.",
    )
    ctx.allow_overlap(
        lever_1,
        terminal_tab,
        elem_a="terminal_pin",
        elem_b="tab_plate",
        reason="The terminal pin shaft is intentionally captured through the compact output tab eye.",
    )

    revolute_joints = [
        articulation
        for articulation in object_model.articulations
        if articulation.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "three serial revolute pin joints",
        len(revolute_joints) == 3,
        details=f"found {len(revolute_joints)} revolute joints",
    )
    ctx.check(
        "pin axes are parallel",
        all(
            abs(joint.axis[0]) < 1.0e-6
            and abs(abs(joint.axis[1]) - 1.0) < 1.0e-6
            and abs(joint.axis[2]) < 1.0e-6
            for joint in revolute_joints
        ),
        details=f"axes={[joint.axis for joint in revolute_joints]}",
    )

    with ctx.pose({root_joint: 0.0, middle_joint: 0.0, terminal_joint: 0.0}):
        ctx.expect_gap(
            root,
            lever_0,
            axis="y",
            positive_elem="cheek_pos",
            negative_elem="lever_plate",
            min_gap=0.006,
            name="first lever clears positive clevis cheek",
        )
        ctx.expect_gap(
            lever_0,
            root,
            axis="y",
            positive_elem="lever_plate",
            negative_elem="cheek_neg",
            min_gap=0.006,
            name="first lever clears negative clevis cheek",
        )
        ctx.expect_overlap(
            lever_0,
            root,
            axes="xz",
            elem_a="lever_plate",
            elem_b="root_pin",
            min_overlap=0.006,
            name="root pin passes through first lever eye",
        )
        ctx.expect_overlap(
            lever_1,
            lever_0,
            axes="xz",
            elem_a="lever_plate",
            elem_b="middle_pin",
            min_overlap=0.006,
            name="middle pin passes through offset second lever eye",
        )
        ctx.expect_overlap(
            terminal_tab,
            lever_1,
            axes="xz",
            elem_a="tab_plate",
            elem_b="terminal_pin",
            min_overlap=0.006,
            name="terminal pin passes through compact tab eye",
        )
        ctx.expect_gap(
            lever_1,
            lever_0,
            axis="y",
            positive_elem="lever_plate",
            negative_elem="lever_plate",
            min_gap=0.005,
            name="offset middle lever clears first lever",
        )
        ctx.expect_gap(
            lever_0,
            terminal_tab,
            axis="y",
            positive_elem="lever_plate",
            negative_elem="tab_plate",
            min_gap=0.006,
            name="terminal tab sits on the opposite offset side",
        )

        origin_0 = ctx.part_world_position(lever_0)
        origin_1 = ctx.part_world_position(lever_1)
        origin_2 = ctx.part_world_position(terminal_tab)
        ctx.check(
            "serial joint origins step forward",
            origin_0 is not None
            and origin_1 is not None
            and origin_2 is not None
            and origin_1[0] > origin_0[0] + 0.25
            and origin_2[0] > origin_1[0] + 0.17,
            details=f"origins={origin_0}, {origin_1}, {origin_2}",
        )

    rest_aabb = ctx.part_element_world_aabb(lever_0, elem="lever_plate")
    with ctx.pose({root_joint: 0.45, middle_joint: 0.0, terminal_joint: 0.0}):
        lifted_aabb = ctx.part_element_world_aabb(lever_0, elem="lever_plate")
    ctx.check(
        "positive root rotation lifts the long lever",
        rest_aabb is not None
        and lifted_aabb is not None
        and lifted_aabb[1][2] > rest_aabb[1][2] + 0.070,
        details=f"rest={rest_aabb}, lifted={lifted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
