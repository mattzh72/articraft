from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _armor_mesh(name: str):
    """Rounded rectangular rubber-armored barrel body, long axis along +X."""
    # The extrude helper builds along local +Z; after rotating around Y, that
    # extrusion becomes the binocular viewing axis (+X).  The profile's local X
    # becomes vertical height, and local Y becomes barrel width.
    profile = superellipse_profile(0.058, 0.048, exponent=3.2, segments=56)
    geom = ExtrudeGeometry(profile, 0.126, cap=True, center=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _objective_ring_mesh(name: str):
    """Short hollow flared front ring, oriented along +X."""
    outer = [(0.0215, -0.010), (0.0240, -0.005), (0.0240, 0.006), (0.0220, 0.011)]
    inner = [(0.0155, -0.009), (0.0150, -0.004), (0.0150, 0.008), (0.0158, 0.010)]
    geom = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _eyepiece_ring_mesh(name: str):
    """Soft rubber eyecup shell, oriented along +X."""
    outer = [(0.0160, -0.012), (0.0185, -0.006), (0.0185, 0.007), (0.0165, 0.012)]
    inner = [(0.0108, -0.011), (0.0108, -0.004), (0.0118, 0.007), (0.0122, 0.011)]
    geom = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _focus_wheel_mesh():
    geom = KnobGeometry(
        0.034,
        0.026,
        body_style="hourglass",
        edge_radius=0.001,
        grip=KnobGrip(style="ribbed", count=24, depth=0.0010, width=0.0012),
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, "focus_wheel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="image_stabilized_10x30_binocular")

    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    dark_rubber = model.material("dark_charcoal_rubber", rgba=(0.045, 0.047, 0.045, 1.0))
    graphite = model.material("graphite_polymer", rgba=(0.12, 0.13, 0.13, 1.0))
    lens = model.material("blue_coated_glass", rgba=(0.10, 0.22, 0.36, 0.55))
    label = model.material("muted_silver_label", rgba=(0.55, 0.57, 0.55, 1.0))

    barrel_y = 0.038

    left_body = model.part("left_body")
    left_body.visual(
        _armor_mesh("left_armor"),
        origin=Origin(xyz=(0.0, -barrel_y, 0.0)),
        material=rubber,
        name="left_armor",
    )
    left_body.visual(
        _objective_ring_mesh("left_objective_ring"),
        origin=Origin(xyz=(0.070, -barrel_y, 0.0)),
        material=dark_rubber,
        name="left_objective_ring",
    )
    left_body.visual(
        _eyepiece_ring_mesh("left_eyepiece_ring"),
        origin=Origin(xyz=(-0.071, -barrel_y, 0.0)),
        material=dark_rubber,
        name="left_eyepiece_ring",
    )
    left_body.visual(
        Cylinder(radius=0.0158, length=0.0025),
        origin=Origin(xyz=(0.080, -barrel_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="left_objective_glass",
    )
    left_body.visual(
        Cylinder(radius=0.0118, length=0.0020),
        origin=Origin(xyz=(-0.081, -barrel_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="left_eyepiece_glass",
    )
    for i, x in enumerate((-0.032, -0.010, 0.014, 0.038)):
        left_body.visual(
            Box((0.010, 0.005, 0.039)),
            origin=Origin(xyz=(x, -0.063, 0.000)),
            material=dark_rubber,
            name=f"left_grip_rib_{i}",
        )
    left_body.visual(
        Box((0.036, 0.018, 0.0025)),
        origin=Origin(xyz=(0.012, -barrel_y, 0.0300)),
        material=label,
        name="left_10x30_plate",
    )
    # Central hinge pin and the left-side bridge lugs are in the root half.
    left_body.visual(
        Cylinder(radius=0.007, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="central_hinge_pin",
    )
    for i, x in enumerate((-0.034, 0.034)):
        left_body.visual(
            Box((0.036, 0.008, 0.010)),
            origin=Origin(xyz=(x, -0.011, 0.0)),
            material=graphite,
            name=f"left_bridge_lug_{i}",
        )
    # Fork cheeks hold the focus wheel axle above the central bridge.
    for i, x in enumerate((-0.043, 0.003)):
        left_body.visual(
            Box((0.006, 0.014, 0.0366)),
            origin=Origin(xyz=(x, 0.0, 0.0183)),
            material=graphite,
            name=f"focus_fork_{i}",
        )
    left_body.visual(
        Cylinder(radius=0.0048, length=0.050),
        origin=Origin(xyz=(-0.020, 0.0, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="focus_axle",
    )

    right_body = model.part("right_body")
    right_body.visual(
        _armor_mesh("right_armor"),
        origin=Origin(xyz=(0.0, barrel_y, 0.0)),
        material=rubber,
        name="right_armor",
    )
    right_body.visual(
        _objective_ring_mesh("right_objective_ring"),
        origin=Origin(xyz=(0.070, barrel_y, 0.0)),
        material=dark_rubber,
        name="right_objective_ring",
    )
    right_body.visual(
        _eyepiece_ring_mesh("right_eyepiece_ring"),
        origin=Origin(xyz=(-0.071, barrel_y, 0.0)),
        material=dark_rubber,
        name="right_eyepiece_ring",
    )
    right_body.visual(
        Cylinder(radius=0.0158, length=0.0025),
        origin=Origin(xyz=(0.080, barrel_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="right_objective_glass",
    )
    right_body.visual(
        Cylinder(radius=0.0118, length=0.0020),
        origin=Origin(xyz=(-0.081, barrel_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="right_eyepiece_glass",
    )
    for i, x in enumerate((-0.032, -0.010, 0.014, 0.038)):
        right_body.visual(
            Box((0.010, 0.005, 0.039)),
            origin=Origin(xyz=(x, 0.063, 0.000)),
            material=dark_rubber,
            name=f"right_grip_rib_{i}",
        )
    right_body.visual(
        Box((0.036, 0.018, 0.0025)),
        origin=Origin(xyz=(0.012, barrel_y, 0.0300)),
        material=label,
        name="right_is_plate",
    )
    for x, lug_name in ((-0.034, "right_bridge_lug_0"), (0.034, "right_bridge_lug_1")):
        right_body.visual(
            Box((0.036, 0.008, 0.010)),
            origin=Origin(xyz=(x, 0.011, 0.0)),
            material=graphite,
            name=lug_name,
        )
    right_body.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.010, 0.065, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="stabilizer_boss",
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        _focus_wheel_mesh(),
        origin=Origin(),
        material=dark_rubber,
        name="focus_wheel",
    )

    stabilizer_lever = model.part("stabilizer_lever")
    stabilizer_lever.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="lever_knuckle",
    )
    stabilizer_lever.visual(
        Box((0.030, 0.005, 0.014)),
        origin=Origin(xyz=(0.014, 0.007, 0.0)),
        material=dark_rubber,
        name="lever_paddle",
    )

    model.articulation(
        "center_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.8, lower=-0.16, upper=0.16),
    )
    model.articulation(
        "focus_joint",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=focus_knob,
        origin=Origin(xyz=(-0.020, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "lever_hinge",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=stabilizer_lever,
        origin=Origin(xyz=(0.010, 0.068, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=3.0, lower=0.0, upper=0.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_knob = object_model.get_part("focus_knob")
    stabilizer_lever = object_model.get_part("stabilizer_lever")
    center_hinge = object_model.get_articulation("center_hinge")
    focus_joint = object_model.get_articulation("focus_joint")
    lever_hinge = object_model.get_articulation("lever_hinge")

    ctx.allow_overlap(
        left_body,
        focus_knob,
        elem_a="focus_axle",
        elem_b="focus_wheel",
        reason="The focus wheel is intentionally captured around a hidden metal axle on the bridge.",
    )
    ctx.expect_within(
        left_body,
        focus_knob,
        axes="yz",
        inner_elem="focus_axle",
        outer_elem="focus_wheel",
        margin=0.001,
        name="focus axle is centered inside wheel",
    )
    ctx.expect_overlap(
        left_body,
        focus_knob,
        axes="x",
        elem_a="focus_axle",
        elem_b="focus_wheel",
        min_overlap=0.024,
        name="focus axle spans the wheel hub",
    )

    ctx.expect_gap(
        right_body,
        left_body,
        axis="y",
        positive_elem="right_bridge_lug_0",
        negative_elem="central_hinge_pin",
        max_gap=0.001,
        max_penetration=0.0001,
        name="right bridge lug seats on central hinge pin",
    )
    ctx.expect_contact(
        stabilizer_lever,
        right_body,
        elem_a="lever_knuckle",
        elem_b="stabilizer_boss",
        contact_tol=0.0015,
        name="stabilizer lever knuckle is mounted on its boss",
    )

    ctx.check(
        "binocular has three revolute user mechanisms",
        center_hinge.articulation_type == ArticulationType.REVOLUTE
        and focus_joint.articulation_type == ArticulationType.REVOLUTE
        and lever_hinge.articulation_type == ArticulationType.REVOLUTE,
    )

    rest_aabb = ctx.part_element_world_aabb(right_body, elem="right_armor")
    rest_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    with ctx.pose({center_hinge: center_hinge.motion_limits.upper}):
        folded_aabb = ctx.part_element_world_aabb(right_body, elem="right_armor")
        folded_z = None if folded_aabb is None else (folded_aabb[0][2] + folded_aabb[1][2]) * 0.5
    ctx.check(
        "right barrel pivots about the central hinge",
        rest_z is not None and folded_z is not None and folded_z > rest_z + 0.004,
        details=f"rest_z={rest_z}, folded_z={folded_z}",
    )

    rest_lever_aabb = ctx.part_element_world_aabb(stabilizer_lever, elem="lever_paddle")
    rest_lever_z = None if rest_lever_aabb is None else (rest_lever_aabb[0][2] + rest_lever_aabb[1][2]) * 0.5
    with ctx.pose({lever_hinge: lever_hinge.motion_limits.upper}):
        lever_aabb = ctx.part_element_world_aabb(stabilizer_lever, elem="lever_paddle")
        lever_z = None if lever_aabb is None else (lever_aabb[0][2] + lever_aabb[1][2]) * 0.5
    ctx.check(
        "stabilizer lever swings on right barrel hinge",
        rest_lever_z is not None and lever_z is not None and lever_z < rest_lever_z - 0.005,
        details=f"rest_z={rest_lever_z}, activated_z={lever_z}",
    )

    return ctx.report()


object_model = build_object_model()
