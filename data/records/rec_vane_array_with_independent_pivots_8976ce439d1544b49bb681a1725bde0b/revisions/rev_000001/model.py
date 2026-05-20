from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


VANE_COUNT = 6
VANE_Z0 = 0.165
VANE_SPACING = 0.090
VANE_AXIS_Y = 0.055
VANE_REST_ANGLE = -0.34
VANE_SPAN = 0.594
VANE_CHORD = 0.092
VANE_THICKNESS = 0.022
BOSS_RADIUS = 0.024
BOSS_LENGTH = 0.024
SOCKET_RADIUS = 0.030
SOCKET_LENGTH = 0.028


def _vane_body_mesh():
    """Rounded louver blade, extruded across the bank width."""

    profile = rounded_rect_profile(
        VANE_THICKNESS,
        VANE_CHORD,
        VANE_THICKNESS * 0.48,
        corner_segments=8,
    )
    body = ExtrudeGeometry(profile, VANE_SPAN, cap=True, center=True)
    # The extrusion builder runs along local Z.  Rotate it so the blade spans
    # local X, then set the built-in resting louver pitch about the pivot axis.
    body.rotate_y(math.pi / 2.0).rotate_x(VANE_REST_ANGLE)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_louver_bank")

    dark_powdercoat = model.material("dark_powdercoat", rgba=(0.08, 0.09, 0.095, 1.0))
    graphite = model.material("graphite_vanes", rgba=(0.18, 0.20, 0.21, 1.0))
    black_bushing = model.material("black_bushing", rgba=(0.015, 0.016, 0.017, 1.0))
    wall_paint = model.material("warm_wall", rgba=(0.78, 0.75, 0.68, 1.0))
    screw_metal = model.material("brushed_screw", rgba=(0.58, 0.56, 0.52, 1.0))

    housing = model.part("housing")

    # Four overlapping rails make a continuous rectangular wall frame with a
    # genuinely open center, not a solid proxy across the louver aperture.
    housing.visual(
        Box((0.820, 0.100, 0.080)),
        origin=Origin(xyz=(0.0, 0.040, 0.040)),
        material=dark_powdercoat,
        name="bottom_rail",
    )
    housing.visual(
        Box((0.820, 0.100, 0.080)),
        origin=Origin(xyz=(0.0, 0.040, 0.760)),
        material=dark_powdercoat,
        name="top_rail",
    )
    for side, x in (("left", -0.382), ("right", 0.382)):
        housing.visual(
            Box((0.080, 0.100, 0.800)),
            origin=Origin(xyz=(x, 0.040, 0.400)),
            material=dark_powdercoat,
            name=f"{side}_rail",
        )

    # Flat backing wall / mounting plate behind the frame, touching the rear of
    # the housing so the bank reads as wall-backed rather than freestanding.
    housing.visual(
        Box((0.940, 0.060, 0.860)),
        origin=Origin(xyz=(0.0, -0.040, 0.430)),
        material=wall_paint,
        name="wall_back",
    )

    # Side pivot sockets are fixed to the housing and line up with each blade
    # axis.  They protrude just inside the opening without occupying the clear
    # vane gaps.
    for i in range(VANE_COUNT):
        z = VANE_Z0 + i * VANE_SPACING
        for side, sx in (("left", -1.0), ("right", 1.0)):
            housing.visual(
                Cylinder(radius=SOCKET_RADIUS, length=SOCKET_LENGTH),
                origin=Origin(
                    xyz=(sx * 0.335, VANE_AXIS_Y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black_bushing,
                name=f"socket_{i}_{side}",
            )

    # Four exposed screw heads on the front lip make the housing read as a real
    # wall-mounted bank. They are seated slightly into the same root part.
    for j, z in enumerate((0.095, 0.705)):
        for side, x in (("left", -0.355), ("right", 0.355)):
            housing.visual(
                Cylinder(radius=0.018, length=0.006),
                origin=Origin(xyz=(x, 0.093, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=screw_metal,
                name=f"screw_{j}_{side}",
            )

    vane_mesh = mesh_from_geometry(_vane_body_mesh(), "rounded_louver_vane")
    joint_limits = MotionLimits(effort=2.0, velocity=1.2, lower=-0.60, upper=0.60)

    for i in range(VANE_COUNT):
        z = VANE_Z0 + i * VANE_SPACING
        vane = model.part(f"vane_{i}")
        vane.visual(vane_mesh, material=graphite, name="blade")
        for side, sx, boss_name in (("left", -1.0, "boss_left"), ("right", 1.0, "boss_right")):
            vane.visual(
                Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
                origin=Origin(
                    xyz=(sx * (VANE_SPAN / 2.0 + BOSS_LENGTH / 2.0), 0.0, 0.0),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black_bushing,
                name=boss_name,
            )

        model.articulation(
            f"housing_to_vane_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            origin=Origin(xyz=(0.0, VANE_AXIS_Y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=joint_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    vanes = [object_model.get_part(f"vane_{i}") for i in range(VANE_COUNT)]
    joints = [object_model.get_articulation(f"housing_to_vane_{i}") for i in range(VANE_COUNT)]

    ctx.check(
        "separate vanes have separate revolute joints",
        len(vanes) == VANE_COUNT
        and len(joints) == VANE_COUNT
        and all(j.joint_type == ArticulationType.REVOLUTE for j in joints),
        details=f"vanes={len(vanes)}, joints={[(j.name, j.joint_type) for j in joints]}",
    )
    ctx.check(
        "vane axes are parallel across the bank",
        all(tuple(round(v, 6) for v in j.axis) == (1.0, 0.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    for i in range(VANE_COUNT - 1):
        ctx.expect_gap(
            vanes[i + 1],
            vanes[i],
            axis="z",
            min_gap=0.018,
            name=f"clear air gap between vane {i} and {i + 1}",
        )

    for i, vane in enumerate(vanes):
        ctx.expect_gap(
            housing,
            vane,
            axis="x",
            positive_elem=f"socket_{i}_right",
            negative_elem="boss_right",
            max_gap=0.003,
            max_penetration=0.0,
            name=f"right pivot boss seats at socket {i}",
        )
        ctx.expect_gap(
            vane,
            housing,
            axis="x",
            positive_elem="boss_left",
            negative_elem=f"socket_{i}_left",
            max_gap=0.003,
            max_penetration=0.0,
            name=f"left pivot boss seats at socket {i}",
        )

    middle_joint = joints[VANE_COUNT // 2]
    middle_vane = vanes[VANE_COUNT // 2]
    rest_aabb = ctx.part_element_world_aabb(middle_vane, elem="blade")
    with ctx.pose({middle_joint: middle_joint.motion_limits.upper}):
        open_aabb = ctx.part_element_world_aabb(middle_vane, elem="blade")
    ctx.check(
        "a commanded vane rotation changes blade pitch",
        rest_aabb is not None
        and open_aabb is not None
        and abs((open_aabb[1][1] - open_aabb[0][1]) - (rest_aabb[1][1] - rest_aabb[0][1])) > 0.010,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
