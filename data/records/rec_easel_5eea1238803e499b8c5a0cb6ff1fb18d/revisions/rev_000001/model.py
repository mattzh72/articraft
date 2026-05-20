from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_between(start: tuple[float, float, float], end: tuple[float, float, float], radius: float):
    """Return a mesh cylinder whose local endpoints are start and end."""

    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")

    geom = CylinderGeometry(radius, length, radial_segments=28, closed=True)
    ux, uy, uz = vx / length, vy / length, vz / length
    # Rotate the cylinder's local +Z axis onto the requested direction.
    ax, ay, az = -uy, ux, 0.0  # cross((0, 0, 1), direction)
    axis_len = math.sqrt(ax * ax + ay * ay + az * az)
    dot = max(-1.0, min(1.0, uz))
    if axis_len > 1e-9:
        geom.rotate((ax / axis_len, ay / axis_len, az / axis_len), math.acos(dot))
    elif uz < 0.0:
        geom.rotate((1.0, 0.0, 0.0), math.pi)
    geom.translate((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chalkboard_easel")

    paint = model.material("blue_powder_coat", rgba=(0.05, 0.22, 0.55, 1.0))
    board_mat = model.material("matte_chalkboard_green", rgba=(0.02, 0.11, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.04, 0.018, 0.68)),
        origin=Origin(xyz=(0.0, -0.038, 1.08)),
        material=board_mat,
        name="writing_surface",
    )
    frame.visual(
        Box((1.16, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, -0.025, 1.455)),
        material=paint,
        name="top_rail",
    )
    frame.visual(
        Box((1.16, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, -0.025, 0.715)),
        material=paint,
        name="bottom_rail",
    )
    for side, x in ((0, -0.585), (1, 0.585)):
        frame.visual(
            Box((0.055, 0.055, 0.78)),
            origin=Origin(xyz=(x, -0.025, 1.085)),
            material=paint,
            name=f"side_rail_{side}",
        )
    frame.visual(
        Cylinder(radius=0.025, length=1.42),
        origin=Origin(xyz=(0.0, 0.020, 1.550), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=paint,
        name="top_pin",
    )
    for side, x in ((0, -0.30), (1, 0.30)):
        frame.visual(
            Box((0.045, 0.050, 0.080)),
            origin=Origin(xyz=(x, -0.002, 1.505)),
            material=paint,
            name=f"crown_strut_{side}",
        )
    frame.visual(
        Box((1.16, 0.130, 0.025)),
        origin=Origin(xyz=(0.0, -0.105, 0.700)),
        material=paint,
        name="tray_shelf",
    )
    frame.visual(
        Box((1.18, 0.025, 0.080)),
        origin=Origin(xyz=(0.0, -0.172, 0.732)),
        material=paint,
        name="tray_lip",
    )
    for side, x in ((0, -0.59), (1, 0.59)):
        frame.visual(
            Box((0.030, 0.120, 0.060)),
            origin=Origin(xyz=(x, -0.108, 0.728)),
            material=paint,
            name=f"tray_end_{side}",
        )

    hinge_x = 0.655
    front_specs = (
        ("front_leg_0", -hinge_x, (-0.055, -0.335, -1.505)),
        ("front_leg_1", hinge_x, (0.055, -0.335, -1.505)),
    )
    for leg_name, x, foot in front_specs:
        leg = model.part(leg_name)
        leg.visual(
            Cylinder(radius=0.036, length=0.105),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=paint,
            name="hinge_collar",
        )
        leg.visual(
            Box((0.060, 0.014, 0.040)),
            origin=Origin(xyz=(0.0, -0.033, -0.025)),
            material=paint,
            name="hinge_socket",
        )
        leg.visual(
            mesh_from_geometry(
                _tube_between((0.0, -0.025, -0.045), foot, 0.022),
                f"{leg_name}_tube",
            ),
            material=paint,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=0.043),
            origin=Origin(xyz=foot),
            material=rubber,
            name="rubber_foot",
        )
        model.articulation(
            f"{leg_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=leg,
            origin=Origin(xyz=(x, 0.020, 1.550)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-0.18, upper=0.55),
        )

    rear_leg = model.part("rear_leg")
    rear_foot = (0.0, 0.700, -1.505)
    rear_leg.visual(
        Cylinder(radius=0.036, length=0.125),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=paint,
        name="hinge_collar",
    )
    rear_leg.visual(
        Box((0.065, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, 0.038, -0.025)),
        material=paint,
        name="hinge_socket",
    )
    rear_leg.visual(
        mesh_from_geometry(
            _tube_between((0.0, 0.045, -0.045), rear_foot, 0.024),
            "rear_leg_tube",
        ),
        material=paint,
        name="leg_tube",
    )
    rear_leg.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=rear_foot),
        material=rubber,
        name="rubber_foot",
    )
    model.articulation(
        "rear_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, 0.020, 1.550)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-0.45, upper=0.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_leg_0 = object_model.get_part("front_leg_0")
    front_leg_1 = object_model.get_part("front_leg_1")
    rear_leg = object_model.get_part("rear_leg")

    for leg_name in ("front_leg_0", "front_leg_1", "rear_leg"):
        ctx.allow_overlap(
            "frame",
            leg_name,
            elem_a="top_pin",
            elem_b="hinge_collar",
            reason="The steel hinge collar is intentionally captured around the solid crown pin proxy.",
        )
        ctx.expect_within(
            "frame",
            leg_name,
            axes="yz",
            inner_elem="top_pin",
            outer_elem="hinge_collar",
            margin=0.0,
            name=f"{leg_name} collar surrounds crown pin",
        )

    ctx.expect_overlap(
        frame,
        front_leg_0,
        axes="x",
        elem_a="top_pin",
        elem_b="hinge_collar",
        min_overlap=0.075,
        name="front_leg_0 retained on pin",
    )
    ctx.expect_overlap(
        frame,
        front_leg_1,
        axes="x",
        elem_a="top_pin",
        elem_b="hinge_collar",
        min_overlap=0.075,
        name="front_leg_1 retained on pin",
    )
    ctx.expect_overlap(
        frame,
        rear_leg,
        axes="x",
        elem_a="top_pin",
        elem_b="hinge_collar",
        min_overlap=0.095,
        name="rear leg retained on pin",
    )

    def _element_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) * 0.5,
            (lo[1] + hi[1]) * 0.5,
            (lo[2] + hi[2]) * 0.5,
        )

    shelf_aabb = ctx.part_element_world_aabb(frame, elem="tray_shelf")
    surface_aabb = ctx.part_element_world_aabb(frame, elem="writing_surface")
    ctx.check(
        "tray is full width of chalkboard",
        shelf_aabb is not None
        and surface_aabb is not None
        and (shelf_aabb[1][0] - shelf_aabb[0][0]) >= (surface_aabb[1][0] - surface_aabb[0][0]) + 0.08,
        details=f"shelf={shelf_aabb}, surface={surface_aabb}",
    )

    foot_0 = _element_center(front_leg_0, "rubber_foot")
    foot_1 = _element_center(front_leg_1, "rubber_foot")
    rear_foot = _element_center(rear_leg, "rubber_foot")
    ctx.check(
        "feet form a wide A-frame stance",
        foot_0 is not None
        and foot_1 is not None
        and rear_foot is not None
        and abs(foot_1[0] - foot_0[0]) > 1.25
        and foot_0[1] < -0.25
        and foot_1[1] < -0.25
        and rear_foot[1] > 0.65,
        details=f"front0={foot_0}, front1={foot_1}, rear={rear_foot}",
    )

    for joint_name in ("front_leg_0_hinge", "front_leg_1_hinge", "rear_leg_hinge"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a crown revolute",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None,
            details=f"{joint}",
        )

    front_rest = _element_center(front_leg_0, "rubber_foot")
    front_hinge = object_model.get_articulation("front_leg_0_hinge")
    with ctx.pose({front_hinge: 0.12}):
        front_folded = _element_center(front_leg_0, "rubber_foot")
    ctx.check(
        "front leg swings on crown hinge",
        front_rest is not None and front_folded is not None and front_folded[1] > front_rest[1] + 0.12,
        details=f"rest={front_rest}, folded={front_folded}",
    )

    rear_rest = _element_center(rear_leg, "rubber_foot")
    rear_hinge = object_model.get_articulation("rear_leg_hinge")
    with ctx.pose({rear_hinge: 0.12}):
        rear_opened = _element_center(rear_leg, "rubber_foot")
    ctx.check(
        "rear leg swings on crown hinge",
        rear_rest is not None and rear_opened is not None and rear_opened[1] > rear_rest[1] + 0.12,
        details=f"rest={rear_rest}, opened={rear_opened}",
    )

    return ctx.report()


object_model = build_object_model()
