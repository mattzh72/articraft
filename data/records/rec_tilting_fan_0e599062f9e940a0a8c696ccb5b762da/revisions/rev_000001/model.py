from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Return a mesh cylinder whose axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("guard wire segment is too short")

    ux, uy, uz = dx / length, dy / length, dz / length
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)

    # Rotate the cylinder's local +Z onto the desired segment direction.
    dot = max(-1.0, min(1.0, uz))
    angle = math.acos(dot)
    axis = (-uy, ux, 0.0)
    axis_len = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
    if axis_len > 1e-9:
        geom.rotate((axis[0] / axis_len, axis[1] / axis_len, 0.0), angle)
    elif uz < 0.0:
        geom.rotate((1.0, 0.0, 0.0), math.pi)

    geom.translate((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5)
    return geom


def _guard_ring(radius: float, tube_radius: float, x: float, name: str):
    geom = TorusGeometry(radius, tube_radius, radial_segments=18, tubular_segments=72)
    # TorusGeometry is in local XY with a local-Z normal; put it in the YZ fan-guard plane.
    geom.rotate_y(math.pi / 2.0).translate(x, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def _radial_guard_wire(
    radius_inner: float,
    radius_outer: float,
    angle: float,
    x: float,
    name: str,
):
    start = (x, radius_inner * math.cos(angle), radius_inner * math.sin(angle))
    end = (x, radius_outer * math.cos(angle), radius_outer * math.sin(angle))
    return mesh_from_geometry(_cylinder_between(start, end, 0.0028), name)


def _rim_depth_wire(radius: float, angle: float, x0: float, x1: float, name: str):
    y = radius * math.cos(angle)
    z = radius * math.sin(angle)
    return mesh_from_geometry(_cylinder_between((x0, y, z), (x1, y, z), 0.0032), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_desk_fan")

    dark_plastic = model.material("dark_plastic", rgba=(0.035, 0.038, 0.043, 1.0))
    satin_black = model.material("satin_black", rgba=(0.01, 0.012, 0.014, 1.0))
    grey_plastic = model.material("warm_grey_plastic", rgba=(0.58, 0.60, 0.61, 1.0))
    wire_chrome = model.material("dark_chrome_wire", rgba=(0.16, 0.17, 0.18, 1.0))
    blade_blue = model.material("translucent_blue_blades", rgba=(0.50, 0.72, 0.92, 0.72))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.165, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_plastic,
        name="oval_foot",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=grey_plastic,
        name="pedestal_cap",
    )
    base.visual(
        Cylinder(radius=0.021, length=0.300),
        origin=Origin(xyz=(-0.030, 0.0, 0.190)),
        material=satin_black,
        name="support_post",
    )
    base.visual(
        Box((0.070, 0.360, 0.034)),
        origin=Origin(xyz=(-0.045, 0.0, 0.342)),
        material=satin_black,
        name="yoke_bridge",
    )
    base.visual(
        Box((0.036, 0.026, 0.260)),
        origin=Origin(xyz=(-0.045, -0.178, 0.458)),
        material=satin_black,
        name="side_bracket_0",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(-0.045, -0.178, 0.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey_plastic,
        name="pivot_bushing_0",
    )
    base.visual(
        Box((0.036, 0.026, 0.260)),
        origin=Origin(xyz=(-0.045, 0.178, 0.458)),
        material=satin_black,
        name="side_bracket_1",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(-0.045, 0.178, 0.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey_plastic,
        name="pivot_bushing_1",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.056, length=0.076),
        origin=Origin(xyz=(-0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_plastic,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.390),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey_plastic,
        name="tilt_axle",
    )
    head.visual(
        Cylinder(radius=0.071, length=0.024),
        origin=Origin(xyz=(-0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_plastic,
        name="rear_mount_plate",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(xyz=(-0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_plastic,
        name="motor_neck",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.045),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="drive_shaft",
    )

    # Stationary protective cage: front/rear circular rings, radial spokes, and depth struts.
    head.visual(
        _guard_ring(0.148, 0.0056, 0.070, "front_outer_ring"),
        material=wire_chrome,
        name="front_outer_ring",
    )
    head.visual(
        _guard_ring(0.148, 0.0052, -0.064, "rear_outer_ring"),
        material=wire_chrome,
        name="rear_outer_ring",
    )
    for r in (0.054, 0.092, 0.124):
        name = f"front_guard_ring_{int(r * 1000)}"
        head.visual(_guard_ring(r, 0.0025, 0.073, name), material=wire_chrome, name=name)
    for r in (0.080, 0.120):
        name = f"rear_guard_ring_{int(r * 1000)}"
        head.visual(_guard_ring(r, 0.0024, -0.064, name), material=wire_chrome, name=name)
    for idx in range(12):
        angle = idx * math.tau / 12.0
        name = f"front_spoke_{idx}"
        head.visual(
            _radial_guard_wire(0.026, 0.151, angle, 0.073, name),
            material=wire_chrome,
            name=name,
        )
    for idx in range(8):
        angle = idx * math.tau / 8.0
        name = f"rear_spoke_{idx}"
        head.visual(
            _radial_guard_wire(0.060, 0.151, angle, -0.064, name),
            material=wire_chrome,
            name=name,
        )
    for idx in range(8):
        angle = idx * math.tau / 8.0
        name = f"rim_depth_wire_{idx}"
        head.visual(
            _rim_depth_wire(0.148, angle, -0.071, 0.077, name),
            material=wire_chrome,
            name=name,
        )
    head.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(0.077, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wire_chrome,
        name="front_badge",
    )

    rotor = model.part("rotor")
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.109,
            0.025,
            5,
            thickness=0.025,
            blade_pitch_deg=32.0,
            blade_sweep_deg=24.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.14),
            hub=FanRotorHub(style="spinner", bore_diameter=0.006),
        ),
        "rotor_blades",
    )
    rotor.visual(
        rotor_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_blue,
        name="rotor_blades",
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.045, 0.0, 0.520)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("head_tilt")
    spin = object_model.get_articulation("rotor_spin")

    for bushing in ("pivot_bushing_0", "pivot_bushing_1"):
        ctx.allow_overlap(
            head,
            base,
            elem_a="tilt_axle",
            elem_b=bushing,
            reason="The head trunnion axle is intentionally captured inside the side-bracket bushing.",
        )
        ctx.expect_within(
            head,
            base,
            axes="xz",
            inner_elem="tilt_axle",
            outer_elem=bushing,
            margin=0.002,
            name=f"tilt axle centered in {bushing}",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="y",
            elem_a="tilt_axle",
            elem_b=bushing,
            min_overlap=0.020,
            name=f"tilt axle retained by {bushing}",
        )

    for bracket in ("side_bracket_0", "side_bracket_1"):
        ctx.allow_overlap(
            head,
            base,
            elem_a="tilt_axle",
            elem_b=bracket,
            reason="The solid bracket cheek is a simplified pivot plate with the trunnion axle passing through its bore.",
        )
        ctx.expect_within(
            head,
            base,
            axes="xz",
            inner_elem="tilt_axle",
            outer_elem=bracket,
            margin=0.006,
            name=f"tilt axle passes through {bracket}",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="y",
            elem_a="tilt_axle",
            elem_b=bracket,
            min_overlap=0.020,
            name=f"tilt axle crosses {bracket}",
        )

    ctx.allow_overlap(
        head,
        rotor,
        elem_a="drive_shaft",
        elem_b="rotor_blades",
        reason="The compact rotor hub is intentionally captured on the short motor shaft.",
    )
    ctx.expect_within(
        head,
        rotor,
        axes="yz",
        inner_elem="drive_shaft",
        outer_elem="rotor_blades",
        margin=0.002,
        name="drive shaft stays centered in rotor hub",
    )
    ctx.expect_overlap(
        head,
        rotor,
        axes="x",
        elem_a="drive_shaft",
        elem_b="rotor_blades",
        min_overlap=0.008,
        name="rotor hub remains seated on drive shaft",
    )

    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="rotor_blades",
        outer_elem="front_outer_ring",
        margin=0.040,
        name="rotor disk is compact inside the guard diameter",
    )
    ctx.expect_gap(
        base,
        head,
        axis="y",
        positive_elem="side_bracket_1",
        negative_elem="front_outer_ring",
        min_gap=0.006,
        name="guard clears side bracket on positive side",
    )
    ctx.expect_gap(
        head,
        base,
        axis="y",
        positive_elem="front_outer_ring",
        negative_elem="side_bracket_0",
        min_gap=0.006,
        name="guard clears side bracket on negative side",
    )

    ctx.check(
        "rotor uses continuous axle joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    closed_aabb = ctx.part_element_world_aabb(head, elem="front_badge")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_aabb = ctx.part_element_world_aabb(head, elem="front_badge")
    closed_z = None if closed_aabb is None else (closed_aabb[0][2] + closed_aabb[1][2]) * 0.5
    tilted_z = None if tilted_aabb is None else (tilted_aabb[0][2] + tilted_aabb[1][2]) * 0.5
    ctx.check(
        "positive tilt raises the guarded head",
        closed_z is not None and tilted_z is not None and tilted_z > closed_z + 0.020,
        details=f"closed_z={closed_z}, tilted_z={tilted_z}",
    )

    return ctx.report()


object_model = build_object_model()
