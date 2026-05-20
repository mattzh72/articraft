from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PIN_RPY = (-math.pi / 2.0, 0.0, 0.0)


def _superellipse_point(width: float, height: float, angle: float, exponent: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    y = (width * 0.5) * math.copysign(abs(c) ** (2.0 / exponent), c)
    z = (height * 0.5) * math.copysign(abs(s) ** (2.0 / exponent), s)
    return y, z


def _tapered_segment_mesh(
    *,
    x0: float,
    x1: float,
    width0: float,
    height0: float,
    width1: float,
    height1: float,
    segments: int = 28,
) -> MeshGeometry:
    """Closed rounded-rectangular tapered beam running along local +X."""
    geom = MeshGeometry()
    exponent = 3.0
    for x, width, height in ((x0, width0, height0), (x1, width1, height1)):
        for i in range(segments):
            y, z = _superellipse_point(width, height, 2.0 * math.pi * i / segments, exponent)
            geom.add_vertex(x, y, z)

    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(i, j, segments + i)
        geom.add_face(j, segments + j, segments + i)

    c0 = geom.add_vertex(x0, 0.0, 0.0)
    c1 = geom.add_vertex(x1, 0.0, 0.0)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(c0, j, i)
        geom.add_face(c1, segments + i, segments + j)
    return geom


def _u_yoke_mesh(
    *,
    x_center: float,
    x_length: float,
    outer_width: float,
    gap_width: float,
    height: float,
    bridge_thickness: float,
) -> MeshGeometry:
    """One-piece inverted-U clevis/yoke extruded along X."""
    x_back = x_center - x_length * 0.5
    x_front = x_center + x_length * 0.5
    y_outer = outer_width * 0.5
    y_gap = gap_width * 0.5
    z_bottom = -height * 0.5
    z_top = height * 0.5
    z_bridge = z_top - bridge_thickness

    profile = [
        (-y_outer, z_bottom),
        (-y_gap, z_bottom),
        (-y_gap, z_bridge),
        (y_gap, z_bridge),
        (y_gap, z_bottom),
        (y_outer, z_bottom),
        (y_outer, z_top),
        (-y_outer, z_top),
    ]

    geom = MeshGeometry()
    for x in (x_back, x_front):
        for y, z in profile:
            geom.add_vertex(x, y, z)

    # Side walls around the U-shaped boundary.
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(i, j, n + i)
        geom.add_face(j, n + j, n + i)

    # Cap each end as three rectangles: left cheek, bridge, right cheek.
    cap_quads = ((0, 1, 2, 7), (2, 3, 6, 7), (4, 5, 6, 3))
    for a, b, c, d in cap_quads:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
        geom.add_face(n + a, n + c, n + b)
        geom.add_face(n + a, n + d, n + c)
    return geom


def _add_pin_stack(
    part,
    *,
    metal,
    brass,
    hub_radius: float,
    hub_length: float,
    pin_radius: float,
    pin_length: float,
    cap_radius: float,
    cap_thickness: float,
) -> None:
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=PIN_RPY),
        material=metal,
        name="proximal_bearing",
    )
    part.visual(
        Cylinder(radius=pin_radius, length=pin_length),
        origin=Origin(rpy=PIN_RPY),
        material=brass,
        name="pin_shaft",
    )
    cap_offset = pin_length * 0.5 + cap_thickness * 0.5
    for side, y in enumerate((-cap_offset, cap_offset)):
        part.visual(
            Cylinder(radius=cap_radius, length=cap_thickness),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=PIN_RPY),
            material=brass,
            name=f"pin_cap_{side}",
        )


def _add_segment(
    part,
    *,
    length: float,
    body_start: float,
    body_end: float,
    width0: float,
    height0: float,
    width1: float,
    height1: float,
    body_material,
    metal,
    brass,
    has_yoke: bool,
    mesh_name: str,
) -> None:
    _add_pin_stack(
        part,
        metal=metal,
        brass=brass,
        hub_radius=0.014,
        hub_length=0.034,
        pin_radius=0.0055,
        pin_length=0.066,
        cap_radius=0.0105,
        cap_thickness=0.006,
    )
    part.visual(
        mesh_from_geometry(
            _tapered_segment_mesh(
                x0=body_start,
                x1=body_end,
                width0=width0,
                height0=height0,
                width1=width1,
                height1=height1,
            ),
            mesh_name,
        ),
        material=body_material,
        name="tapered_body",
    )
    # Low-profile cover screws on the machined link body.
    for i, x in enumerate((body_start + 0.030, max(body_start + 0.050, body_end - 0.020))):
        if x < body_end - 0.004:
            local_height = height0 + (height1 - height0) * ((x - body_start) / (body_end - body_start))
            part.visual(
                Cylinder(radius=0.004, length=0.002),
                origin=Origin(xyz=(x, 0.0, local_height * 0.5 + 0.0008)),
                material=metal,
                name=f"cover_screw_{i}",
            )

    if has_yoke:
        part.visual(
            mesh_from_geometry(
                _u_yoke_mesh(
                    x_center=length,
                    x_length=0.046,
                    outer_width=0.060,
                    gap_width=0.042,
                    height=0.044,
                    bridge_thickness=0.006,
                ),
                f"{mesh_name}_yoke",
            ),
            material=metal,
            name="distal_yoke",
        )
        part.visual(
            Box((0.020, 0.025, 0.006)),
            origin=Origin(xyz=(length - 0.026, 0.0, 0.0145)),
            material=metal,
            name="distal_saddle",
        )
    else:
        part.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(length + 0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="black_rubber",
            name="end_bumper",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_phalanx_chain")

    gunmetal = model.material("anodized_gunmetal", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    brass = model.material("brass_pin", rgba=(0.86, 0.63, 0.25, 1.0))
    model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base_mount")
    base.visual(
        Box((0.120, 0.085, 0.010)),
        origin=Origin(xyz=(-0.045, 0.0, -0.034)),
        material=gunmetal,
        name="mount_plate",
    )
    for side, y in enumerate((-0.026, 0.026)):
        base.visual(
            Box((0.030, 0.008, 0.058)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_steel,
            name=f"clevis_cheek_{side}",
        )
    for i, x in enumerate((-0.080, -0.028)):
        for j, y in enumerate((-0.026, 0.026)):
            base.visual(
                Cylinder(radius=0.0045, length=0.0025),
                origin=Origin(xyz=(x, y, -0.0285)),
                material=dark_steel,
                name=f"mount_screw_{i}_{j}",
            )

    proximal = model.part("proximal_segment")
    middle = model.part("middle_segment")
    distal = model.part("distal_segment")

    l1 = 0.118
    l2 = 0.083
    l3 = 0.060

    _add_segment(
        proximal,
        length=l1,
        body_start=0.010,
        body_end=l1 - 0.020,
        width0=0.032,
        height0=0.030,
        width1=0.026,
        height1=0.025,
        body_material=gunmetal,
        metal=dark_steel,
        brass=brass,
        has_yoke=True,
        mesh_name="proximal_tapered_body",
    )
    _add_segment(
        middle,
        length=l2,
        body_start=0.010,
        body_end=l2 - 0.018,
        width0=0.028,
        height0=0.026,
        width1=0.022,
        height1=0.022,
        body_material=gunmetal,
        metal=dark_steel,
        brass=brass,
        has_yoke=True,
        mesh_name="middle_tapered_body",
    )
    _add_segment(
        distal,
        length=l3,
        body_start=0.010,
        body_end=l3,
        width0=0.024,
        height0=0.023,
        width1=0.016,
        height1=0.017,
        body_material=gunmetal,
        metal=dark_steel,
        brass=brass,
        has_yoke=False,
        mesh_name="distal_tapered_body",
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(l1, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(l2, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mount")
    proximal = object_model.get_part("proximal_segment")
    middle = object_model.get_part("middle_segment")
    distal = object_model.get_part("distal_segment")

    base_joint = object_model.get_articulation("base_to_proximal")
    proximal_joint = object_model.get_articulation("proximal_to_middle")
    middle_joint = object_model.get_articulation("middle_to_distal")

    for cheek in ("clevis_cheek_0", "clevis_cheek_1"):
        ctx.allow_overlap(
            base,
            proximal,
            elem_a=cheek,
            elem_b="pin_shaft",
            reason="The brass shaft is intentionally captured through the base clevis bore proxy.",
        )
        ctx.expect_overlap(
            base,
            proximal,
            axes="xyz",
            min_overlap=0.003,
            elem_a=cheek,
            elem_b="pin_shaft",
            name=f"base pin passes through {cheek}",
        )

    for parent, child, label in (
        (proximal, middle, "middle"),
        (middle, distal, "distal"),
    ):
        ctx.allow_overlap(
            parent,
            child,
            elem_a="distal_yoke",
            elem_b="pin_shaft",
            reason=f"The {label} brass shaft is intentionally captured through a solid yoke-bore proxy.",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="xyz",
            min_overlap=0.003,
            elem_a="distal_yoke",
            elem_b="pin_shaft",
            name=f"{label} pin remains captured in yoke",
        )

    rest_tip = ctx.part_element_world_aabb(distal, elem="end_bumper")
    with ctx.pose({base_joint: 0.75, proximal_joint: 0.75, middle_joint: 0.55}):
        flexed_tip = ctx.part_element_world_aabb(distal, elem="end_bumper")

    ctx.check(
        "serial revolutes flex the chain downward",
        rest_tip is not None
        and flexed_tip is not None
        and flexed_tip[0][2] < rest_tip[0][2] - 0.045,
        details=f"rest_tip={rest_tip}, flexed_tip={flexed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
