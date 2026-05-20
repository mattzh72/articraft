from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _section_loop(
    width: float,
    depth: float,
    z: float,
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    exponent: float = 3.0,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    return [
        (x + dx, y + dy, z)
        for x, y in superellipse_profile(width, depth, exponent=exponent, segments=segments)
    ]


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    dx: float = 0.0,
    dy: float = 0.0,
    radial_segments: int = 48,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.002,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(dx, dy, z_center)


def _build_bottle_shell():
    outer_sections = [
        _section_loop(0.102, 0.066, 0.000, exponent=3.3),
        _section_loop(0.108, 0.072, 0.014, exponent=3.6),
        _section_loop(0.114, 0.075, 0.090, dx=0.002, dy=-0.003, exponent=3.6),
        _section_loop(0.108, 0.070, 0.160, dx=0.006, dy=-0.004, exponent=3.2),
        _section_loop(0.082, 0.058, 0.198, dx=0.012, dy=-0.003, exponent=2.8),
        _section_loop(0.046, 0.040, 0.220, dx=0.017, dy=-0.001, exponent=2.2),
        _section_loop(0.030, 0.030, 0.246, dx=0.018, exponent=2.0),
    ]
    inner_sections = [
        _section_loop(0.094, 0.058, 0.008, exponent=3.3),
        _section_loop(0.100, 0.064, 0.016, exponent=3.6),
        _section_loop(0.106, 0.067, 0.090, dx=0.002, dy=-0.003, exponent=3.6),
        _section_loop(0.100, 0.062, 0.160, dx=0.006, dy=-0.004, exponent=3.2),
        _section_loop(0.074, 0.050, 0.198, dx=0.012, dy=-0.003, exponent=2.8),
        _section_loop(0.024, 0.024, 0.220, dx=0.017, dy=-0.001, exponent=2.0),
        _section_loop(0.024, 0.024, 0.260, dx=0.018, exponent=2.0),
    ]
    outer_solid = section_loft(outer_sections)
    inner_solid = section_loft(inner_sections)
    return boolean_difference(outer_solid, inner_solid)


def _build_cap_mesh():
    cap_shell = boolean_difference(
        CylinderGeometry(radius=0.0205, height=0.031, radial_segments=64).translate(0.0, 0.0, 0.0155),
        CylinderGeometry(radius=0.0172, height=0.028, radial_segments=64).translate(0.0, 0.0, 0.0135),
    )
    for rib_index in range(18):
        angle = rib_index * math.tau / 18.0
        rib = (
            BoxGeometry((0.0042, 0.0024, 0.022))
            .translate(0.0198, 0.0, 0.0135)
            .rotate_z(angle)
        )
        cap_shell.merge(rib)
    return cap_shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.91, 0.93, 0.88, 1.0))
    cap_plastic = model.material("cap_plastic", rgba=(0.16, 0.25, 0.58, 1.0))
    neck_plastic = model.material("neck_plastic", rgba=(0.84, 0.86, 0.80, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.106, 0.072, 0.015, corner_segments=8),
                0.162,
            ),
            "bottle_lower_body",
        ),
        origin=Origin(xyz=(0.003, -0.002, 0.0)),
        material=bottle_plastic,
        name="lower_body",
    )
    bottle_body.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _section_loop(0.106, 0.072, 0.154, dx=0.003, dy=-0.002, exponent=3.6),
                    _section_loop(0.094, 0.064, 0.170, dx=0.007, dy=-0.002, exponent=3.3),
                    _section_loop(0.072, 0.054, 0.186, dx=0.012, dy=-0.001, exponent=2.9),
                    _section_loop(0.040, 0.036, 0.204, dx=0.015, dy=-0.001, exponent=2.4),
                ]
            ),
            "bottle_shoulder",
        ),
        material=bottle_plastic,
        name="bottle_shoulder",
    )
    bottle_body.visual(
        Cylinder(radius=0.0144, length=0.035),
        origin=Origin(xyz=(0.018, 0.0, 0.2205)),
        material=neck_plastic,
        name="thread_core",
    )
    bottle_body.visual(
        Cylinder(radius=0.0152, length=0.010),
        origin=Origin(xyz=(0.018, 0.0, 0.200)),
        material=neck_plastic,
        name="support_collar",
    )
    bottle_body.visual(
        Cylinder(radius=0.0172, length=0.004),
        origin=Origin(xyz=(0.018, 0.0, 0.206)),
        material=neck_plastic,
        name="transfer_ring",
    )
    bottle_body.visual(
        Cylinder(radius=0.0152, length=0.003),
        origin=Origin(xyz=(0.018, 0.0, 0.2370)),
        material=neck_plastic,
        name="top_lip",
    )

    bottle_body.visual(
        Cylinder(radius=0.0159, length=0.0025),
        origin=Origin(xyz=(0.018, 0.0, 0.219)),
        material=neck_plastic,
        name="neck_thread_band_1",
    )
    bottle_body.visual(
        Cylinder(radius=0.0159, length=0.0025),
        origin=Origin(xyz=(0.018, 0.0, 0.225)),
        material=neck_plastic,
        name="neck_thread_band_2",
    )
    bottle_body.visual(
        Cylinder(radius=0.0159, length=0.0025),
        origin=Origin(xyz=(0.018, 0.0, 0.231)),
        material=neck_plastic,
        name="neck_thread_band_3",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.106, 0.072, 0.248)),
        mass=0.11,
        origin=Origin(xyz=(0.003, -0.002, 0.124)),
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_build_cap_mesh(), "cap_shell"),
        material=cap_plastic,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0205, length=0.031),
        mass=0.014,
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0.018, 0.0, 0.211)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.expect_overlap(
        cap,
        bottle_body,
        axes="xy",
        elem_a="cap_shell",
        elem_b="thread_core",
        min_overlap=0.028,
        name="cap remains centered over the threaded neck",
    )
    ctx.expect_contact(
        cap,
        bottle_body,
        elem_a="cap_shell",
        elem_b="top_lip",
        contact_tol=0.0002,
        name="cap closes onto the bottle lip",
    )

    bottle_origin = ctx.part_world_position(bottle_body)
    cap_origin = ctx.part_world_position(cap)
    ctx.check(
        "cap axis is offset from the bottle body centerline",
        bottle_origin is not None
        and cap_origin is not None
        and abs(cap_origin[0] - bottle_origin[0]) >= 0.015,
        details=f"bottle_origin={bottle_origin}, cap_origin={cap_origin}",
    )

    with ctx.pose({cap_spin: math.pi * 0.75}):
        spun_origin = ctx.part_world_position(cap)
        ctx.check(
            "cap spins in place about the neck axis",
            cap_origin is not None
            and spun_origin is not None
            and abs(cap_origin[0] - spun_origin[0]) <= 1e-6
            and abs(cap_origin[1] - spun_origin[1]) <= 1e-6
            and abs(cap_origin[2] - spun_origin[2]) <= 1e-6,
            details=f"rest={cap_origin}, spun={spun_origin}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
