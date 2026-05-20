from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _helix_points(
    radius: float,
    z_start: float,
    z_end: float,
    *,
    turns: float,
    phase: float = 0.0,
    samples_per_turn: int = 24,
) -> list[tuple[float, float, float]]:
    samples = max(16, int(abs(turns) * samples_per_turn) + 1)
    points: list[tuple[float, float, float]] = []
    for index in range(samples):
        t = index / (samples - 1)
        angle = phase + (turns * math.tau * t)
        z = z_start + ((z_end - z_start) * t)
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _hollow_cylinder(outer_radius: float, inner_radius: float, height: float, *, radial_segments: int = 56):
    return boolean_difference(
        CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments),
        CylinderGeometry(radius=inner_radius, height=height + 0.002, radial_segments=radial_segments),
    )


def _build_bulb_thread():
    return tube_from_spline_points(
        _helix_points(
            0.01345,
            -0.0038,
            -0.0234,
            turns=4.75,
            samples_per_turn=20,
        ),
        radius=0.00095,
        samples_per_segment=3,
        radial_segments=12,
        cap_ends=True,
    )


def _build_bulb_glass():
    globe = Sphere(radius=0.034)
    shoulder = Sphere(radius=0.021)
    neck = Cylinder(radius=0.0135, length=0.038)
    return globe, shoulder, neck


def _build_bulb_base():
    base = _hollow_cylinder(0.0132, 0.0114, 0.0260, radial_segments=56).translate(0.0, 0.0, -0.0130)
    base.merge(
        CylinderGeometry(radius=0.0150, height=0.0020, radial_segments=40).translate(
            0.0,
            0.0,
            0.0010,
        )
    )
    return base


def _build_socket_shell():
    return _hollow_cylinder(0.0245, 0.0192, 0.0960, radial_segments=64).translate(0.0, 0.0, 0.0160)


def _build_thread_insert():
    return _hollow_cylinder(0.0194, 0.0160, 0.0420, radial_segments=64).translate(0.0, 0.0, -0.0110)


def _build_socket_thread():
    return tube_from_spline_points(
        _helix_points(
            0.0168,
            -0.0260,
            -0.0010,
            turns=4.6,
            samples_per_turn=22,
        ),
        radius=0.0009,
        samples_per_segment=3,
        radial_segments=12,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_screw_bulb_socket")

    frosted_glass = model.material("frosted_glass", rgba=(0.96, 0.97, 0.98, 0.58))
    bulb_metal = model.material("bulb_metal", rgba=(0.78, 0.79, 0.80, 1.0))
    brass_contact = model.material("brass_contact", rgba=(0.72, 0.58, 0.28, 1.0))
    socket_body = model.material("socket_body", rgba=(0.18, 0.19, 0.20, 1.0))
    socket_insert = model.material("socket_insert", rgba=(0.66, 0.68, 0.70, 1.0))
    gasket_rubber = model.material("gasket_rubber", rgba=(0.11, 0.12, 0.12, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    socket = model.part("socket")
    socket.visual(
        _mesh("socket_housing_shell", _build_socket_shell()),
        material=socket_body,
        name="housing_shell",
    )
    socket.visual(
        _mesh(
            "socket_drip_hood",
            _hollow_cylinder(0.0315, 0.0188, 0.0100, radial_segments=64).translate(0.0, 0.0, 0.0100),
        ),
        origin=Origin(),
        material=socket_body,
        name="drip_hood",
    )
    socket.visual(
        _mesh(
            "socket_drip_lip",
            TorusGeometry(
                radius=0.0315,
                tube=0.0018,
                radial_segments=18,
                tubular_segments=56,
            ).translate(0.0, 0.0, 0.0050),
        ),
        material=socket_body,
        name="drip_lip",
    )
    socket.visual(
        _mesh("socket_thread_insert", _build_thread_insert()),
        material=socket_insert,
        name="thread_insert",
    )
    socket.visual(
        _mesh("socket_female_thread", _build_socket_thread()),
        material=socket_insert,
        name="female_thread",
    )
    socket.visual(
        _mesh(
            "socket_mouth_gasket",
            TorusGeometry(
                radius=0.0149,
                tube=0.0013,
                radial_segments=16,
                tubular_segments=48,
            ).translate(0.0, 0.0, 0.0008),
        ),
        material=gasket_rubber,
        name="mouth_gasket",
    )
    socket.visual(
        Cylinder(radius=0.0245, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0610)),
        material=socket_body,
        name="rear_cap",
    )
    socket.visual(
        Cylinder(radius=0.0090, length=0.0120),
        origin=Origin(xyz=(0.0, 0.0, 0.0580)),
        material=gasket_rubber,
        name="cable_gland",
    )
    socket.visual(
        Cylinder(radius=0.0030, length=0.0120),
        origin=Origin(xyz=(-0.0085, 0.0, 0.0600)),
        material=hardware_steel,
        name="mount_screw_left",
    )
    socket.visual(
        Cylinder(radius=0.0030, length=0.0120),
        origin=Origin(xyz=(0.0085, 0.0, 0.0600)),
        material=hardware_steel,
        name="mount_screw_right",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0340, length=0.0720),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.0200)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0135, length=0.0340),
        origin=Origin(xyz=(0.0, 0.0, -0.0190)),
        material=frosted_glass,
        name="glass_shell",
    )
    bulb.visual(
        Sphere(radius=0.0210),
        origin=Origin(xyz=(0.0, 0.0, -0.0510)),
        material=frosted_glass,
        name="glass_shoulder",
    )
    bulb.visual(
        Sphere(radius=0.0340),
        origin=Origin(xyz=(0.0, 0.0, -0.0840)),
        material=frosted_glass,
        name="glass_globe",
    )
    bulb.visual(
        _mesh("bulb_base_assembly", _build_bulb_base()),
        material=bulb_metal,
        name="base_shell",
    )
    bulb.visual(
        _mesh("bulb_male_thread", _build_bulb_thread()),
        material=bulb_metal,
        name="male_thread",
    )
    bulb.visual(
        _mesh(
            "bulb_seal_ring",
            TorusGeometry(
                radius=0.0143,
                tube=0.0010,
                radial_segments=18,
                tubular_segments=56,
            ).translate(0.0, 0.0, 0.0008),
        ),
        material=gasket_rubber,
        name="seal_ring",
    )
    bulb.visual(
        Cylinder(radius=0.0045, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, -0.0273)),
        material=brass_contact,
        name="tip_contact",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0320, length=0.1380),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, -0.0420)),
    )

    model.articulation(
        "bulb_thread",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=12.0,
            lower=0.0,
            upper=6.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    bulb_thread = object_model.get_articulation("bulb_thread")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(
        socket,
        bulb,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="bulb_and_socket_are_coaxial",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="base_shell",
        outer_elem="thread_insert",
        margin=0.001,
        name="screw_base_within_threaded_collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="xy",
        elem_a="male_thread",
        elem_b="female_thread",
        min_overlap=0.028,
        name="threaded_parts_share_footprint",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="seal_ring",
        elem_b="mouth_gasket",
        contact_tol=5e-4,
        name="weather_seal_is_seated",
    )
    ctx.expect_gap(
        socket,
        bulb,
        axis="z",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="drip_hood",
        negative_elem="glass_shell",
        name="drip_hood_overhangs_glass_neck",
    )
    ctx.check(
        "thread_joint_axis_is_vertical",
        abs(bulb_thread.axis[0]) < 1e-9
        and abs(bulb_thread.axis[1]) < 1e-9
        and abs(abs(bulb_thread.axis[2]) - 1.0) < 1e-9,
        details=f"axis={bulb_thread.axis}",
    )
    limits = bulb_thread.motion_limits
    ctx.check(
        "thread_joint_has_realistic_turn_range",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 4.0 * math.pi,
        details=f"limits={limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
