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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helical_thread_mesh(
    name: str,
    *,
    radius: float,
    z0: float,
    z1: float,
    turns: float,
    tube_radius: float,
    phase: float = 0.0,
):
    point_count = max(32, int(turns * 28))
    points = []
    for index in range(point_count + 1):
        t = index / point_count
        angle = phase + turns * math.tau * t
        z = z0 + (z1 - z0) * t
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=tube_radius,
            samples_per_segment=3,
            radial_segments=16,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb")

    socket_plastic = model.material("socket_plastic", rgba=(0.18, 0.18, 0.18, 1.0))
    socket_metal = model.material("socket_metal", rgba=(0.60, 0.60, 0.58, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.24, 1.0))
    glass = model.material("glass", rgba=(0.92, 0.96, 1.0, 0.35))
    frosted_white = model.material("frosted_white", rgba=(0.96, 0.95, 0.92, 0.85))

    socket_body_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.019, 0.000),
                (0.027, 0.006),
                (0.029, 0.016),
                (0.029, 0.036),
                (0.026, 0.042),
                (0.021, 0.046),
            ],
            [
                (0.010, 0.000),
                (0.012, 0.006),
                (0.019, 0.016),
                (0.019, 0.036),
                (0.017, 0.042),
                (0.013, 0.046),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "socket_body_mesh",
    )
    collar_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0205, 0.044),
                (0.0240, 0.050),
                (0.0240, 0.080),
                (0.0224, 0.082),
            ],
            [
                (0.0168, 0.044),
                (0.0183, 0.050),
                (0.0183, 0.078),
                (0.0172, 0.082),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "socket_collar_shell",
    )
    inner_thread_mesh = _helical_thread_mesh(
        "socket_inner_thread",
        radius=0.0172,
        z0=0.048,
        z1=0.078,
        turns=2.1,
        tube_radius=0.00105,
        phase=0.35,
    )

    socket = model.part("socket")
    socket.visual(
        socket_body_mesh,
        material=socket_plastic,
        name="socket_body",
    )
    socket.visual(
        collar_shell_mesh,
        material=socket_metal,
        name="threaded_collar",
    )
    socket.visual(
        inner_thread_mesh,
        material=socket_metal,
        name="inner_thread",
    )
    socket.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.0, 0.000),
                    (0.010, 0.000),
                    (0.012, 0.006),
                    (0.014, 0.016),
                    (0.012, 0.026),
                    (0.010, 0.034),
                    (0.0, 0.034),
                ],
                segments=48,
            ),
            "socket_contact_insulator",
        ),
        material=frosted_white,
        name="contact_insulator",
    )
    socket.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=brass,
        name="center_contact",
    )
    socket.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.084)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    glass_envelope_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.168),
                (0.004, 0.166),
                (0.012, 0.158),
                (0.022, 0.142),
                (0.030, 0.120),
                (0.0325, 0.100),
                (0.0315, 0.082),
                (0.0260, 0.066),
                (0.0180, 0.060),
                (0.0120, 0.056),
                (0.0, 0.056),
            ],
            segments=72,
        ),
        "bulb_glass_envelope",
    )
    metal_base_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.020),
                (0.007, 0.020),
                (0.011, 0.022),
                (0.0144, 0.024),
                (0.0144, 0.050),
                (0.0132, 0.056),
                (0.0118, 0.060),
                (0.0, 0.060),
            ],
            segments=64,
        ),
        "bulb_metal_base",
    )
    outer_thread_mesh = _helical_thread_mesh(
        "bulb_outer_thread",
        radius=0.01455,
        z0=0.026,
        z1=0.048,
        turns=2.1,
        tube_radius=0.00085,
        phase=0.10,
    )

    bulb = model.part("bulb")
    bulb.visual(
        glass_envelope_mesh,
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=frosted_white,
        name="insulator_neck",
    )
    bulb.visual(
        metal_base_mesh,
        material=socket_metal,
        name="threaded_base",
    )
    bulb.visual(
        outer_thread_mesh,
        material=socket_metal,
        name="outer_thread",
    )
    bulb.visual(
        Sphere(radius=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=brass,
        name="base_tip",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.066, 0.066, 0.168)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("bulb_spin")

    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="threaded_collar",
        margin=0.009,
        name="bulb base stays centered in collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_base",
        elem_b="threaded_collar",
        min_overlap=0.020,
        name="threaded base remains seated in collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="threaded_collar",
        min_gap=0.010,
        name="glass envelope clears collar rim",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="base_tip",
        elem_b="center_contact",
        contact_tol=0.001,
        name="bulb tip meets socket contact",
    )
    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(bulb)
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="threaded_base",
            outer_elem="threaded_collar",
            margin=0.009,
            name="rotated bulb base stays centered in collar",
        )
    ctx.check(
        "bulb rotates about a fixed shared axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
