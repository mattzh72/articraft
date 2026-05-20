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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helical_thread(
    *,
    centerline_radius: float,
    tube_radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    phase: float = 0.0,
    points_per_turn: int = 42,
) -> object:
    steps = max(8, int(points_per_turn * turns))
    points = []
    for i in range(steps + 1):
        t = i / steps
        angle = phase + turns * 2.0 * math.pi * t
        z = z_start + (z_end - z_start) * t
        points.append(
            (
                centerline_radius * math.cos(angle),
                centerline_radius * math.sin(angle),
                z,
            )
        )
    return tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=3,
        radial_segments=12,
        cap_ends=True,
    )


def _build_bulb_base_mesh():
    core = CylinderGeometry(radius=0.0118, height=0.024).translate(0.0, 0.0, -0.0100)
    shoulder = CylinderGeometry(radius=0.0129, height=0.004).translate(0.0, 0.0, 0.0005)
    thread = _helical_thread(
        centerline_radius=0.0127,
        tube_radius=0.0010,
        z_start=0.0020,
        z_end=-0.0205,
        turns=2.7,
    )
    core.merge(shoulder)
    core.merge(thread)
    return mesh_from_geometry(core, "bulb_screw_base")


def _build_glass_envelope_mesh():
    outer_profile = [
        (0.0155, 0.0090),
        (0.0220, 0.0200),
        (0.0450, 0.0400),
        (0.0550, 0.0540),
        (0.0510, 0.0680),
        (0.0300, 0.0810),
        (0.0100, 0.0860),
        (0.0000, 0.0880),
    ]
    inner_profile = [
        (0.0138, 0.0105),
        (0.0205, 0.0210),
        (0.0425, 0.0400),
        (0.0525, 0.0540),
        (0.0485, 0.0670),
        (0.0280, 0.0790),
        (0.0085, 0.0835),
        (0.0000, 0.0850),
    ]
    envelope = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
    )
    return mesh_from_geometry(envelope, "bulb_glass_envelope")


def _build_socket_body_mesh():
    outer_profile = [
        (0.0320, 0.0080),
        (0.0320, 0.0240),
        (0.0280, 0.0310),
        (0.0225, 0.0380),
        (0.0205, 0.0430),
    ]
    inner_profile = [
        (0.0200, 0.0080),
        (0.0200, 0.0240),
        (0.0188, 0.0310),
        (0.0175, 0.0380),
        (0.0167, 0.0430),
    ]
    body = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
    )
    inner_thread = _helical_thread(
        centerline_radius=0.0159,
        tube_radius=0.0008,
        z_start=0.0420,
        z_end=0.0265,
        turns=2.0,
        phase=0.4,
    )
    body.merge(inner_thread)
    return mesh_from_geometry(body, "socket_body_and_collar")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb")

    ceramic_black = model.material("ceramic_black", rgba=(0.13, 0.13, 0.14, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.97, 0.93, 0.84, 0.42))
    brass = model.material("brass", rgba=(0.67, 0.56, 0.30, 1.0))
    nickel = model.material("nickel", rgba=(0.74, 0.75, 0.77, 1.0))
    dark_contact = model.material("dark_contact", rgba=(0.18, 0.18, 0.19, 1.0))
    insulating_black = model.material("insulating_black", rgba=(0.08, 0.08, 0.09, 1.0))

    socket_body_mesh = _build_socket_body_mesh()
    bulb_base_mesh = _build_bulb_base_mesh()
    bulb_glass_mesh = _build_glass_envelope_mesh()

    socket = model.part("socket")
    socket.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=ceramic_black,
        name="mount_flange",
    )
    socket.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_contact,
        name="rear_housing",
    )
    socket.visual(
        socket_body_mesh,
        material=ceramic_black,
        name="socket_collar",
    )
    socket.visual(
        Cylinder(radius=0.0225, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=nickel,
        name="socket_rim",
    )
    socket.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=insulating_black,
        name="socket_insulator",
    )
    socket.visual(
        Cylinder(radius=0.0045, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.0172)),
        material=brass,
        name="socket_contact",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.043),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        bulb_base_mesh,
        material=brass,
        name="bulb_screw",
    )
    bulb.visual(
        Cylinder(radius=0.0125, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=nickel,
        name="bulb_cap",
    )
    bulb.visual(
        Cylinder(radius=0.0162, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=insulating_black,
        name="bulb_neck_band",
    )
    bulb.visual(
        bulb_glass_mesh,
        material=warm_glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0042, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=dark_contact,
        name="bulb_tip",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.112, 0.112, 0.112)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
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
    bulb_spin = object_model.get_articulation("bulb_spin")

    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="bulb_screw",
        outer_elem="socket_collar",
        margin=0.0015,
        name="bulb screw stays centered in socket collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="bulb_screw",
        elem_b="socket_collar",
        min_overlap=0.015,
        name="bulb screw remains seated in threaded collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="socket_rim",
        min_gap=0.006,
        name="glass envelope clears the socket rim",
    )

    with ctx.pose({bulb_spin: math.pi * 0.75}):
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="bulb_screw",
            outer_elem="socket_collar",
            margin=0.0015,
            name="rotated bulb stays coaxial in the collar",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="bulb_screw",
            elem_b="socket_collar",
            min_overlap=0.015,
            name="rotated bulb remains seated in the collar",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
