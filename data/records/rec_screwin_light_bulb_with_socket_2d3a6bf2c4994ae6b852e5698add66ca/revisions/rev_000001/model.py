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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_in_socket")

    bakelite = model.material("bakelite", rgba=(0.16, 0.10, 0.07, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.71, 0.73, 1.0))
    aluminum = model.material("aluminum", rgba=(0.82, 0.83, 0.84, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.64, 0.31, 1.0))
    porcelain = model.material("porcelain", rgba=(0.93, 0.92, 0.88, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.97, 0.96, 0.92, 0.72))

    socket_body_mesh = _mesh(
        "socket_body_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.026, 0.000),
                (0.031, 0.010),
                (0.031, 0.022),
                (0.030, 0.034),
                (0.027, 0.046),
                (0.024, 0.050),
            ],
            [
                (0.000, 0.003),
                (0.010, 0.012),
                (0.012, 0.022),
                (0.014, 0.034),
                (0.0155, 0.045),
                (0.0160, 0.049),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    socket_collar_mesh = _mesh(
        "socket_collar_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0205, 0.028),
                (0.0215, 0.036),
                (0.0215, 0.049),
                (0.0225, 0.056),
            ],
            [
                (0.0148, 0.029),
                (0.0150, 0.037),
                (0.0150, 0.048),
                (0.0154, 0.055),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    glass_mesh = _mesh(
        "bulb_glass_envelope",
        LatheGeometry.from_shell_profiles(
            [
                (0.0115, 0.010),
                (0.0140, 0.017),
                (0.0220, 0.030),
                (0.0310, 0.050),
                (0.0330, 0.068),
                (0.0280, 0.088),
                (0.0120, 0.106),
                (0.0000, 0.112),
            ],
            [
                (0.0098, 0.011),
                (0.0120, 0.018),
                (0.0195, 0.031),
                (0.0288, 0.050),
                (0.0305, 0.067),
                (0.0260, 0.087),
                (0.0105, 0.103),
                (0.0000, 0.109),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    socket = model.part("socket")
    socket.visual(socket_body_mesh, material=bakelite, name="socket_body")
    socket.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=bakelite,
        name="mounting_shoulder",
    )
    socket.visual(socket_collar_mesh, material=steel, name="socket_collar")
    socket.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brass,
        name="socket_contact",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.056),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0132, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=aluminum,
        name="threaded_shell",
    )
    for ridge_z in (-0.004, -0.009, -0.014, -0.019, -0.024):
        bulb.visual(
            Cylinder(radius=0.0142, length=0.0028),
            origin=Origin(xyz=(0.0, 0.0, ridge_z)),
            material=aluminum,
        )
    bulb.visual(
        Cylinder(radius=0.0158, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=aluminum,
        name="base_shoulder",
    )
    bulb.visual(
        Cylinder(radius=0.0116, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=porcelain,
        name="neck_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0050, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=porcelain,
        name="led_stem",
    )
    bulb.visual(glass_mesh, material=frosted_glass, name="glass_envelope")
    bulb.visual(
        Cylinder(radius=0.0038, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, -0.0272)),
        material=brass,
        name="tip_contact",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.140)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
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
        inner_elem="threaded_shell",
        outer_elem="socket_collar",
        margin=0.001,
        name="threaded shell stays centered in socket collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_shell",
        elem_b="socket_collar",
        min_overlap=0.020,
        name="threaded shell remains inserted in collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="socket_collar",
        min_gap=0.003,
        max_gap=0.020,
        name="glass envelope sits above the socket collar",
    )

    with ctx.pose({bulb_spin: 1.7 * math.pi}):
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="threaded_shell",
            outer_elem="socket_collar",
            margin=0.001,
            name="threaded shell stays centered while rotated",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="threaded_shell",
            elem_b="socket_collar",
            min_overlap=0.020,
            name="threaded shell stays engaged while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
