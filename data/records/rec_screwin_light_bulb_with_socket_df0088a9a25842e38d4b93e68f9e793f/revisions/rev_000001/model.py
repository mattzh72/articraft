from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def _build_bulb_envelope():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0115, 0.031),
            (0.0175, 0.039),
            (0.0245, 0.052),
            (0.0300, 0.066),
            (0.0315, 0.078),
            (0.0285, 0.091),
            (0.0225, 0.103),
            (0.0110, 0.114),
            (0.0, 0.118),
        ],
        [
            (0.0098, 0.033),
            (0.0158, 0.040),
            (0.0225, 0.053),
            (0.0273, 0.066),
            (0.0285, 0.077),
            (0.0260, 0.089),
            (0.0200, 0.100),
            (0.0095, 0.110),
            (0.0, 0.114),
        ],
        segments=72,
    )


def _build_bulb_neck():
    return LatheGeometry(
        [
            (0.0, 0.027),
            (0.0106, 0.027),
            (0.0124, 0.030),
            (0.0130, 0.033),
            (0.0118, 0.036),
            (0.0, 0.036),
        ],
        segments=56,
    )


def _build_bulb_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0118, 0.002),
            (0.0125, 0.004),
            (0.0132, 0.006),
            (0.0127, 0.008),
            (0.0133, 0.010),
            (0.0128, 0.012),
            (0.0134, 0.014),
            (0.0128, 0.016),
            (0.0133, 0.018),
            (0.0128, 0.020),
            (0.0132, 0.022),
            (0.0127, 0.024),
            (0.0131, 0.026),
            (0.0126, 0.028),
            (0.0125, 0.030),
        ],
        [
            (0.0108, 0.004),
            (0.0110, 0.011),
            (0.0113, 0.019),
            (0.0116, 0.030),
        ],
        segments=64,
    )


def _build_socket_body():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0180, 0.000),
            (0.0215, 0.003),
            (0.0248, 0.010),
            (0.0240, 0.019),
            (0.0218, 0.023),
            (0.0194, 0.026),
        ],
        [
            (0.0090, 0.006),
            (0.0162, 0.010),
            (0.0165, 0.020),
            (0.0161, 0.026),
        ],
        segments=64,
    )


def _build_socket_collar():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0166, 0.008),
            (0.0169, 0.010),
            (0.0171, 0.018),
            (0.0169, 0.023),
            (0.0163, 0.026),
        ],
        [
            (0.0156, 0.008),
            (0.0149, 0.010),
            (0.0155, 0.012),
            (0.0148, 0.014),
            (0.0155, 0.016),
            (0.0148, 0.018),
            (0.0154, 0.020),
            (0.0147, 0.022),
            (0.0153, 0.024),
            (0.0149, 0.026),
        ],
        segments=64,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="light_bulb_in_socket")

    ceramic = model.material("ceramic", rgba=(0.89, 0.86, 0.79, 1.0))
    glass = model.material("glass", rgba=(0.93, 0.96, 1.0, 0.38))
    threaded_metal = model.material("threaded_metal", rgba=(0.72, 0.72, 0.70, 1.0))
    dark_contact = model.material("dark_contact", rgba=(0.18, 0.18, 0.18, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.61, 0.28, 1.0))

    socket = model.part("socket")
    socket.visual(
        _mesh("socket_body", _build_socket_body()),
        material=ceramic,
        name="socket_body",
    )
    socket.visual(
        _mesh("socket_collar", _build_socket_collar()),
        material=threaded_metal,
        name="socket_collar",
    )
    socket.visual(
        Cylinder(radius=0.0090, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=brass,
        name="socket_contact",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.030),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        _mesh("glass_envelope", _build_bulb_envelope()),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        _mesh("bulb_neck", _build_bulb_neck()),
        material=dark_contact,
        name="glass_neck",
    )
    bulb.visual(
        _mesh("threaded_shell", _build_bulb_shell()),
        material=threaded_metal,
        name="threaded_shell",
    )
    bulb.visual(
        Cylinder(radius=0.0084, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.00175)),
        material=dark_contact,
        name="shell_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0112, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00475)),
        material=dark_contact,
        name="insulator_shoulder",
    )
    bulb.visual(
        Cylinder(radius=0.0032, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.00175)),
        material=brass,
        name="bottom_contact",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.118),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("bulb_spin")

    ctx.expect_origin_distance(
        bulb,
        socket,
        axes="xy",
        max_dist=1e-6,
        name="bulb stays centered on the socket axis",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_shell",
        outer_elem="socket_collar",
        margin=0.0012,
        name="threaded shell stays nested inside the collar footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_shell",
        elem_b="socket_collar",
        min_overlap=0.015,
        name="bulb remains seated in the socket collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="socket_body",
        min_gap=0.005,
        max_gap=0.020,
        name="glass envelope begins above the socket body",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            bulb,
            socket,
            axes="xy",
            max_dist=1e-6,
            name="rotated bulb still shares the socket axis",
        )
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="threaded_shell",
            outer_elem="socket_collar",
            margin=0.0012,
            name="rotated threaded shell stays inside the collar footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
