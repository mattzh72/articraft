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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_in_socket")

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    socket_black = model.material("socket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    ceramic_white = model.material("ceramic_white", rgba=(0.93, 0.92, 0.88, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.28, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.75, 0.78, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.95, 0.96, 0.98, 0.60))

    socket = model.part("socket")
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.055),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    socket_body_outer = [
        (0.0210, 0.0000),
        (0.0225, 0.0060),
        (0.0233, 0.0200),
        (0.0238, 0.0440),
        (0.0250, 0.0550),
    ]
    socket_body_inner = [
        (0.0170, 0.0050),
        (0.0183, 0.0120),
        (0.0190, 0.0440),
        (0.0195, 0.0550),
    ]
    socket.visual(
        _mesh(
            "socket_body_shell",
            LatheGeometry.from_shell_profiles(
                socket_body_outer,
                socket_body_inner,
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=socket_black,
        name="socket_body_shell",
    )

    collar_outer = [
        (0.0178, 0.0000),
        (0.0178, 0.0330),
    ]
    collar_inner = [
        (0.0158, 0.0000),
        (0.0148, 0.0045),
        (0.0157, 0.0090),
        (0.0147, 0.0135),
        (0.0156, 0.0180),
        (0.0146, 0.0225),
        (0.0155, 0.0270),
        (0.0148, 0.0315),
        (0.0152, 0.0330),
    ]
    socket.visual(
        _mesh(
            "socket_threaded_collar",
            LatheGeometry.from_shell_profiles(
                collar_outer,
                collar_inner,
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0100)),
        material=brass,
        name="threaded_collar",
    )

    throat_outer = [
        (0.0192, 0.0000),
        (0.0202, 0.0120),
    ]
    throat_inner = [
        (0.0148, 0.0000),
        (0.0148, 0.0120),
    ]
    socket.visual(
        _mesh(
            "socket_throat_ring",
            LatheGeometry.from_shell_profiles(
                throat_outer,
                throat_inner,
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0430)),
        material=ceramic_white,
        name="socket_throat_ring",
    )

    bulb = model.part("bulb")
    bulb.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.149)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.0745)),
    )

    bulb_thread_outer = [
        (0.0126, 0.0020),
        (0.0137, 0.0060),
        (0.0127, 0.0110),
        (0.0136, 0.0160),
        (0.0127, 0.0210),
        (0.0135, 0.0260),
        (0.0126, 0.0310),
        (0.0134, 0.0360),
        (0.0125, 0.0410),
        (0.0132, 0.0460),
        (0.0122, 0.0500),
    ]
    bulb_thread_inner = [
        (0.0106, 0.0040),
        (0.0106, 0.0500),
    ]
    bulb.visual(
        _mesh(
            "bulb_threaded_shell",
            LatheGeometry.from_shell_profiles(
                bulb_thread_outer,
                bulb_thread_inner,
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=aluminum,
        name="threaded_shell",
    )

    bulb.visual(
        Cylinder(radius=0.0148, length=0.0100),
        origin=Origin(xyz=(0.0, 0.0, 0.0500)),
        material=aluminum,
        name="neck_band",
    )

    glass_outer = [
        (0.0138, 0.0520),
        (0.0180, 0.0610),
        (0.0240, 0.0750),
        (0.0295, 0.0930),
        (0.0288, 0.1080),
        (0.0235, 0.1250),
        (0.0145, 0.1400),
        (0.0055, 0.1470),
        (0.0000, 0.1490),
    ]
    glass_inner = [
        (0.0114, 0.0550),
        (0.0158, 0.0630),
        (0.0218, 0.0760),
        (0.0268, 0.0930),
        (0.0260, 0.1070),
        (0.0210, 0.1230),
        (0.0126, 0.1365),
        (0.0040, 0.1450),
        (0.0000, 0.1490),
    ]
    bulb.visual(
        _mesh(
            "bulb_glass_envelope",
            LatheGeometry.from_shell_profiles(
                glass_outer,
                glass_inner,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=frosted_glass,
        name="glass_envelope",
    )

    bulb.visual(
        Cylinder(radius=0.0110, length=0.0046),
        origin=Origin(xyz=(0.0, 0.0, 0.0023)),
        material=socket_black,
        name="base_insulator",
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    bulb_spin = object_model.get_articulation("bulb_spin")

    ctx.expect_origin_distance(
        bulb,
        socket,
        axes="xy",
        max_dist=1e-6,
        name="bulb stays centered on socket axis",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="xy",
        elem_a="threaded_shell",
        elem_b="threaded_collar",
        min_overlap=0.025,
        name="threaded bulb base stays coaxial with collar footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_shell",
        elem_b="threaded_collar",
        min_overlap=0.030,
        name="threaded bulb base remains seated in the collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        min_gap=0.002,
        positive_elem="glass_envelope",
        negative_elem="socket_throat_ring",
        name="glass envelope starts above the socket throat",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({bulb_spin: 1.9}):
        spun_pos = ctx.part_world_position(bulb)
        ctx.expect_origin_distance(
            bulb,
            socket,
            axes="xy",
            max_dist=1e-6,
            name="bulb remains centered while spinning",
        )

    ctx.check(
        "continuous spin keeps bulb on the same axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) <= 1e-6
        and abs(rest_pos[1] - spun_pos[1]) <= 1e-6
        and abs(rest_pos[2] - spun_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
