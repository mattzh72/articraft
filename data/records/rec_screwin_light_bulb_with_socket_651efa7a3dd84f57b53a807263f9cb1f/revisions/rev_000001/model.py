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

    glass = model.material("glass", rgba=(0.92, 0.96, 1.0, 0.32))
    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.22, 1.0))
    ceramic = model.material("ceramic", rgba=(0.93, 0.92, 0.89, 1.0))
    bakelite = model.material("bakelite", rgba=(0.16, 0.14, 0.12, 1.0))
    contact_metal = model.material("contact_metal", rgba=(0.84, 0.69, 0.33, 1.0))

    socket_body = model.part("socket_body")
    socket_body.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=bakelite,
        name="lower_mount",
    )
    socket_body.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=bakelite,
        name="shoulder_ring",
    )
    socket_body.visual(
        Cylinder(radius=0.017, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=ceramic,
        name="insulator_core",
    )
    socket_body.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0345)),
        material=contact_metal,
        name="center_contact",
    )

    collar_outer = [
        (0.0208, 0.0),
        (0.0208, 0.020),
        (0.0196, 0.0235),
        (0.0196, 0.027),
    ]
    collar_inner = [
        (0.0148, 0.001),
        (0.0148, 0.0205),
        (0.0138, 0.0235),
        (0.0138, 0.026),
    ]
    threaded_collar_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            collar_outer,
            collar_inner,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "threaded_collar",
    )
    socket_body.visual(
        threaded_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=brass,
        name="threaded_collar",
    )
    socket_body.visual(
        Cylinder(radius=0.0185, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=brass,
        name="collar_seat_ring",
    )
    socket_body.inertial = Inertial.from_geometry(
        Box((0.048, 0.048, 0.060)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.004, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=contact_metal,
        name="center_tip",
    )
    bulb.visual(
        Cylinder(radius=0.0062, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=ceramic,
        name="base_insulator",
    )

    screw_profile = [
        (0.0, 0.004),
        (0.0117, 0.004),
        (0.0117, 0.0062),
        (0.0138, 0.0062),
        (0.0138, 0.0080),
        (0.0122, 0.0080),
        (0.0122, 0.0103),
        (0.0140, 0.0103),
        (0.0140, 0.0121),
        (0.0123, 0.0121),
        (0.0123, 0.0144),
        (0.0141, 0.0144),
        (0.0141, 0.0162),
        (0.0125, 0.0162),
        (0.0125, 0.0185),
        (0.0142, 0.0185),
        (0.0142, 0.0203),
        (0.0126, 0.0203),
        (0.0126, 0.0226),
        (0.0140, 0.0226),
        (0.0140, 0.0244),
        (0.0125, 0.0244),
        (0.0125, 0.0268),
        (0.0136, 0.0268),
        (0.0136, 0.0280),
        (0.0, 0.0280),
    ]
    screw_shell_mesh = mesh_from_geometry(LatheGeometry(screw_profile, segments=64), "screw_shell")
    bulb.visual(
        screw_shell_mesh,
        material=aluminum,
        name="screw_shell",
    )
    bulb.visual(
        Cylinder(radius=0.0142, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material=aluminum,
        name="neck_ferrule",
    )

    glass_outer = [
        (0.0145, 0.000),
        (0.0185, 0.010),
        (0.0265, 0.022),
        (0.0300, 0.033),
        (0.0290, 0.060),
        (0.0200, 0.072),
        (0.0080, 0.076),
        (0.0024, 0.078),
    ]
    glass_inner = [
        (0.0118, 0.001),
        (0.0158, 0.011),
        (0.0238, 0.023),
        (0.0272, 0.034),
        (0.0262, 0.059),
        (0.0174, 0.070),
        (0.0060, 0.0735),
        (0.0008, 0.0770),
    ]
    glass_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            glass_outer,
            glass_inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "glass_envelope",
    )
    bulb.visual(
        glass_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0146, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0310)),
        material=glass,
        name="glass_neck_bridge",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.108),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket_body,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket_body")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("bulb_spin")

    ctx.expect_overlap(
        bulb,
        socket,
        axes="xy",
        elem_a="screw_shell",
        elem_b="threaded_collar",
        min_overlap=0.026,
        name="bulb screw shell stays coaxially inside the collar footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="screw_shell",
        elem_b="threaded_collar",
        min_overlap=0.022,
        name="bulb screw shell remains inserted in the collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="threaded_collar",
        min_gap=0.0005,
        max_gap=0.0030,
        name="glass envelope starts just above the socket collar",
    )
    ctx.expect_gap(
        socket,
        bulb,
        axis="z",
        positive_elem="center_contact",
        negative_elem="center_tip",
        min_gap=0.0,
        max_gap=0.0015,
        name="socket contact sits directly beneath the bulb tip",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(bulb)
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="screw_shell",
            elem_b="threaded_collar",
            min_overlap=0.022,
            name="quarter turn keeps the bulb seated in the collar",
        )

    axis = tuple(float(v) for v in spin.axis)
    ctx.check(
        "bulb articulation is a continuous spin about the socket axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and axis == (0.0, 0.0, 1.0)
        and rest_pos is not None
        and turned_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, turned_pos)) < 1e-9,
        details=f"type={spin.articulation_type}, axis={axis}, rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
