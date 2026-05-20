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
    TorusGeometry,
    mesh_from_geometry,
)


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 64,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb")

    socket_black = model.material("socket_black", rgba=(0.12, 0.11, 0.10, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.71, 0.74, 1.0))
    brass = model.material("brass", rgba=(0.70, 0.58, 0.28, 1.0))
    glass = model.material("glass", rgba=(0.92, 0.95, 0.98, 0.40))

    lower_socket_shell = _shell_mesh(
        "lower_socket_shell",
        outer_profile=[
            (0.0210, 0.0000),
            (0.0210, 0.0040),
            (0.0200, 0.0160),
        ],
        inner_profile=[
            (0.0160, 0.0025),
            (0.0160, 0.0160),
        ],
        segments=56,
    )
    collar_shell = _shell_mesh(
        "collar_shell",
        outer_profile=[
            (0.0185, 0.0000),
            (0.0185, 0.0240),
        ],
        inner_profile=[
            (0.0147, 0.0015),
            (0.0147, 0.0240),
        ],
    )
    bulb_glass = _shell_mesh(
        "bulb_glass",
        outer_profile=[
            (0.0128, 0.0260),
            (0.0180, 0.0340),
            (0.0258, 0.0500),
            (0.0298, 0.0680),
            (0.0280, 0.0840),
            (0.0190, 0.1000),
            (0.0070, 0.1100),
            (0.0000, 0.1120),
        ],
        inner_profile=[
            (0.0112, 0.0280),
            (0.0163, 0.0355),
            (0.0241, 0.0515),
            (0.0277, 0.0680),
            (0.0260, 0.0830),
            (0.0180, 0.0980),
            (0.0060, 0.1070),
            (0.0000, 0.1085),
        ],
    )
    bulb_thread_band = mesh_from_geometry(
        TorusGeometry(radius=0.01275, tube=0.00060, radial_segments=14, tubular_segments=52),
        "bulb_thread_band",
    )
    collar_thread_band = mesh_from_geometry(
        TorusGeometry(radius=0.01425, tube=0.00060, radial_segments=12, tubular_segments=48),
        "collar_thread_band",
    )

    socket_body = model.part("socket_body")
    socket_body.visual(
        Cylinder(radius=0.0230, length=0.0050),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=socket_black,
        name="mount_flange",
    )
    socket_body.visual(
        lower_socket_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=socket_black,
        name="socket_cup",
    )
    socket_body.visual(
        collar_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0180)),
        material=brass,
        name="threaded_collar",
    )
    for index, z_pos in enumerate((0.0240, 0.0290, 0.0340, 0.0390)):
        socket_body.visual(
            collar_thread_band,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=brass,
            name=f"collar_thread_{index}",
        )
    socket_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0230, length=0.0420),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.0210)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.01225, length=0.0260),
        origin=Origin(xyz=(0.0, 0.0, 0.0130)),
        material=aluminum,
        name="screw_shell",
    )
    for index, z_pos in enumerate((0.0040, 0.0085, 0.0130, 0.0175, 0.0220)):
        bulb.visual(
            bulb_thread_band,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=aluminum,
            name=f"thread_band_{index}",
        )
    bulb.visual(
        Cylinder(radius=0.0128, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0280)),
        material=aluminum,
        name="base_neck",
    )
    bulb.visual(
        Cylinder(radius=0.0168, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0270)),
        material=aluminum,
        name="seat_shoulder",
    )
    bulb.visual(
        bulb_glass,
        material=glass,
        name="glass_envelope",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.112)),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket_body,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket_body = object_model.get_part("socket_body")
    bulb = object_model.get_part("bulb")
    bulb_spin = object_model.get_articulation("bulb_spin")

    limits = bulb_spin.motion_limits
    ctx.check(
        "bulb uses a continuous spin joint on the socket axis",
        bulb_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(bulb_spin.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={bulb_spin.joint_type}, axis={bulb_spin.axis}, "
            f"limits=({None if limits is None else limits.lower}, "
            f"{None if limits is None else limits.upper})"
        ),
    )

    ctx.expect_within(
        bulb,
        socket_body,
        axes="xy",
        inner_elem="screw_shell",
        outer_elem="threaded_collar",
        margin=0.0,
        name="threaded bulb shell stays centered in the collar",
    )
    ctx.expect_overlap(
        bulb,
        socket_body,
        axes="z",
        elem_a="screw_shell",
        elem_b="threaded_collar",
        min_overlap=0.022,
        name="threaded shell remains substantially engaged in the socket",
    )
    ctx.expect_contact(
        bulb,
        socket_body,
        elem_a="seat_shoulder",
        elem_b="threaded_collar",
        name="bulb shoulder seats on the collar rim",
    )
    ctx.expect_gap(
        bulb,
        socket_body,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="threaded_collar",
        min_gap=0.0,
        max_gap=0.001,
        name="glass envelope starts immediately above the collar",
    )

    with ctx.pose({bulb_spin: math.pi / 2.0}):
        ctx.expect_within(
            bulb,
            socket_body,
            axes="xy",
            inner_elem="screw_shell",
            outer_elem="threaded_collar",
            margin=0.0,
            name="rotated bulb shell stays centered in the collar",
        )
        ctx.expect_contact(
            bulb,
            socket_body,
            elem_a="seat_shoulder",
            elem_b="threaded_collar",
            name="shoulder remains seated while the bulb rotates",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
