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


def _thread_profile(radius_root: float, radius_crest: float, z_start: float, pitch: float, turns: int):
    profile = [(radius_root, z_start)]
    z = z_start
    for _ in range(turns):
        profile.extend(
            [
                (radius_root, z + pitch * 0.10),
                (radius_crest, z + pitch * 0.40),
                (radius_root, z + pitch * 0.82),
            ]
        )
        z += pitch
    profile.append((radius_root, z))
    return profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="light_bulb_in_socket")

    socket_black = model.material("socket_black", rgba=(0.12, 0.10, 0.08, 1.0))
    ceramic_ivory = model.material("ceramic_ivory", rgba=(0.91, 0.88, 0.79, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.28, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.96, 0.97, 0.92, 0.42))
    tungsten = model.material("tungsten", rgba=(0.45, 0.34, 0.16, 1.0))

    socket = model.part("socket_body")
    socket.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=socket_black,
        name="wire_entry",
    )
    socket_shell_outer = [(0.0210, 0.0000), (0.0210, 0.0220), (0.0240, 0.0270), (0.0240, 0.0340)]
    socket_shell_inner = [(0.0176, 0.0006), (0.0176, 0.0308)]
    socket.visual(
        _mesh(
            "socket_shell",
            LatheGeometry.from_shell_profiles(
                socket_shell_outer,
                socket_shell_inner,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=socket_black,
        name="socket_shell",
    )
    socket.visual(
        Cylinder(radius=0.0168, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=ceramic_ivory,
        name="insulator_floor",
    )
    socket.visual(
        Cylinder(radius=0.0045, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=brass,
        name="center_contact",
    )

    collar_outer = [(0.0177, 0.0042), (0.0177, 0.0290)]
    collar_inner = _thread_profile(0.0146, 0.0140, 0.0046, 0.0056, 4)
    socket.visual(
        _mesh(
            "threaded_collar",
            LatheGeometry.from_shell_profiles(
                collar_outer,
                collar_inner,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=brass,
        name="threaded_collar",
    )
    socket.inertial = Inertial.from_geometry(
        Box((0.048, 0.048, 0.044)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0036, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0009)),
        material=brass,
        name="contact_button",
    )
    bulb.visual(
        Cylinder(radius=0.0066, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0027)),
        material=ceramic_ivory,
        name="base_insulator",
    )

    screw_outer = _thread_profile(0.0118, 0.0129, 0.0044, 0.0056, 4)
    screw_inner = [(0.0105, 0.0048), (0.0105, 0.0278)]
    bulb.visual(
        _mesh(
            "threaded_shell",
            LatheGeometry.from_shell_profiles(
                screw_outer,
                screw_inner,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=aluminum,
        name="threaded_shell",
    )
    bulb.visual(
        Cylinder(radius=0.0122, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0282)),
        material=aluminum,
        name="shell_neck_band",
    )

    neck_outer = [(0.0119, 0.0268), (0.0124, 0.0318), (0.0134, 0.0372), (0.0142, 0.0404)]
    neck_inner = [(0.0096, 0.0272), (0.0100, 0.0318), (0.0113, 0.0378), (0.0120, 0.0404)]
    bulb.visual(
        _mesh(
            "glass_neck",
            LatheGeometry.from_shell_profiles(
                neck_outer,
                neck_inner,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=warm_glass,
        name="glass_neck",
    )

    body_outer = [
        (0.0140, 0.0368),
        (0.0200, 0.0450),
        (0.0280, 0.0540),
        (0.0300, 0.0660),
        (0.0280, 0.0740),
        (0.0220, 0.0830),
        (0.0100, 0.0910),
        (0.0012, 0.0940),
    ]
    body_inner = [
        (0.0116, 0.0376),
        (0.0182, 0.0450),
        (0.0256, 0.0540),
        (0.0276, 0.0650),
        (0.0254, 0.0730),
        (0.0198, 0.0820),
        (0.0080, 0.0900),
        (0.0006, 0.0922),
    ]
    bulb.visual(
        _mesh(
            "glass_body",
            LatheGeometry.from_shell_profiles(
                body_outer,
                body_inner,
                segments=80,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=warm_glass,
        name="glass_body",
    )

    bulb.visual(
        Cylinder(radius=0.0044, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=ceramic_ivory,
        name="mount_stem",
    )
    bulb.visual(
        Cylinder(radius=0.0132, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0386)),
        material=ceramic_ivory,
        name="stem_flare",
    )
    bulb.visual(
        Cylinder(radius=0.0016, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=tungsten,
        name="support_rod",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.094)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    socket = object_model.get_part("socket_body")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("bulb_spin")

    limits = spin.motion_limits
    ctx.check(
        "bulb uses continuous screw rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={spin.articulation_type}, limits={limits}",
    )

    rest_pos = ctx.part_world_position(bulb)
    ctx.expect_overlap(
        bulb,
        socket,
        axes="xy",
        elem_a="threaded_shell",
        elem_b="threaded_collar",
        min_overlap=0.024,
        name="bulb screw shell stays concentric with socket collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="contact_button",
        negative_elem="center_contact",
        min_gap=0.0002,
        max_gap=0.0012,
        name="bulb contact button sits just above socket contact",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_body",
        negative_elem="socket_shell",
        min_gap=0.0055,
        name="glass bulb body rises above the socket shoulder",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(bulb)
        ctx.expect_overlap(
            bulb,
            socket,
            axes="xy",
            elem_a="threaded_shell",
            elem_b="threaded_collar",
            min_overlap=0.024,
            name="rotated bulb remains centered in the collar",
        )
        ctx.check(
            "bulb spins in place about the shared axis",
            rest_pos is not None
            and spun_pos is not None
            and abs(rest_pos[0] - spun_pos[0]) < 1e-9
            and abs(rest_pos[1] - spun_pos[1]) < 1e-9
            and abs(rest_pos[2] - spun_pos[2]) < 1e-9,
            details=f"rest={rest_pos}, spun={spun_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
