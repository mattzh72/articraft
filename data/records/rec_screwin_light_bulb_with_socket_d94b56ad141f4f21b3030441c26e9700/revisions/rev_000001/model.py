from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _annulus_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb")

    phenolic = model.material("phenolic", rgba=(0.14, 0.12, 0.10, 1.0))
    ceramic = model.material("ceramic", rgba=(0.93, 0.91, 0.86, 1.0))
    socket_metal = model.material("socket_metal", rgba=(0.67, 0.65, 0.60, 1.0))
    bulb_metal = model.material("bulb_metal", rgba=(0.72, 0.70, 0.66, 1.0))
    bulb_glass = model.material("bulb_glass", rgba=(0.96, 0.96, 0.91, 0.35))
    filament = model.material("filament", rgba=(1.0, 0.77, 0.36, 1.0))

    socket = model.part("socket")
    socket.visual(
        _annulus_mesh("socket_mount_flange", 0.023, 0.0105, 0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=phenolic,
        name="mount_flange",
    )
    socket.visual(
        _annulus_mesh("socket_body_shell", 0.020, 0.0115, 0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=phenolic,
        name="socket_body_shell",
    )
    socket.visual(
        _annulus_mesh("socket_shoulder_ring", 0.018, 0.0100, 0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=ceramic,
        name="socket_shoulder_ring",
    )
    socket.visual(
        _annulus_mesh("socket_collar", 0.0155, 0.0093, 0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=socket_metal,
        name="socket_collar",
    )
    socket.visual(
        _annulus_mesh("socket_lip", 0.0168, 0.0105, 0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0655)),
        material=socket_metal,
        name="socket_lip",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.070),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0022, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=socket_metal,
        name="center_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0066, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=bulb_metal,
        name="bulb_thread_shell",
    )
    for index, z_pos in enumerate((0.005, 0.0085, 0.012, 0.0155, 0.019, 0.0225)):
        bulb.visual(
            Cylinder(radius=0.0076, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=bulb_metal,
            name=f"thread_ridge_{index}",
        )
    bulb.visual(
        Cylinder(radius=0.0084, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=bulb_metal,
        name="neck_collar",
    )
    bulb.visual(
        Cylinder(radius=0.0060, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=ceramic,
        name="insulator_neck",
    )
    bulb.visual(
        Cylinder(radius=0.0063, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=bulb_glass,
        name="glass_seal",
    )

    envelope_outer = [
        (0.0065, 0.0),
        (0.0090, 0.008),
        (0.0120, 0.024),
        (0.0148, 0.048),
        (0.0162, 0.074),
        (0.0154, 0.094),
        (0.0122, 0.112),
        (0.0080, 0.123),
        (0.0020, 0.129),
        (0.0, 0.131),
    ]
    envelope_inner = [
        (0.0052, 0.002),
        (0.0075, 0.010),
        (0.0103, 0.025),
        (0.0129, 0.048),
        (0.0141, 0.073),
        (0.0133, 0.092),
        (0.0105, 0.109),
        (0.0068, 0.120),
        (0.0012, 0.126),
    ]
    bulb.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                envelope_outer,
                envelope_inner,
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
            "bulb_envelope",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        material=bulb_glass,
        name="bulb_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0010, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=socket_metal,
        name="support_stem",
    )
    bulb.visual(
        Cylinder(radius=0.0016, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=filament,
        name="light_core",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.017, length=0.165),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("socket_to_bulb")

    with ctx.pose({spin: 0.0}):
        ctx.expect_overlap(
            bulb,
            socket,
            axes="xy",
            elem_a="bulb_thread_shell",
            elem_b="socket_collar",
            min_overlap=0.012,
            name="threaded base stays coaxial with the socket collar",
        )
        ctx.expect_gap(
            bulb,
            socket,
            axis="z",
            positive_elem="bulb_envelope",
            negative_elem="socket_lip",
            min_gap=0.001,
            max_gap=0.004,
            name="glass envelope rises cleanly above the socket lip",
        )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(bulb)
        ctx.expect_overlap(
            bulb,
            socket,
            axes="xy",
            elem_a="bulb_thread_shell",
            elem_b="socket_collar",
            min_overlap=0.012,
            name="rotated bulb remains seated in the collar",
        )

    ctx.check(
        "bulb spins about the shared socket axis without shifting",
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
