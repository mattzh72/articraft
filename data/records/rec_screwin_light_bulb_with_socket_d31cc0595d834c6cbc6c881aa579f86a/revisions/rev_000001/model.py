from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="screw_in_light_bulb")

    porcelain = model.material("porcelain", rgba=(0.94, 0.92, 0.88, 1.0))
    socket_black = model.material("socket_black", rgba=(0.18, 0.17, 0.15, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.59, 0.30, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    glass = model.material("glass", rgba=(0.92, 0.96, 0.98, 0.42))
    contact_metal = model.material("contact_metal", rgba=(0.88, 0.76, 0.36, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    socket_body_mesh = save_mesh(
        "socket_body_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.025, 0.000),
                (0.025, 0.005),
                (0.023, 0.010),
                (0.022, 0.026),
                (0.021, 0.040),
                (0.020, 0.048),
                (0.019, 0.054),
            ],
            inner_profile=[
                (0.011, 0.004),
                (0.015, 0.008),
                (0.016, 0.028),
                (0.016, 0.050),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    socket_collar_mesh = save_mesh(
        "socket_threaded_collar",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.0178, 0.000),
                (0.0178, 0.020),
                (0.0185, 0.024),
                (0.0192, 0.028),
            ],
            inner_profile=[
                (0.0147, 0.000),
                (0.0147, 0.026),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    bulb_glass_mesh = save_mesh(
        "bulb_glass_envelope",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.009, 0.000),
                (0.016, 0.010),
                (0.026, 0.025),
                (0.030, 0.043),
                (0.029, 0.066),
                (0.022, 0.086),
                (0.010, 0.101),
                (0.000, 0.106),
            ],
            inner_profile=[
                (0.007, 0.003),
                (0.014, 0.012),
                (0.024, 0.027),
                (0.0278, 0.043),
                (0.0268, 0.064),
                (0.020, 0.082),
                (0.008, 0.096),
                (0.000, 0.101),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    bulb_base_mesh = save_mesh(
        "bulb_threaded_base",
        LatheGeometry(
            [
                (0.0040, -0.0160),
                (0.0055, -0.0145),
                (0.0055, -0.0110),
                (0.0118, -0.0110),
                (0.0129, -0.0092),
                (0.0121, -0.0070),
                (0.0132, -0.0048),
                (0.0122, -0.0024),
                (0.0133, 0.0000),
                (0.0124, 0.0025),
                (0.0134, 0.0050),
                (0.0126, 0.0076),
                (0.0135, 0.0102),
                (0.0129, 0.0124),
                (0.0148, 0.0143),
                (0.0152, 0.0162),
                (0.0112, 0.0186),
                (0.0092, 0.0205),
                (0.0000, 0.0205),
            ],
            segments=72,
        ),
    )

    socket = model.part("socket")
    socket.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=socket_black,
        name="mount_foot",
    )
    socket.visual(
        socket_body_mesh,
        material=porcelain,
        name="socket_body_shell",
    )
    socket.visual(
        socket_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=aluminum,
        name="threaded_collar",
    )
    socket.visual(
        Box((0.011, 0.020, 0.022)),
        origin=Origin(xyz=(0.0212, 0.0, 0.040)),
        material=socket_black,
        name="support_cheek_right",
    )
    socket.visual(
        Box((0.011, 0.020, 0.022)),
        origin=Origin(xyz=(-0.0212, 0.0, 0.040)),
        material=socket_black,
        name="support_cheek_left",
    )
    socket.visual(
        Cylinder(radius=0.0032, length=0.0182),
        origin=Origin(xyz=(0.0, 0.0, 0.0151)),
        material=contact_metal,
        name="center_contact_post",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.058),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        bulb_base_mesh,
        material=brass,
        name="threaded_base",
    )
    bulb.visual(
        Cylinder(radius=0.0044, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, -0.0165)),
        material=contact_metal,
        name="base_contact_button",
    )
    bulb.visual(
        bulb_glass_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=glass,
        name="glass_envelope",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.128)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("bulb_spin")

    ctx.expect_contact(
        bulb,
        socket,
        elem_a="base_contact_button",
        elem_b="center_contact_post",
        name="bulb button stays seated on the socket center contact",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="threaded_collar",
        margin=0.0,
        name="threaded bulb base stays centered inside the socket collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="threaded_collar",
        min_gap=0.003,
        max_gap=0.006,
        name="glass envelope rises cleanly above the socket collar",
    )
    ctx.expect_gap(
        socket,
        bulb,
        axis="x",
        positive_elem="support_cheek_right",
        negative_elem="threaded_base",
        min_gap=0.0003,
        max_gap=0.0012,
        name="right support cheek closely frames the rotating bulb base",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="x",
        positive_elem="threaded_base",
        negative_elem="support_cheek_left",
        min_gap=0.0003,
        max_gap=0.0012,
        name="left support cheek closely frames the rotating bulb base",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: 1.8}):
        ctx.expect_contact(
            bulb,
            socket,
            elem_a="base_contact_button",
            elem_b="center_contact_post",
            name="center electrical contact persists while the bulb spins",
        )
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="threaded_base",
            outer_elem="threaded_collar",
            margin=0.0,
            name="rotated bulb base remains captured by the threaded collar",
        )
        rotated_pos = ctx.part_world_position(bulb)

    same_axis = (
        rest_pos is not None
        and rotated_pos is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(rest_pos, rotated_pos))
    )
    ctx.check(
        "bulb spins about a fixed socket axis",
        same_axis,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
