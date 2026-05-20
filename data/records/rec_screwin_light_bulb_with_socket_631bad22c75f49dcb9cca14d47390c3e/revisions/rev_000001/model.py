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
    model = ArticulatedObject(name="screw_in_light_bulb")

    glass = model.material("glass", rgba=(0.97, 0.95, 0.89, 0.45))
    aluminum = model.material("aluminum", rgba=(0.70, 0.71, 0.74, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.28, 1.0))
    phenolic = model.material("phenolic", rgba=(0.14, 0.10, 0.08, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    socket = model.part("socket")

    socket_body_outer = [
        (0.015, -0.045),
        (0.022, -0.041),
        (0.022, -0.014),
        (0.021, 0.000),
        (0.019, 0.008),
    ]
    socket_body_inner = [
        (0.007, -0.041),
        (0.0158, -0.030),
        (0.0162, -0.010),
        (0.0162, 0.002),
        (0.0154, 0.006),
    ]
    socket_body_mesh = save_mesh(
        "socket_body_shell",
        LatheGeometry.from_shell_profiles(
            socket_body_outer,
            socket_body_inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    socket.visual(
        socket_body_mesh,
        material=phenolic,
        name="socket_body",
    )

    collar_outer = [
        (0.0162, -0.024),
        (0.0162, 0.006),
    ]
    collar_inner = [
        (0.0147, -0.024),
        (0.0138, -0.020),
        (0.0147, -0.016),
        (0.0139, -0.012),
        (0.0147, -0.008),
        (0.0139, -0.004),
        (0.0146, 0.000),
        (0.0138, 0.004),
        (0.0146, 0.006),
    ]
    collar_mesh = save_mesh(
        "threaded_collar",
        LatheGeometry.from_shell_profiles(
            collar_outer,
            collar_inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
    )
    socket.visual(
        collar_mesh,
        material=aluminum,
        name="threaded_collar",
    )
    socket_contact_insulator = save_mesh(
        "socket_contact_insulator",
        LatheGeometry.from_shell_profiles(
            [
                (0.0076, -0.0407),
                (0.0076, -0.0340),
                (0.0094, -0.0290),
            ],
            [
                (0.0046, -0.0405),
                (0.0046, -0.0292),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
    )
    socket.visual(
        socket_contact_insulator,
        material=phenolic,
        name="socket_contact_insulator",
    )
    socket.visual(
        Cylinder(radius=0.0046, length=0.0122),
        origin=Origin(xyz=(0.0, 0.0, -0.0351)),
        material=brass,
        name="socket_contact",
    )
    socket.inertial = Inertial.from_geometry(
        Box((0.044, 0.044, 0.053)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.0185)),
    )

    bulb = model.part("bulb")

    glass_outer = [
        (0.0082, 0.010),
        (0.0120, 0.018),
        (0.0205, 0.037),
        (0.0275, 0.058),
        (0.0300, 0.073),
        (0.0285, 0.084),
        (0.0210, 0.097),
        (0.0100, 0.105),
        (0.0020, 0.108),
    ]
    glass_inner = [
        (0.0068, 0.009),
        (0.0105, 0.018),
        (0.0180, 0.036),
        (0.0245, 0.057),
        (0.0270, 0.072),
        (0.0258, 0.082),
        (0.0190, 0.094),
        (0.0090, 0.102),
        (0.0012, 0.105),
    ]
    glass_mesh = save_mesh(
        "glass_envelope",
        LatheGeometry.from_shell_profiles(
            glass_outer,
            glass_inner,
            segments=88,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ),
    )
    bulb.visual(
        glass_mesh,
        material=glass,
        name="glass_envelope",
    )

    metal_base_profile = [
        (0.0, -0.026),
        (0.0052, -0.026),
        (0.0102, -0.0248),
        (0.0122, -0.0223),
        (0.0132, -0.0194),
        (0.0120, -0.0158),
        (0.0131, -0.0122),
        (0.0120, -0.0086),
        (0.0130, -0.0050),
        (0.0116, -0.0010),
        (0.0106, 0.0038),
        (0.0091, 0.0088),
        (0.0078, 0.0120),
        (0.0, 0.0120),
    ]
    metal_base_mesh = save_mesh(
        "metal_base",
        LatheGeometry(metal_base_profile, segments=72),
    )
    bulb.visual(
        metal_base_mesh,
        material=aluminum,
        name="metal_base",
    )
    bulb.visual(
        Cylinder(radius=0.0046, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, -0.0270)),
        material=brass,
        name="contact_button",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.134),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    bulb_spin = object_model.get_articulation("bulb_spin")

    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="metal_base",
        outer_elem="threaded_collar",
        margin=0.0009,
        name="bulb base stays centered inside the threaded collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="metal_base",
        elem_b="threaded_collar",
        min_overlap=0.024,
        name="bulb base remains seated in the collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="socket_body",
        min_gap=0.0007,
        max_gap=0.006,
        name="glass envelope starts just above the socket rim",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({bulb_spin: 1.7}):
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="metal_base",
            outer_elem="threaded_collar",
            margin=0.0009,
            name="rotated bulb base stays centered inside the collar",
        )
        rotated_pos = ctx.part_world_position(bulb)

    ctx.check(
        "continuous rotation keeps bulb seated on the shared axis",
        rest_pos is not None
        and rotated_pos is not None
        and abs(rest_pos[0] - rotated_pos[0]) <= 1e-6
        and abs(rest_pos[1] - rotated_pos[1]) <= 1e-6
        and abs(rest_pos[2] - rotated_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
