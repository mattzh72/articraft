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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_thread_ridges(
    part,
    *,
    prefix: str,
    z_positions: tuple[float, ...],
    radius: float,
    length: float,
    material,
) -> None:
    for index, z in enumerate(z_positions):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=material,
            name=f"{prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb")

    socket_body = model.material("socket_body", rgba=(0.22, 0.19, 0.15, 1.0))
    socket_metal = model.material("socket_metal", rgba=(0.72, 0.70, 0.64, 1.0))
    bulb_metal = model.material("bulb_metal", rgba=(0.77, 0.75, 0.70, 1.0))
    insulator = model.material("insulator", rgba=(0.16, 0.15, 0.14, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.97, 0.95, 0.86, 0.42))

    socket = model.part("socket")
    body_outer = [
        (0.0260, -0.050),
        (0.0265, -0.034),
        (0.0245, -0.010),
        (0.0215, 0.002),
    ]
    body_inner = [
        (0.0190, -0.050),
        (0.0190, -0.030),
        (0.0188, -0.010),
        (0.0186, 0.001),
    ]
    socket.visual(
        _mesh(
            "socket_body_shell",
            LatheGeometry.from_shell_profiles(
                body_outer,
                body_inner,
                segments=56,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=socket_body,
        name="body_shell",
    )
    socket.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.0555)),
        material=socket_body,
        name="mount_flange",
    )
    socket.visual(
        Box((0.056, 0.042, 0.017)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=socket_body,
        name="mount_block",
    )
    collar_outer = [
        (0.0168, -0.024),
        (0.0176, -0.018),
        (0.0176, -0.004),
        (0.0171, 0.002),
    ]
    collar_inner = [
        (0.0146, -0.024),
        (0.0154, -0.018),
        (0.0154, -0.004),
        (0.0150, 0.0015),
    ]
    socket.visual(
        _mesh(
            "socket_threaded_collar",
            LatheGeometry.from_shell_profiles(
                collar_outer,
                collar_inner,
                segments=56,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=socket_metal,
        name="threaded_collar",
    )
    _add_thread_ridges(
        socket,
        prefix="body_rib",
        z_positions=(-0.018, -0.030, -0.042),
        radius=0.0275,
        length=0.003,
        material=socket_body,
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.078),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0132, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=bulb_metal,
        name="threaded_base",
    )
    _add_thread_ridges(
        bulb,
        prefix="thread_ridge",
        z_positions=(-0.0185, -0.0135, -0.0085, -0.0035, 0.0010),
        radius=0.0142,
        length=0.0032,
        material=bulb_metal,
    )
    bulb.visual(
        Cylinder(radius=0.0046, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0245)),
        material=bulb_metal,
        name="contact_button",
    )
    bulb.visual(
        Cylinder(radius=0.0145, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=insulator,
        name="neck_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0180, length=0.0105),
        origin=Origin(xyz=(0.0, 0.0, 0.01625)),
        material=bulb_metal,
        name="shoulder_cap",
    )
    glass_outer = [
        (0.0180, 0.000),
        (0.0290, 0.009),
        (0.0415, 0.021),
        (0.0485, 0.033),
        (0.0500, 0.044),
        (0.0450, 0.056),
        (0.0300, 0.067),
        (0.0000, 0.074),
    ]
    glass_inner = [
        (0.0150, 0.002),
        (0.0260, 0.011),
        (0.0375, 0.023),
        (0.0445, 0.034),
        (0.0460, 0.043),
        (0.0410, 0.054),
        (0.0260, 0.064),
        (0.0000, 0.070),
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
                lip_samples=8,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=warm_glass,
        name="glass_envelope",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.119)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.0350)),
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("socket_to_bulb")
    limits = spin.motion_limits

    ctx.check(
        "bulb uses a continuous spin joint on the socket axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={spin.articulation_type}, axis={spin.axis}, limits={limits}",
    )

    with ctx.pose({spin: 0.0}):
        ctx.expect_overlap(
            bulb,
            socket,
            axes="xy",
            elem_a="threaded_base",
            elem_b="threaded_collar",
            min_overlap=0.025,
            name="bulb base stays centered inside the collar footprint",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="threaded_base",
            elem_b="threaded_collar",
            min_overlap=0.023,
            name="threaded base remains deeply engaged in the collar",
        )
        ctx.expect_gap(
            bulb,
            socket,
            axis="z",
            positive_elem="neck_insulator",
            negative_elem="threaded_collar",
            min_gap=0.0005,
            max_gap=0.004,
            name="insulator neck starts just above the collar lip",
        )
        ctx.expect_gap(
            bulb,
            socket,
            axis="z",
            positive_elem="glass_envelope",
            negative_elem="threaded_collar",
            min_gap=0.014,
            max_gap=0.024,
            name="broad bulb envelope sits above the socket collar",
        )

    with ctx.pose({spin: 1.75}):
        ctx.expect_overlap(
            bulb,
            socket,
            axes="xy",
            elem_a="threaded_base",
            elem_b="threaded_collar",
            min_overlap=0.025,
            name="rotated bulb keeps the threaded base centered in the collar",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
