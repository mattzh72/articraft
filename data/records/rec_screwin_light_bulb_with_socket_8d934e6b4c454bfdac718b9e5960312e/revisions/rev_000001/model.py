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
    CylinderGeometry,
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

    porcelain = model.material("porcelain", rgba=(0.92, 0.91, 0.86, 1.0))
    socket_black = model.material("socket_black", rgba=(0.12, 0.12, 0.13, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.95, 0.94, 0.90, 0.35))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.32, 1.0))
    shell_metal = model.material("shell_metal", rgba=(0.72, 0.72, 0.74, 1.0))
    contact_metal = model.material("contact_metal", rgba=(0.82, 0.66, 0.30, 1.0))
    phenolic = model.material("phenolic", rgba=(0.30, 0.22, 0.15, 1.0))

    socket = model.part("socket_assembly")
    socket.visual(
        Cylinder(radius=0.036, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=porcelain,
        name="mounting_flange",
    )
    socket.visual(
        Cylinder(radius=0.012, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=porcelain,
        name="stem",
    )
    socket.visual(
        Cylinder(radius=0.019, length=0.023),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=socket_black,
        name="socket_body",
    )
    socket.visual(
        Cylinder(radius=0.0164, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=phenolic,
        name="insulator_core",
    )

    collar_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0198, 0.0),
            (0.0208, 0.004),
            (0.0208, 0.031),
            (0.0192, 0.033),
        ],
        [
            (0.0162, 0.0),
            (0.0162, 0.029),
            (0.0158, 0.033),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    socket.visual(
        mesh_from_geometry(collar_shell, "socket_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=shell_metal,
        name="socket_collar",
    )
    socket.visual(
        Cylinder(radius=0.0044, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0477)),
        material=contact_metal,
        name="socket_contact",
    )
    socket.inertial = Inertial.from_geometry(
        Box((0.072, 0.072, 0.078)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )

    bulb = model.part("bulb")

    screw_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0112, 0.0),
            (0.0124, 0.0014),
            (0.0128, 0.006),
            (0.0128, 0.023),
            (0.0114, 0.026),
        ],
        [
            (0.0094, 0.0),
            (0.0107, 0.0014),
            (0.0112, 0.006),
            (0.0112, 0.0225),
            (0.0099, 0.026),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    bulb.visual(
        mesh_from_geometry(screw_shell, "bulb_screw_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shell_metal,
        name="screw_shell",
    )
    bulb.visual(
        Cylinder(radius=0.0045, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0, 0.0011)),
        material=contact_metal,
        name="tip_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=phenolic,
        name="base_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0085, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=phenolic,
        name="center_insulator",
    )

    glass_envelope = LatheGeometry.from_shell_profiles(
        [
            (0.0105, 0.0),
            (0.0135, 0.006),
            (0.022, 0.018),
            (0.0295, 0.041),
            (0.0315, 0.056),
            (0.0285, 0.074),
            (0.018, 0.095),
            (0.006, 0.108),
            (0.0025, 0.114),
        ],
        [
            (0.0088, 0.0),
            (0.0115, 0.006),
            (0.0192, 0.018),
            (0.0262, 0.041),
            (0.0282, 0.056),
            (0.0252, 0.073),
            (0.0152, 0.093),
            (0.0042, 0.106),
            (0.001, 0.114),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    bulb.visual(
        mesh_from_geometry(glass_envelope, "bulb_glass_envelope"),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=warm_glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0105, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=warm_glass,
        name="glass_neck",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.145),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0468)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket_assembly")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("bulb_spin")

    ctx.expect_contact(
        bulb,
        socket,
        elem_a="tip_contact",
        elem_b="socket_contact",
        contact_tol=0.0006,
        name="bulb tip contact is seated on the socket contact",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="xy",
        elem_a="screw_shell",
        elem_b="socket_collar",
        min_overlap=0.020,
        name="screw shell remains coaxially nested in the collar footprint",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="socket_body",
        min_gap=0.020,
        name="glass envelope rises clearly above the socket body",
    )

    with ctx.pose({spin: 1.5}):
        ctx.expect_contact(
            bulb,
            socket,
            elem_a="tip_contact",
            elem_b="socket_contact",
            contact_tol=0.0006,
            name="rotated bulb still stays seated on the contact tip",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="xy",
            elem_a="screw_shell",
            elem_b="socket_collar",
            min_overlap=0.020,
            name="rotated bulb stays centered in the collar",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
