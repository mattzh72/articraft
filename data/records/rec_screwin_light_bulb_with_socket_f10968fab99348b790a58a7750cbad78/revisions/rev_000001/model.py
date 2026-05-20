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
    model = ArticulatedObject(name="light_bulb_in_socket")

    socket_body_mat = model.material("socket_body_mat", rgba=(0.14, 0.10, 0.08, 1.0))
    socket_metal_mat = model.material("socket_metal_mat", rgba=(0.72, 0.61, 0.35, 1.0))
    bulb_metal_mat = model.material("bulb_metal_mat", rgba=(0.77, 0.77, 0.79, 1.0))
    bulb_glass_mat = model.material("bulb_glass_mat", rgba=(0.97, 0.97, 0.96, 0.45))

    socket_body = model.part("socket_body")
    socket_body.visual(
        Box((0.062, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=socket_body_mat,
        name="housing",
    )
    socket_body.visual(
        Box((0.018, 0.026, 0.020)),
        origin=Origin(xyz=(-0.040, 0.0, 0.016)),
        material=socket_body_mat,
        name="cable_block",
    )
    socket_body.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(-0.053, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=socket_body_mat,
        name="cable_gland",
    )
    socket_body.visual(
        Box((0.020, 0.034, 0.010)),
        origin=Origin(xyz=(0.011, 0.0, 0.041)),
        material=socket_body_mat,
        name="tower_shoulder",
    )
    socket_body.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.0230, 0.000),
                    (0.0235, 0.005),
                    (0.0235, 0.018),
                    (0.0245, 0.027),
                    (0.0245, 0.030),
                ],
                [
                    (0.0157, 0.003),
                    (0.0149, 0.006),
                    (0.0159, 0.009),
                    (0.0149, 0.012),
                    (0.0159, 0.015),
                    (0.0149, 0.018),
                    (0.0159, 0.021),
                    (0.0149, 0.024),
                    (0.0156, 0.027),
                    (0.0156, 0.030),
                ],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            "socket_collar",
        ),
        origin=Origin(xyz=(0.011, 0.0, 0.037)),
        material=socket_metal_mat,
        name="threaded_collar",
    )
    socket_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.050, 0.074)),
        mass=0.35,
        origin=Origin(xyz=(-0.010, 0.0, 0.037)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.0137, 0.000),
                    (0.0137, 0.0015),
                    (0.0129, 0.0035),
                    (0.0141, 0.0060),
                    (0.0129, 0.0085),
                    (0.0141, 0.0110),
                    (0.0129, 0.0135),
                    (0.0140, 0.0160),
                    (0.0129, 0.0185),
                    (0.0140, 0.0210),
                    (0.0134, 0.0240),
                    (0.0134, 0.0270),
                ],
                [
                    (0.0112, 0.000),
                    (0.0112, 0.0270),
                ],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            "bulb_threaded_base",
        ),
        material=bulb_metal_mat,
        name="threaded_base",
    )
    bulb.visual(
        Cylinder(radius=0.0132, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material=socket_body_mat,
        name="insulator_collar",
    )
    bulb.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.0145, 0.000),
                    (0.0170, 0.006),
                    (0.0240, 0.020),
                    (0.0295, 0.040),
                    (0.0290, 0.058),
                    (0.0240, 0.073),
                    (0.0130, 0.083),
                    (0.0060, 0.086),
                    (0.0036, 0.088),
                ],
                [
                    (0.0115, 0.001),
                    (0.0135, 0.006),
                    (0.0205, 0.020),
                    (0.0265, 0.040),
                    (0.0260, 0.058),
                    (0.0210, 0.071),
                    (0.0110, 0.081),
                    (0.0042, 0.084),
                    (0.0018, 0.086),
                ],
                segments=56,
                start_cap="flat",
                end_cap="flat",
            ),
            "bulb_glass_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=bulb_glass_mat,
        name="glass_envelope",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.118),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    model.articulation(
        "bulb_spin",
        ArticulationType.CONTINUOUS,
        parent=socket_body,
        child=bulb,
        origin=Origin(xyz=(0.011, 0.0, 0.041)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    socket_body = object_model.get_part("socket_body")
    bulb = object_model.get_part("bulb")
    bulb_spin = object_model.get_articulation("bulb_spin")

    ctx.expect_within(
        bulb,
        socket_body,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="threaded_collar",
        margin=0.0008,
        name="bulb base stays centered in socket collar",
    )
    ctx.expect_overlap(
        bulb,
        socket_body,
        axes="z",
        elem_a="threaded_base",
        elem_b="threaded_collar",
        min_overlap=0.020,
        name="bulb base remains inserted in socket collar",
    )
    ctx.expect_gap(
        bulb,
        socket_body,
        axis="z",
        positive_elem="glass_envelope",
        negative_elem="threaded_collar",
        min_gap=0.003,
        name="glass envelope rises above threaded collar",
    )

    with ctx.pose({bulb_spin: math.pi / 2.0}):
        ctx.expect_within(
            bulb,
            socket_body,
            axes="xy",
            inner_elem="threaded_base",
            outer_elem="threaded_collar",
            margin=0.0008,
            name="rotated bulb base stays centered in socket collar",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
