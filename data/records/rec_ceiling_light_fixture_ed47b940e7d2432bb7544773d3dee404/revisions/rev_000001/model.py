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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_high_bay_ufo_light")

    housing_gray = model.material("housing_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    fin_gray = model.material("fin_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.90, 0.92, 0.93, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.96, 0.97, 0.98, 0.94))
    hardware_black = model.material("hardware_black", rgba=(0.10, 0.11, 0.12, 1.0))

    reflector_shell_mesh = _mesh(
        "reflector_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.100, 0.006),
                (0.145, 0.004),
                (0.225, -0.004),
                (0.315, -0.018),
                (0.382, -0.037),
                (0.396, -0.048),
            ],
            inner_profile=[
                (0.088, 0.001),
                (0.136, -0.001),
                (0.217, -0.008),
                (0.307, -0.021),
                (0.376, -0.040),
                (0.389, -0.044),
            ],
            segments=72,
        ),
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.092, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=housing_gray,
        name="mounting_plate",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=hardware_black,
        name="drop_stem",
    )
    body.visual(
        Cylinder(radius=0.116, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=housing_gray,
        name="driver_housing",
    )
    body.visual(
        Cylinder(radius=0.078, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=fin_gray,
        name="top_cap",
    )
    for index, z in enumerate((0.058, 0.036, 0.014, -0.008), start=1):
        body.visual(
            Cylinder(radius=0.134, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=fin_gray,
            name=f"cooling_fin_{index}",
        )
    body.visual(
        Cylinder(radius=0.128, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=housing_gray,
        name="lower_housing_ring",
    )
    body.visual(
        Box((0.050, 0.036, 0.024)),
        origin=Origin(xyz=(0.141, 0.0, -0.052)),
        material=hardware_black,
        name="hinge_bracket",
    )
    body.visual(
        Box((0.030, 0.012, 0.012)),
        origin=Origin(xyz=(0.169, -0.017, -0.058)),
        material=hardware_black,
        name="hinge_fork_lower",
    )
    body.visual(
        Box((0.030, 0.012, 0.012)),
        origin=Origin(xyz=(0.169, 0.017, -0.058)),
        material=hardware_black,
        name="hinge_fork_upper",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.186, -0.017, -0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_black,
        name="hinge_ear_lower",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.186, 0.017, -0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_black,
        name="hinge_ear_upper",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.40, 0.30, 0.22)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    reflector = model.part("reflector")
    reflector.visual(
        Cylinder(radius=0.0115, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_black,
        name="hinge_lug",
    )
    reflector.visual(
        Box((0.082, 0.028, 0.015)),
        origin=Origin(xyz=(-0.032, 0.0, -0.0185)),
        material=hardware_black,
        name="hinge_arm",
    )
    reflector.visual(
        Cylinder(radius=0.110, length=0.012),
        origin=Origin(xyz=(-0.186, 0.0, -0.016)),
        material=housing_gray,
        name="reflector_hub",
    )
    reflector.visual(
        reflector_shell_mesh,
        origin=Origin(xyz=(-0.186, 0.0, -0.016)),
        material=reflector_white,
        name="reflector_shell",
    )
    reflector.visual(
        Cylinder(radius=0.088, length=0.012),
        origin=Origin(xyz=(-0.186, 0.0, -0.024)),
        material=diffuser_white,
        name="light_diffuser",
    )
    reflector.inertial = Inertial.from_geometry(
        Box((0.82, 0.82, 0.10)),
        mass=4.0,
        origin=Origin(xyz=(-0.186, 0.0, -0.028)),
    )

    model.articulation(
        "body_to_reflector_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=reflector,
        origin=Origin(xyz=(0.186, 0.0, -0.058)),
        # The reflector center sits at local -X from the side hinge.
        # Using -Y makes positive motion tilt the broad disc downward
        # on its free side for beam aiming.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=0.80,
        ),
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

    body = object_model.get_part("body")
    reflector = object_model.get_part("reflector")
    hinge = object_model.get_articulation("body_to_reflector_tilt")

    ctx.expect_gap(
        body,
        reflector,
        axis="z",
        positive_elem="lower_housing_ring",
        negative_elem="reflector_shell",
        min_gap=0.004,
        max_gap=0.025,
        name="reflector disc hangs just below the lower housing",
    )
    ctx.expect_overlap(
        body,
        reflector,
        axes="xy",
        elem_a="lower_housing_ring",
        elem_b="reflector_shell",
        min_overlap=0.22,
        name="reflector disc remains centered under the cylindrical housing",
    )

    rest_aabb = ctx.part_element_world_aabb(reflector, elem="reflector_shell")
    upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None
    tilted_aabb = None
    if upper is not None:
        with ctx.pose({hinge: upper}):
            tilted_aabb = ctx.part_element_world_aabb(reflector, elem="reflector_shell")

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def _aabb_center_x(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_center_z = _aabb_center_z(rest_aabb)
    tilted_center_z = _aabb_center_z(tilted_aabb)
    rest_center_x = _aabb_center_x(rest_aabb)
    tilted_center_x = _aabb_center_x(tilted_aabb)

    ctx.check(
        "reflector tilt lowers the broad disc for beam aiming",
        rest_center_z is not None
        and tilted_center_z is not None
        and tilted_center_z < rest_center_z - 0.035,
        details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
    )
    ctx.check(
        "reflector tilt swings the disc toward the hinge side",
        rest_center_x is not None
        and tilted_center_x is not None
        and tilted_center_x > rest_center_x + 0.030,
        details=f"rest_center_x={rest_center_x}, tilted_center_x={tilted_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
