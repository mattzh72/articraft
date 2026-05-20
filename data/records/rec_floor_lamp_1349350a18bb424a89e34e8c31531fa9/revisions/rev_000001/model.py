from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_RADIUS = 0.17
BASE_HEIGHT = 0.045
BASE_COLLAR_RADIUS = 0.03
BASE_COLLAR_HEIGHT = 0.016
COLUMN_RADIUS = 0.012
SWIVEL_RADIUS = 0.028
SWIVEL_HEIGHT = 0.028
JOINT_Z = 0.84
UPPER_COLUMN_LENGTH = 0.74
SOCKET_STEM_RADIUS = 0.018
SOCKET_STEM_HEIGHT = 0.044
SOCKET_CAP_RADIUS = 0.048
SOCKET_CAP_HEIGHT = 0.014
SHADE_BOTTOM_Z = 0.81


def _build_bowl_shade_mesh():
    outer_profile = [
        (0.036, 0.000),
        (0.070, 0.012),
        (0.130, 0.050),
        (0.188, 0.105),
        (0.226, 0.148),
    ]
    inner_profile = [
        (0.000, 0.010),
        (0.052, 0.024),
        (0.114, 0.060),
        (0.172, 0.110),
        (0.212, 0.139),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ),
        "torchiere_bowl_shade",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torchiere_floor_lamp")

    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_nickel = model.material("satin_nickel", rgba=(0.66, 0.68, 0.71, 1.0))
    socket_gray = model.material("socket_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    glass_white = model.material("glass_white", rgba=(0.90, 0.88, 0.82, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
        material=base_black,
        name="weighted_base",
    )
    lower_body.visual(
        Cylinder(radius=BASE_COLLAR_RADIUS, length=BASE_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + (BASE_COLLAR_HEIGHT * 0.5))),
        material=base_black,
        name="base_hub",
    )
    lower_body.visual(
        Cylinder(radius=COLUMN_RADIUS, length=JOINT_Z - BASE_HEIGHT - BASE_COLLAR_HEIGHT - SWIVEL_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (BASE_HEIGHT + BASE_COLLAR_HEIGHT)
                + ((JOINT_Z - BASE_HEIGHT - BASE_COLLAR_HEIGHT - SWIVEL_HEIGHT) * 0.5),
            )
        ),
        material=satin_nickel,
        name="lower_column",
    )
    lower_body.visual(
        Cylinder(radius=SWIVEL_RADIUS, length=SWIVEL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z - (SWIVEL_HEIGHT * 0.5))),
        material=socket_gray,
        name="lower_swivel_collar",
    )
    lower_body.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=JOINT_Z),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z * 0.5)),
    )

    upper_assembly = model.part("upper_assembly")
    upper_assembly.visual(
        Cylinder(radius=SWIVEL_RADIUS, length=SWIVEL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_HEIGHT * 0.5)),
        material=socket_gray,
        name="upper_swivel_collar",
    )
    upper_assembly.visual(
        Cylinder(radius=COLUMN_RADIUS, length=UPPER_COLUMN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_HEIGHT + (UPPER_COLUMN_LENGTH * 0.5))),
        material=satin_nickel,
        name="upper_column",
    )
    upper_assembly.visual(
        Cylinder(radius=SOCKET_STEM_RADIUS, length=SOCKET_STEM_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                SWIVEL_HEIGHT + UPPER_COLUMN_LENGTH + (SOCKET_STEM_HEIGHT * 0.5),
            )
        ),
        material=socket_gray,
        name="socket_stem",
    )
    upper_assembly.visual(
        Cylinder(radius=SOCKET_CAP_RADIUS, length=SOCKET_CAP_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                SWIVEL_HEIGHT + UPPER_COLUMN_LENGTH + SOCKET_STEM_HEIGHT - 0.006,
            )
        ),
        material=socket_gray,
        name="socket_cap",
    )
    upper_assembly.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(
            xyz=(
                0.053,
                0.0,
                SWIVEL_HEIGHT + UPPER_COLUMN_LENGTH + SOCKET_STEM_HEIGHT - 0.016,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=socket_gray,
        name="switch_knob",
    )
    upper_assembly.visual(
        _build_bowl_shade_mesh(),
        origin=Origin(xyz=(0.0, 0.0, SHADE_BOTTOM_Z)),
        material=glass_white,
        name="shade_bowl",
    )
    upper_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=0.98),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    model.articulation(
        "column_swivel",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_assembly,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-1.75,
            upper=1.75,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_assembly = object_model.get_part("upper_assembly")
    swivel = object_model.get_articulation("column_swivel")
    limits = swivel.motion_limits

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "lamp uses the intended two-part swivel layout",
        lower_body is not None and upper_assembly is not None and swivel is not None,
        details="Expected lower_body, upper_assembly, and column_swivel.",
    )
    ctx.check(
        "swivel axis is vertical",
        swivel.axis == (0.0, 0.0, 1.0),
        details=f"axis={swivel.axis}",
    )
    ctx.check(
        "swivel range is broad enough for directional aiming",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -1.5
        and limits.upper >= 1.5,
        details=f"limits={limits}",
    )

    ctx.expect_gap(
        upper_assembly,
        lower_body,
        axis="z",
        positive_elem="upper_swivel_collar",
        negative_elem="lower_swivel_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel collars stay seated face-to-face",
    )
    ctx.expect_overlap(
        upper_assembly,
        lower_body,
        axes="xy",
        elem_a="upper_swivel_collar",
        elem_b="lower_swivel_collar",
        min_overlap=0.05,
        name="swivel collars share a real bearing footprint",
    )
    ctx.expect_overlap(
        upper_assembly,
        lower_body,
        axes="xy",
        elem_a="shade_bowl",
        elem_b="weighted_base",
        min_overlap=0.30,
        name="shade stays centered over the weighted base footprint",
    )

    rest_knob_center = _aabb_center(ctx.part_element_world_aabb(upper_assembly, elem="switch_knob"))
    with ctx.pose({swivel: 1.1}):
        turned_knob_center = _aabb_center(ctx.part_element_world_aabb(upper_assembly, elem="switch_knob"))
        ctx.fail_if_parts_overlap_in_current_pose(name="lamp remains clear at a turned swivel pose")
        ctx.expect_gap(
            upper_assembly,
            lower_body,
            axis="z",
            positive_elem="upper_swivel_collar",
            negative_elem="lower_swivel_collar",
            max_gap=0.001,
            max_penetration=0.0,
            name="swivel collars stay seated when turned",
        )

    ctx.check(
        "swivel rotates the upper socket hardware around the column",
        rest_knob_center is not None
        and turned_knob_center is not None
        and turned_knob_center[1] > rest_knob_center[1] + 0.03
        and turned_knob_center[0] < rest_knob_center[0] - 0.01
        and abs(turned_knob_center[2] - rest_knob_center[2]) < 0.002,
        details=f"rest={rest_knob_center}, turned={turned_knob_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
