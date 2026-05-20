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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def _build_roof_hook_mesh(name: str):
    hook_path = [
        (0.0, 0.0, 0.0),
        (0.0, 0.002, 0.12),
        (0.0, -0.018, 0.32),
        (0.0, -0.082, 0.56),
        (0.0, -0.165, 0.74),
        (0.0, -0.090, 0.84),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            hook_path,
            radius=0.012,
            samples_per_segment=18,
            radial_segments=18,
            up_hint=(1.0, 0.0, 0.0),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_access_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.36, 0.39, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    warning_red = model.material("warning_red", rgba=(0.76, 0.12, 0.10, 1.0))

    fixed_height = 3.40
    fly_height = 3.00
    fixed_rail_x = 0.170
    fly_rail_x = 0.160

    fixed_section = model.part("fixed_wall_section")
    fixed_section.visual(
        Box((0.050, 0.032, fixed_height)),
        origin=Origin(xyz=(-fixed_rail_x, 0.0, fixed_height * 0.5)),
        material=aluminum,
        name="fixed_left_rail",
    )
    fixed_section.visual(
        Box((0.050, 0.032, fixed_height)),
        origin=Origin(xyz=(fixed_rail_x, 0.0, fixed_height * 0.5)),
        material=aluminum,
        name="fixed_right_rail",
    )
    fixed_section.visual(
        Box((0.360, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, -0.002, 0.080)),
        material=aluminum,
        name="fixed_bottom_tie",
    )
    fixed_section.visual(
        Box((0.340, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, -0.002, fixed_height - 0.070)),
        material=aluminum,
        name="fixed_top_tie",
    )
    for rung_index in range(11):
        fixed_section.visual(
            Box((0.320, 0.024, 0.026)),
            origin=Origin(xyz=(0.0, -0.001, 0.300 + 0.280 * rung_index)),
            material=aluminum,
            name=f"fixed_rung_{rung_index + 1}",
        )
    for bracket_index, z in enumerate((0.82, 2.52), start=1):
        fixed_section.visual(
            Box((0.380, 0.026, 0.036)),
            origin=Origin(xyz=(0.0, -0.029, z)),
            material=steel,
            name=f"wall_standoff_{bracket_index}",
        )
        fixed_section.visual(
            Box((0.090, 0.010, 0.180)),
            origin=Origin(xyz=(-0.110, -0.047, z)),
            material=steel,
            name=f"left_wall_plate_{bracket_index}",
        )
        fixed_section.visual(
            Box((0.090, 0.010, 0.180)),
            origin=Origin(xyz=(0.110, -0.047, z)),
            material=steel,
            name=f"right_wall_plate_{bracket_index}",
        )
    fixed_section.visual(
        Box((0.056, 0.038, 0.030)),
        origin=Origin(xyz=(-fixed_rail_x, 0.0, 0.015)),
        material=dark_rubber,
        name="left_foot",
    )
    fixed_section.visual(
        Box((0.056, 0.038, 0.030)),
        origin=Origin(xyz=(fixed_rail_x, 0.0, 0.015)),
        material=dark_rubber,
        name="right_foot",
    )
    fixed_section.inertial = Inertial.from_geometry(
        Box((0.440, 0.090, fixed_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.010, fixed_height * 0.5)),
    )

    fly_section = model.part("upper_fly_section")
    fly_section.visual(
        Box((0.044, 0.026, fly_height)),
        origin=Origin(xyz=(-fly_rail_x, 0.045, fly_height * 0.5)),
        material=aluminum,
        name="fly_left_rail",
    )
    fly_section.visual(
        Box((0.044, 0.026, fly_height)),
        origin=Origin(xyz=(fly_rail_x, 0.045, fly_height * 0.5)),
        material=aluminum,
        name="fly_right_rail",
    )
    fly_section.visual(
        Box((0.332, 0.022, 0.036)),
        origin=Origin(xyz=(0.0, 0.044, fly_height - 0.020)),
        material=aluminum,
        name="fly_top_crossbar",
    )
    fly_section.visual(
        Box((0.320, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, 0.044, 0.120)),
        material=aluminum,
        name="fly_bottom_tie",
    )
    for rung_index in range(10):
        fly_section.visual(
            Box((0.300, 0.022, 0.022)),
            origin=Origin(xyz=(0.0, 0.045, 0.260 + 0.280 * rung_index)),
            material=aluminum,
            name=f"fly_rung_{rung_index + 1}",
        )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        for shoe_index, z in enumerate((0.45, 1.10), start=1):
            fly_section.visual(
                Box((0.060, 0.016, 0.360)),
                origin=Origin(xyz=(side_sign * fly_rail_x, 0.024, z)),
                material=steel,
                name=f"{side_name}_guide_shoe_{shoe_index}",
            )
    fly_section.visual(
        Box((0.070, 0.028, 0.022)),
        origin=Origin(xyz=(-fly_rail_x, 0.045, fly_height + 0.011)),
        material=steel,
        name="left_hook_mount",
    )
    fly_section.visual(
        Box((0.070, 0.028, 0.022)),
        origin=Origin(xyz=(fly_rail_x, 0.045, fly_height + 0.011)),
        material=steel,
        name="right_hook_mount",
    )
    fly_section.inertial = Inertial.from_geometry(
        Box((0.410, 0.070, fly_height)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.040, fly_height * 0.5)),
    )

    hook_length = 0.84
    left_hook = model.part("left_roof_hook")
    left_hook.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    left_hook.visual(
        _build_roof_hook_mesh("left_roof_hook_body"),
        material=steel,
        name="hook_arm",
    )
    left_hook.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, -0.090, hook_length)),
        material=warning_red,
        name="hook_tip",
    )
    left_hook.inertial = Inertial.from_geometry(
        Box((0.070, 0.240, 0.900)),
        mass=1.3,
        origin=Origin(xyz=(0.0, -0.085, 0.430)),
    )

    right_hook = model.part("right_roof_hook")
    right_hook.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    right_hook.visual(
        _build_roof_hook_mesh("right_roof_hook_body"),
        material=steel,
        name="hook_arm",
    )
    right_hook.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, -0.090, hook_length)),
        material=warning_red,
        name="hook_tip",
    )
    right_hook.inertial = Inertial.from_geometry(
        Box((0.070, 0.240, 0.900)),
        mass=1.3,
        origin=Origin(xyz=(0.0, -0.085, 0.430)),
    )

    model.articulation(
        "fly_extension",
        ArticulationType.PRISMATIC,
        parent=fixed_section,
        child=fly_section,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=1.20,
        ),
    )
    for name, child, x in (
        ("left_hook_hinge", left_hook, -fly_rail_x),
        ("right_hook_hinge", right_hook, fly_rail_x),
    ):
        model.articulation(
            name,
            ArticulationType.REVOLUTE,
            parent=fly_section,
            child=child,
            origin=Origin(xyz=(x, 0.045, fly_height + 0.033)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=1.4,
                lower=0.0,
                upper=math.radians(75.0),
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

    fixed_section = object_model.get_part("fixed_wall_section")
    fly_section = object_model.get_part("upper_fly_section")
    left_hook = object_model.get_part("left_roof_hook")
    right_hook = object_model.get_part("right_roof_hook")
    fly_extension = object_model.get_articulation("fly_extension")
    left_hook_hinge = object_model.get_articulation("left_hook_hinge")
    right_hook_hinge = object_model.get_articulation("right_hook_hinge")

    extension_upper = (
        fly_extension.motion_limits.upper if fly_extension.motion_limits is not None else 1.20
    )
    hook_deploy = (
        left_hook_hinge.motion_limits.upper
        if left_hook_hinge.motion_limits is not None
        else math.radians(75.0)
    )

    with ctx.pose({fly_extension: 0.0, left_hook_hinge: 0.0, right_hook_hinge: 0.0}):
        ctx.expect_contact(
            fly_section,
            fixed_section,
            name="collapsed fly is supported by the fixed guide rails",
        )
        ctx.expect_overlap(
            fly_section,
            fixed_section,
            axes="z",
            min_overlap=2.0,
            name="collapsed fly retains deep insertion into the fixed section",
        )
        ctx.expect_within(
            fly_section,
            fixed_section,
            axes="x",
            margin=0.0,
            name="fly section stays laterally nested inside the fixed section",
        )
        ctx.expect_contact(
            left_hook,
            fly_section,
            name="left roof hook is mounted to the fly top",
        )
        ctx.expect_contact(
            right_hook,
            fly_section,
            name="right roof hook is mounted to the fly top",
        )

    rest_fly_position = None
    extended_fly_position = None
    with ctx.pose({fly_extension: 0.0}):
        rest_fly_position = ctx.part_world_position(fly_section)
    with ctx.pose({fly_extension: extension_upper}):
        ctx.expect_contact(
            fly_section,
            fixed_section,
            name="extended fly still rides on the fixed guide shoes",
        )
        ctx.expect_overlap(
            fly_section,
            fixed_section,
            axes="z",
            min_overlap=1.0,
            name="extended fly keeps retained insertion in the fixed section",
        )
        extended_fly_position = ctx.part_world_position(fly_section)
    ctx.check(
        "fly section extends upward",
        rest_fly_position is not None
        and extended_fly_position is not None
        and extended_fly_position[2] > rest_fly_position[2] + 1.0,
        details=f"rest={rest_fly_position}, extended={extended_fly_position}",
    )

    with ctx.pose({fly_extension: extension_upper, left_hook_hinge: 0.0, right_hook_hinge: 0.0}):
        left_tip_rest = _aabb_center(ctx.part_element_world_aabb(left_hook, elem="hook_tip"))
        right_tip_rest = _aabb_center(ctx.part_element_world_aabb(right_hook, elem="hook_tip"))

    with ctx.pose(
        {
            fly_extension: extension_upper,
            left_hook_hinge: hook_deploy,
            right_hook_hinge: hook_deploy,
        }
    ):
        fly_top_aabb = ctx.part_element_world_aabb(fly_section, elem="fly_top_crossbar")
        left_tip_deployed = _aabb_center(ctx.part_element_world_aabb(left_hook, elem="hook_tip"))
        right_tip_deployed = _aabb_center(ctx.part_element_world_aabb(right_hook, elem="hook_tip"))

    fly_top_z = fly_top_aabb[1][2] if fly_top_aabb is not None else None
    ctx.check(
        "left roof hook folds back over the ridge line",
        left_tip_rest is not None
        and left_tip_deployed is not None
        and fly_top_z is not None
        and left_tip_deployed[1] < left_tip_rest[1] - 0.45
        and left_tip_deployed[2] > fly_top_z + 0.08,
        details=(
            f"rest_tip={left_tip_rest}, deployed_tip={left_tip_deployed}, "
            f"fly_top_z={fly_top_z}"
        ),
    )
    ctx.check(
        "right roof hook folds back over the ridge line",
        right_tip_rest is not None
        and right_tip_deployed is not None
        and fly_top_z is not None
        and right_tip_deployed[1] < right_tip_rest[1] - 0.45
        and right_tip_deployed[2] > fly_top_z + 0.08,
        details=(
            f"rest_tip={right_tip_rest}, deployed_tip={right_tip_deployed}, "
            f"fly_top_z={fly_top_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
