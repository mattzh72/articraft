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
    TestContext,
    TestReport,
)


def _add_ladder_section(
    part,
    *,
    prefix: str,
    rail_depth: float,
    rail_width: float,
    rail_center_y: float,
    rail_center_x: float,
    section_length: float,
    rung_positions: list[float],
    rung_radius: float,
    rail_material,
    rung_material,
) -> None:
    for side_name, y in (("left", -rail_center_y), ("right", rail_center_y)):
        part.visual(
            Box((rail_depth, rail_width, section_length)),
            origin=Origin(xyz=(rail_center_x, y, section_length * 0.5)),
            material=rail_material,
            name=f"{prefix}_{side_name}_rail",
        )

    rung_span = rail_center_y * 2.0
    for index, z in enumerate(rung_positions, start=1):
        part.visual(
            Cylinder(radius=rung_radius, length=rung_span),
            origin=Origin(xyz=(rail_center_x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rung_material,
            name=f"{prefix}_rung_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boat_boarding_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.17, 0.17, 0.18, 1.0))

    base_length = 1.55
    fly_length = 1.45
    slide_travel = 0.68

    base_outer_width = 0.34
    base_rail_depth = 0.028
    base_rail_width = 0.022
    base_rail_center_y = base_outer_width * 0.5 - base_rail_width * 0.5

    fly_outer_width = 0.282
    fly_rail_depth = 0.018
    fly_rail_width = 0.018
    fly_rail_center_y = fly_outer_width * 0.5 - fly_rail_width * 0.5
    fly_rail_center_x = 0.010
    fly_offset_x = 0.028

    base_rungs = [0.19, 0.47, 0.75, 1.03, 1.31]
    fly_rungs = [0.17, 0.45, 0.73, 1.01, 1.29]
    guide_block_centers = [0.62, 1.12, 1.42]

    base_section = model.part("base_section")
    _add_ladder_section(
        base_section,
        prefix="base",
        rail_depth=base_rail_depth,
        rail_width=base_rail_width,
        rail_center_y=base_rail_center_y,
        rail_center_x=0.0,
        section_length=base_length,
        rung_positions=base_rungs,
        rung_radius=0.014,
        rail_material=aluminum,
        rung_material=aluminum,
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        guide_y = sign * (base_rail_center_y - base_rail_width * 0.5 - 0.0035)
        for index, z in enumerate(guide_block_centers, start=1):
            base_section.visual(
                Box((0.015, 0.007, 0.070)),
                origin=Origin(xyz=(0.0215, guide_y, z)),
                material=dark_polymer,
                name=f"{side_name}_guide_block_{index}",
            )
    for side_name, y in (("left", -base_rail_center_y), ("right", base_rail_center_y)):
        base_section.visual(
            Box((0.032, 0.026, 0.014)),
            origin=Origin(xyz=(0.0, y, 0.007)),
            material=rubber,
            name=f"{side_name}_foot_cap",
        )
    base_section.inertial = Inertial.from_geometry(
        Box((0.05, base_outer_width, base_length)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, base_length * 0.5)),
    )

    fly_section = model.part("fly_section")
    _add_ladder_section(
        fly_section,
        prefix="fly",
        rail_depth=fly_rail_depth,
        rail_width=fly_rail_width,
        rail_center_y=fly_rail_center_y,
        rail_center_x=fly_rail_center_x,
        section_length=fly_length,
        rung_positions=fly_rungs,
        rung_radius=0.013,
        rail_material=aluminum,
        rung_material=aluminum,
    )
    fly_section.visual(
        Box((0.028, fly_outer_width, 0.020)),
        origin=Origin(xyz=(0.014, 0.0, fly_length - 0.010)),
        material=aluminum,
        name="fly_headstock",
    )
    for side_name, y in (("left", -fly_rail_center_y), ("right", fly_rail_center_y)):
        fly_section.visual(
            Box((0.028, 0.014, 0.060)),
            origin=Origin(xyz=(0.014, y, fly_length - 0.030)),
            material=aluminum,
            name=f"{side_name}_hinge_cheek",
        )
    fly_section.inertial = Inertial.from_geometry(
        Box((0.05, fly_outer_width, fly_length)),
        mass=3.4,
        origin=Origin(xyz=(0.014, 0.0, fly_length * 0.5)),
    )

    hook_plate = model.part("hook_plate")
    hook_plate.visual(
        Box((0.028, fly_outer_width, 0.020)),
        origin=Origin(xyz=(0.014, 0.0, 0.010)),
        material=aluminum,
        name="hook_mount_bar",
    )
    for side_name, y in (("left", -fly_rail_center_y), ("right", fly_rail_center_y)):
        hook_plate.visual(
            Box((0.028, 0.006, 0.240)),
            origin=Origin(xyz=(0.014, y, 0.140)),
            material=aluminum,
            name=f"{side_name}_hook_upright",
        )
        hook_plate.visual(
            Box((0.028, 0.006, 0.080)),
            origin=Origin(xyz=(0.102, y, 0.220)),
            material=aluminum,
            name=f"{side_name}_hook_lip",
        )
    hook_plate.visual(
        Box((0.100, fly_outer_width, 0.028)),
        origin=Origin(xyz=(0.060, 0.0, 0.274)),
        material=aluminum,
        name="hook_crown",
    )
    for x in (0.040, 0.080):
        hook_plate.visual(
            Box((0.018, 0.100, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.256)),
            material=rubber,
            name=f"gunwale_pad_{int(x * 1000):03d}",
        )
    hook_plate.inertial = Inertial.from_geometry(
        Box((0.13, fly_outer_width, 0.30)),
        mass=1.1,
        origin=Origin(xyz=(0.050, 0.0, 0.150)),
    )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=fly_section,
        origin=Origin(xyz=(fly_offset_x, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.40,
            lower=0.0,
            upper=slide_travel,
        ),
    )
    model.articulation(
        "fly_to_hook",
        ArticulationType.REVOLUTE,
        parent=fly_section,
        child=hook_plate,
        origin=Origin(xyz=(0.0, 0.0, fly_length)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.8,
            lower=-0.20,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    fly_section = object_model.get_part("fly_section")
    hook_plate = object_model.get_part("hook_plate")
    fly_slide = object_model.get_articulation("base_to_fly")
    hook_hinge = object_model.get_articulation("fly_to_hook")

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
        "fly slide axis is vertical",
        tuple(fly_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={fly_slide.axis}",
    )
    ctx.check(
        "hook hinge axis is transverse",
        tuple(hook_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={hook_hinge.axis}",
    )
    ctx.expect_contact(
        fly_section,
        base_section,
        name="fly section is supported by base guide blocks",
    )
    ctx.expect_contact(
        hook_plate,
        fly_section,
        name="hook plate is seated on the fly headstock at rest",
    )
    ctx.expect_within(
        fly_section,
        base_section,
        axes="y",
        margin=0.04,
        name="fly section stays between the base stiles in the stored pose",
    )

    rest_fly_pos = ctx.part_world_position(fly_section)
    slide_upper = fly_slide.motion_limits.upper if fly_slide.motion_limits is not None else 0.0
    with ctx.pose({fly_slide: slide_upper}):
        ctx.expect_overlap(
            fly_section,
            base_section,
            axes="z",
            min_overlap=0.50,
            name="extended fly section retains substantial insertion in the base",
        )
        ctx.expect_within(
            fly_section,
            base_section,
            axes="y",
            margin=0.04,
            name="extended fly section still tracks between the base stiles",
        )
        extended_fly_pos = ctx.part_world_position(fly_section)

    ctx.check(
        "fly section extends upward",
        rest_fly_pos is not None
        and extended_fly_pos is not None
        and extended_fly_pos[2] > rest_fly_pos[2] + 0.60,
        details=f"rest={rest_fly_pos}, extended={extended_fly_pos}",
    )

    closed_hook_aabb = ctx.part_world_aabb(hook_plate)
    with ctx.pose({hook_hinge: 1.20}):
        deployed_hook_aabb = ctx.part_world_aabb(hook_plate)

    ctx.check(
        "hook plate swings forward for gunwale engagement",
        closed_hook_aabb is not None
        and deployed_hook_aabb is not None
        and deployed_hook_aabb[1][0] > closed_hook_aabb[1][0] + 0.08,
        details=f"closed={closed_hook_aabb}, deployed={deployed_hook_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
