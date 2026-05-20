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
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_RAIL_X = 0.185
BASE_RAIL_SIZE = (0.050, 0.025, 3.20)
FLY_RAIL_SIZE = (0.042, 0.020, 2.80)
LADDER_RUNG_RADIUS = 0.012
FLY_REST_BOTTOM_Z = 0.58
FLY_SLIDE_TRAVEL = 1.05
HOOK_DEPLOY_LIMIT = math.radians(72.0)


def _add_ladder_rungs(
    part,
    *,
    prefix: str,
    rung_count: int,
    start_z: float,
    spacing: float,
    rung_length: float,
    y: float,
    material,
) -> None:
    for index in range(rung_count):
        z = start_z + spacing * index
        part.visual(
            Cylinder(radius=LADDER_RUNG_RADIUS, length=rung_length),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=f"{prefix}_rung_{index:02d}",
        )


def _add_guide_bracket(part, *, side: str, x_center: float, z_center: float, material) -> None:
    cheek_half_offset = (FLY_RAIL_SIZE[0] * 0.5) + 0.006
    front_lip_y = 0.050
    cheek_y = 0.040
    back_pad_depth = 0.0125
    bracket_height = 0.060

    part.visual(
        Box((0.054, back_pad_depth, bracket_height)),
        origin=Origin(xyz=(x_center, 0.01875, z_center)),
        material=material,
        name=f"{side}_guide_back_pad_{z_center:.2f}".replace(".", "_"),
    )
    part.visual(
        Box((0.070, 0.010, 0.050)),
        origin=Origin(xyz=(x_center, front_lip_y, z_center)),
        material=material,
        name=f"{side}_guide_front_lip_{z_center:.2f}".replace(".", "_"),
    )
    for cheek_name, x_offset in (("outer", -cheek_half_offset), ("inner", cheek_half_offset)):
        part.visual(
            Box((0.012, 0.044, bracket_height)),
            origin=Origin(xyz=(x_center + x_offset, cheek_y, z_center)),
            material=material,
            name=f"{side}_{cheek_name}_guide_cheek_{z_center:.2f}".replace(".", "_"),
        )


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.0, 0.006, -0.020),
            (0.0, 0.010, -0.200),
            (0.0, 0.020, -0.380),
            (0.0, 0.065, -0.500),
            (0.0, 0.145, -0.560),
            (0.0, 0.215, -0.505),
            (0.0, 0.235, -0.395),
        ],
        radius=0.010,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "wall_hook_arm")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hook_over_wall_extension_ladder")

    galvanized = model.material("galvanized", rgba=(0.72, 0.74, 0.76, 1.0))
    rung_steel = model.material("rung_steel", rgba=(0.82, 0.83, 0.84, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    hook_mesh = _build_hook_mesh()

    base = model.part("base_section")
    base.visual(
        Box(BASE_RAIL_SIZE),
        origin=Origin(xyz=(-BASE_RAIL_X, 0.0, BASE_RAIL_SIZE[2] * 0.5)),
        material=galvanized,
        name="left_base_rail",
    )
    base.visual(
        Box(BASE_RAIL_SIZE),
        origin=Origin(xyz=(BASE_RAIL_X, 0.0, BASE_RAIL_SIZE[2] * 0.5)),
        material=galvanized,
        name="right_base_rail",
    )
    _add_ladder_rungs(
        base,
        prefix="base",
        rung_count=11,
        start_z=0.34,
        spacing=0.275,
        rung_length=0.332,
        y=0.0,
        material=rung_steel,
    )
    for side_name, x_center in (("left", -BASE_RAIL_X), ("right", BASE_RAIL_X)):
        for z_center in (1.70, 2.37):
            _add_guide_bracket(base, side=side_name, x_center=x_center, z_center=z_center, material=dark_steel)
    for side_name, x_center in (("left", -BASE_RAIL_X), ("right", BASE_RAIL_X)):
        base.visual(
            Box((0.060, 0.034, 0.055)),
            origin=Origin(xyz=(x_center, 0.0, 0.0275)),
            material=rubber,
            name=f"{side_name}_foot",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.46, 0.08, 3.26)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 1.63)),
    )

    fly = model.part("fly_section")
    fly.visual(
        Box(FLY_RAIL_SIZE),
        origin=Origin(xyz=(-BASE_RAIL_X, 0.0, FLY_RAIL_SIZE[2] * 0.5)),
        material=galvanized,
        name="left_fly_rail",
    )
    fly.visual(
        Box(FLY_RAIL_SIZE),
        origin=Origin(xyz=(BASE_RAIL_X, 0.0, FLY_RAIL_SIZE[2] * 0.5)),
        material=galvanized,
        name="right_fly_rail",
    )
    _add_ladder_rungs(
        fly,
        prefix="fly",
        rung_count=9,
        start_z=0.31,
        spacing=0.285,
        rung_length=0.340,
        y=0.0,
        material=rung_steel,
    )
    for side_name, x_center in (("left", -BASE_RAIL_X), ("right", BASE_RAIL_X)):
        fly.visual(
            Box((0.060, 0.020, 0.110)),
            origin=Origin(xyz=(x_center, 0.010, 2.745)),
            material=dark_steel,
            name=f"{side_name}_hook_mount",
        )
        fly.visual(
            Box((0.052, 0.012, 0.024)),
            origin=Origin(xyz=(x_center, -0.004, 2.788)),
            material=dark_steel,
            name=f"{side_name}_top_cap",
        )
    fly.inertial = Inertial.from_geometry(
        Box((0.42, 0.06, 2.80)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
    )

    left_hook = model.part("left_hook_arm")
    left_hook.visual(
        Cylinder(radius=0.010, length=0.064),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    left_hook.visual(
        hook_mesh,
        material=dark_steel,
        name="hook_body",
    )
    left_hook.visual(
        Box((0.024, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, 0.006, -0.060)),
        material=dark_steel,
        name="mounting_spine",
    )
    left_hook.inertial = Inertial.from_geometry(
        Box((0.12, 0.24, 0.66)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.12, -0.24)),
    )

    right_hook = model.part("right_hook_arm")
    right_hook.visual(
        Cylinder(radius=0.010, length=0.064),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    right_hook.visual(
        hook_mesh,
        material=dark_steel,
        name="hook_body",
    )
    right_hook.visual(
        Box((0.024, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, 0.006, -0.060)),
        material=dark_steel,
        name="mounting_spine",
    )
    right_hook.inertial = Inertial.from_geometry(
        Box((0.12, 0.24, 0.66)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.12, -0.24)),
    )

    model.articulation(
        "base_to_fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.035, FLY_REST_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=700.0,
            velocity=0.35,
            lower=0.0,
            upper=FLY_SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "fly_to_left_hook",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=left_hook,
        origin=Origin(xyz=(-BASE_RAIL_X, 0.030, 2.758)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=0.0,
            upper=HOOK_DEPLOY_LIMIT,
        ),
    )
    model.articulation(
        "fly_to_right_hook",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=right_hook,
        origin=Origin(xyz=(BASE_RAIL_X, 0.030, 2.758)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=0.0,
            upper=HOOK_DEPLOY_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    left_hook = object_model.get_part("left_hook_arm")
    right_hook = object_model.get_part("right_hook_arm")

    fly_slide = object_model.get_articulation("base_to_fly_slide")
    left_hook_hinge = object_model.get_articulation("fly_to_left_hook")
    right_hook_hinge = object_model.get_articulation("fly_to_right_hook")

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

    ctx.expect_contact(
        fly,
        base,
        elem_a="left_fly_rail",
        elem_b="left_guide_back_pad_2_37",
        name="left fly rail is carried by the upper left guide bracket",
    )
    ctx.expect_contact(
        fly,
        base,
        elem_a="right_fly_rail",
        elem_b="right_guide_back_pad_2_37",
        name="right fly rail is carried by the upper right guide bracket",
    )
    ctx.expect_contact(
        left_hook,
        fly,
        elem_a="hinge_barrel",
        elem_b="left_hook_mount",
        name="left hook hinge barrel is mounted to the left fly stile end",
    )
    ctx.expect_contact(
        right_hook,
        fly,
        elem_a="hinge_barrel",
        elem_b="right_hook_mount",
        name="right hook hinge barrel is mounted to the right fly stile end",
    )

    fly_rest = ctx.part_world_position(fly)
    with ctx.pose({fly_slide: FLY_SLIDE_TRAVEL}):
        fly_extended = ctx.part_world_position(fly)
        ctx.expect_contact(
            fly,
            base,
            elem_a="left_fly_rail",
            elem_b="left_guide_back_pad_2_37",
            name="left fly rail stays in contact with the guide at full extension",
        )
        ctx.expect_contact(
            fly,
            base,
            elem_a="right_fly_rail",
            elem_b="right_guide_back_pad_2_37",
            name="right fly rail stays in contact with the guide at full extension",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.45,
            name="fly section retains substantial insertion in the base rails",
        )

    ctx.check(
        "fly section extends upward along the ladder",
        fly_rest is not None
        and fly_extended is not None
        and fly_extended[2] > fly_rest[2] + 1.0
        and abs(fly_extended[0] - fly_rest[0]) < 1e-6,
        details=f"rest={fly_rest}, extended={fly_extended}",
    )

    left_hook_rest_aabb = ctx.part_world_aabb(left_hook)
    right_hook_rest_aabb = ctx.part_world_aabb(right_hook)
    with ctx.pose(
        {
            fly_slide: FLY_SLIDE_TRAVEL,
            left_hook_hinge: HOOK_DEPLOY_LIMIT,
            right_hook_hinge: HOOK_DEPLOY_LIMIT,
        }
    ):
        left_hook_deployed_aabb = ctx.part_world_aabb(left_hook)
        right_hook_deployed_aabb = ctx.part_world_aabb(right_hook)
        ctx.fail_if_parts_overlap_in_current_pose(name="extended ladder with deployed hooks stays collision-free")

    ctx.check(
        "left hook swings forward to engage a ledge",
        left_hook_rest_aabb is not None
        and left_hook_deployed_aabb is not None
        and left_hook_deployed_aabb[1][1] > left_hook_rest_aabb[1][1] + 0.18,
        details=f"rest={left_hook_rest_aabb}, deployed={left_hook_deployed_aabb}",
    )
    ctx.check(
        "right hook swings forward to engage a ledge",
        right_hook_rest_aabb is not None
        and right_hook_deployed_aabb is not None
        and right_hook_deployed_aabb[1][1] > right_hook_rest_aabb[1][1] + 0.18,
        details=f"rest={right_hook_rest_aabb}, deployed={right_hook_deployed_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
