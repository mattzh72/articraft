from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _add_ladder_section(
    part,
    *,
    rail_offset: float,
    rail_size: tuple[float, float, float],
    rail_y: float,
    rail_material,
    rung_count: int,
    rung_z_start: float,
    rung_z_end: float,
    rung_radius: float,
    rung_span: float,
    rung_y: float,
    rung_material,
) -> None:
    for rail_index, rail_x in enumerate((-rail_offset, rail_offset)):
        part.visual(
            Box(rail_size),
            origin=Origin(xyz=(rail_x, rail_y, rail_size[2] * 0.5)),
            material=rail_material,
            name=f"rail_{rail_index}",
        )

    if rung_count == 1:
        rung_positions = [rung_z_start]
    else:
        rung_positions = [
            rung_z_start + (rung_z_end - rung_z_start) * rung_index / (rung_count - 1)
            for rung_index in range(rung_count)
        ]
    for rung_index, rung_z in enumerate(rung_positions):
        part.visual(
            Cylinder(radius=rung_radius, length=rung_span),
            origin=Origin(xyz=(0.0, rung_y, rung_z), rpy=(0.0, pi * 0.5, 0.0)),
            material=rung_material,
            name=f"rung_{rung_index}",
        )


def _build_roof_hook_mesh():
    # Adapted from the spline-tube example pattern, but rebuilt here as one
    # narrower hook arm in the ladder's local YZ plane rather than a suspended
    # crane hook.
    hook_path = [
        (0.0, 0.018, 0.028),
        (0.0, 0.030, 0.095),
        (0.0, 0.076, 0.280),
        (0.0, 0.112, 0.555),
        (0.0, 0.098, 0.815),
        (0.0, 0.020, 0.985),
        (0.0, -0.098, 1.045),
        (0.0, -0.146, 0.920),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            hook_path,
            radius=0.0105,
            samples_per_segment=16,
            radial_segments=16,
            up_hint=(1.0, 0.0, 0.0),
        ),
        "roof_hook_arm",
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_hook_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    rung_aluminum = model.material("rung_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.44, 0.46, 0.49, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    hook_mesh = _build_roof_hook_mesh()

    base = model.part("base")
    _add_ladder_section(
        base,
        rail_offset=0.190,
        rail_size=(0.064, 0.032, 4.80),
        rail_y=0.0,
        rail_material=aluminum,
        rung_count=13,
        rung_z_start=0.48,
        rung_z_end=4.32,
        rung_radius=0.015,
        rung_span=0.356,
        rung_y=0.0,
        rung_material=rung_aluminum,
    )
    base.visual(
        Box((0.340, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, -0.004, 4.62)),
        material=aluminum,
        name="top_cap",
    )
    for shoe_index, shoe_x in enumerate((-0.190, 0.190)):
        base.visual(
            Box((0.040, 0.012, 0.120)),
            origin=Origin(xyz=(shoe_x, -0.001, 0.060)),
            material=hinge_steel,
            name=f"shoe_bracket_{shoe_index}",
        )
        base.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(shoe_x, 0.012, 0.100), rpy=(0.0, pi * 0.5, 0.0)),
            material=hinge_steel,
            name=f"shoe_pivot_{shoe_index}",
        )

    fly = model.part("fly")
    _add_ladder_section(
        fly,
        rail_offset=0.155,
        rail_size=(0.054, 0.024, 4.05),
        rail_y=0.034,
        rail_material=aluminum,
        rung_count=11,
        rung_z_start=0.30,
        rung_z_end=3.74,
        rung_radius=0.013,
        rung_span=0.278,
        rung_y=0.034,
        rung_material=rung_aluminum,
    )
    fly.visual(
        Box((0.270, 0.014, 0.072)),
        origin=Origin(xyz=(0.0, 0.034, 4.014)),
        material=aluminum,
        name="head_cap",
    )
    for guide_index, guide_z in enumerate((0.55, 1.28)):
        for side_index, guide_x in enumerate((-0.153, 0.153)):
            fly.visual(
                Box((0.010, 0.016, 0.140)),
                origin=Origin(xyz=(guide_x, 0.024, guide_z)),
                material=hinge_steel,
                name=f"guide_{guide_index}_{side_index}",
            )
    for hook_index, hook_x in enumerate((-0.155, 0.155)):
        fly.visual(
            Box((0.030, 0.018, 0.082)),
            origin=Origin(xyz=(hook_x, 0.054, 3.969)),
            material=hinge_steel,
            name=f"hook_bracket_{hook_index}",
        )
        fly.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(xyz=(hook_x, 0.064, 4.010), rpy=(0.0, pi * 0.5, 0.0)),
            material=hinge_steel,
            name=f"hook_hinge_{hook_index}",
        )

    for hook_part_name in ("hook_0", "hook_1"):
        hook = model.part(hook_part_name)
        hook.visual(
            Box((0.038, 0.020, 0.100)),
            origin=Origin(xyz=(0.0, 0.017, 0.050)),
            material=dark_steel,
            name="base_plate",
        )
        hook.visual(
            hook_mesh,
            material=dark_steel,
            name="hook_arm",
        )

    for shoe_part_name in ("shoe_0", "shoe_1"):
        shoe = model.part(shoe_part_name)
        shoe.visual(
            Box((0.034, 0.020, 0.042)),
            origin=Origin(xyz=(0.0, 0.012, -0.028)),
            material=dark_steel,
            name="strap",
        )
        shoe.visual(
            Box((0.024, 0.018, 0.182)),
            origin=Origin(xyz=(0.0, 0.020, -0.095)),
            material=dark_steel,
            name="arm",
        )
        shoe.visual(
            Box((0.118, 0.064, 0.022)),
            origin=Origin(xyz=(0.0, 0.040, -0.188)),
            material=rubber,
            name="pad",
        )

    fly_slide = model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.40, lower=0.0, upper=1.55),
    )
    model.articulation(
        "fly_to_hook_0",
        ArticulationType.REVOLUTE,
        parent=fly,
        child="hook_0",
        origin=Origin(xyz=(-0.155, 0.064, 4.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-0.85, upper=0.95),
    )
    model.articulation(
        "fly_to_hook_1",
        ArticulationType.REVOLUTE,
        parent=fly,
        child="hook_1",
        origin=Origin(xyz=(0.155, 0.064, 4.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-0.85, upper=0.95),
    )
    model.articulation(
        "base_to_shoe_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child="shoe_0",
        origin=Origin(xyz=(-0.190, 0.012, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "base_to_shoe_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child="shoe_1",
        origin=Origin(xyz=(0.190, 0.012, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-0.55, upper=0.75),
    )

    fly_slide.meta["qc_samples"] = [0.0, 0.8, 1.55]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    hook_0 = object_model.get_part("hook_0")
    shoe_0 = object_model.get_part("shoe_0")

    fly_slide = object_model.get_articulation("base_to_fly")
    hook_joint_0 = object_model.get_articulation("fly_to_hook_0")
    hook_joint_1 = object_model.get_articulation("fly_to_hook_1")
    shoe_joint_0 = object_model.get_articulation("base_to_shoe_0")
    shoe_joint_1 = object_model.get_articulation("base_to_shoe_1")

    ctx.expect_within(
        fly,
        base,
        axes="x",
        margin=0.01,
        name="fly section stays between the base rails",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=2.8,
        name="collapsed ladder keeps deep fly insertion",
    )

    fly_rest = ctx.part_world_position(fly)
    with ctx.pose({fly_slide: 1.55}):
        ctx.expect_within(
            fly,
            base,
            axes="x",
            margin=0.01,
            name="extended fly section remains centered between the rails",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.1,
            name="extended fly section retains insertion",
        )
        fly_extended = ctx.part_world_position(fly)
    ctx.check(
        "fly section extends upward",
        fly_rest is not None
        and fly_extended is not None
        and fly_extended[2] > fly_rest[2] + 1.4,
        details=f"rest={fly_rest}, extended={fly_extended}",
    )

    hook_rest = _aabb_center(ctx.part_element_world_aabb(hook_0, elem="hook_arm"))
    with ctx.pose({hook_joint_0: 0.95, hook_joint_1: 0.95}):
        hook_open = _aabb_center(ctx.part_element_world_aabb(hook_0, elem="hook_arm"))
    ctx.check(
        "roof hook swings outward for roof engagement",
        hook_rest is not None
        and hook_open is not None
        and hook_open[1] > hook_rest[1] + 0.16,
        details=f"rest={hook_rest}, open={hook_open}",
    )

    shoe_rest = _aabb_center(ctx.part_element_world_aabb(shoe_0, elem="pad"))
    with ctx.pose({shoe_joint_0: 0.75, shoe_joint_1: 0.75}):
        shoe_swung = _aabb_center(ctx.part_element_world_aabb(shoe_0, elem="pad"))
    ctx.check(
        "lower shoe pivots through a usable range",
        shoe_rest is not None
        and shoe_swung is not None
        and shoe_swung[1] > shoe_rest[1] + 0.07
        and abs(shoe_swung[2] - shoe_rest[2]) > 0.05,
        details=f"rest={shoe_rest}, swung={shoe_swung}",
    )

    return ctx.report()


object_model = build_object_model()
