from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehab_overbed_table")

    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    laminate = model.material("laminate", rgba=(0.87, 0.84, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    handle_black = model.material("handle_black", rgba=(0.11, 0.12, 0.13, 1.0))

    base = model.part("base")
    mast = model.part("mast")
    top = model.part("top")
    wing = model.part("wing")
    brake_bar = model.part("brake_bar")
    casters = [model.part(f"caster_{index}") for index in range(4)]

    rail_size = (0.64, 0.07, 0.04)
    rail_z = 0.115

    base.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.035, 0.270, rail_z)),
        material=steel,
        name="rail_0",
    )
    base.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.035, -0.270, rail_z)),
        material=steel,
        name="rail_1",
    )
    base.visual(
        Box((0.08, 0.61, 0.04)),
        origin=Origin(xyz=(-0.325, 0.0, rail_z)),
        material=steel,
        name="rear_cross",
    )
    base.visual(
        Box((0.12, 0.08, 0.04)),
        origin=Origin(xyz=(-0.095, -0.245, 0.155)),
        material=steel,
        name="sleeve_base",
    )
    base.visual(
        Box((0.019, 0.060, 0.47)),
        origin=Origin(xyz=(-0.1155, -0.245, 0.370)),
        material=steel,
        name="sleeve_wall_x0",
    )
    base.visual(
        Box((0.019, 0.060, 0.47)),
        origin=Origin(xyz=(-0.0445, -0.245, 0.370)),
        material=steel,
        name="sleeve_wall_x1",
    )
    base.visual(
        Box((0.069, 0.010, 0.47)),
        origin=Origin(xyz=(-0.080, -0.270, 0.370)),
        material=steel,
        name="sleeve_wall_y0",
    )
    base.visual(
        Box((0.069, 0.010, 0.47)),
        origin=Origin(xyz=(-0.080, -0.220, 0.370)),
        material=steel,
        name="sleeve_wall_y1",
    )
    base.visual(
        Box((0.03, 0.03, 0.04)),
        origin=Origin(xyz=(-0.060, 0.220, 0.050)),
        material=dark_steel,
        name="brake_bracket_0",
    )
    base.visual(
        Box((0.03, 0.03, 0.04)),
        origin=Origin(xyz=(-0.060, -0.220, 0.050)),
        material=dark_steel,
        name="brake_bracket_1",
    )
    base.visual(
        Box((0.03, 0.03, 0.045)),
        origin=Origin(xyz=(-0.060, 0.220, 0.0725)),
        material=dark_steel,
        name="brake_hanger_0",
    )
    base.visual(
        Box((0.03, 0.03, 0.045)),
        origin=Origin(xyz=(-0.060, -0.220, 0.0725)),
        material=dark_steel,
        name="brake_hanger_1",
    )

    caster_positions = (
        (0.280, 0.270),
        (-0.260, 0.270),
        (0.280, -0.270),
        (-0.260, -0.270),
    )
    for index, (caster_x, caster_y) in enumerate(caster_positions):
        base.visual(
            Box((0.03, 0.05, 0.015)),
            origin=Origin(xyz=(caster_x, caster_y, 0.0875)),
            material=dark_steel,
            name=f"fork_{index}_bridge",
        )
        base.visual(
            Box((0.03, 0.008, 0.04)),
            origin=Origin(xyz=(caster_x, caster_y - 0.016, 0.060)),
            material=dark_steel,
            name=f"fork_{index}_cheek_0",
        )
        base.visual(
            Box((0.03, 0.008, 0.04)),
            origin=Origin(xyz=(caster_x, caster_y + 0.016, 0.060)),
            material=dark_steel,
            name=f"fork_{index}_cheek_1",
        )

    mast.visual(
        Box((0.052, 0.032, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=steel,
        name="column",
    )
    mast.visual(
        Box((0.10, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.035, 0.160)),
        material=steel,
        name="head_neck",
    )
    mast.visual(
        Box((0.56, 0.26, 0.035)),
        origin=Origin(xyz=(0.0, 0.070, 0.2175)),
        material=dark_steel,
        name="support_head",
    )
    mast.visual(
        Cylinder(radius=0.008, length=0.56),
        origin=Origin(xyz=(0.0, 0.112, 0.227), rpy=(0.0, 1.5708, 0.0)),
        material=charcoal,
        name="hinge_bar",
    )
    mast.visual(
        Cylinder(radius=0.008, length=0.07),
        origin=Origin(xyz=(0.180, -0.095, 0.185), rpy=(1.5708, 0.0, 0.0)),
        material=handle_black,
        name="handle_stem",
    )
    mast.visual(
        Box((0.04, 0.06, 0.020)),
        origin=Origin(xyz=(0.180, -0.090, 0.185)),
        material=handle_black,
        name="handle_boss",
    )
    mast.visual(
        Box((0.22, 0.11, 0.020)),
        origin=Origin(xyz=(0.090, -0.015, 0.185)),
        material=handle_black,
        name="handle_arm",
    )
    mast.visual(
        Cylinder(radius=0.010, length=0.12),
        origin=Origin(xyz=(0.180, -0.140, 0.185), rpy=(0.0, 1.5708, 0.0)),
        material=handle_black,
        name="release_handle",
    )

    wing.visual(
        Box((0.66, 0.20, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=laminate,
        name="board",
    )

    top.visual(
        Box((0.66, 0.38, 0.018)),
        origin=Origin(xyz=(0.0, 0.19, 0.009)),
        material=laminate,
        name="board",
    )

    brake_bar.visual(
        Box((0.04, 0.376, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=charcoal,
        name="bar",
    )
    brake_bar.visual(
        Box((0.07, 0.14, 0.010)),
        origin=Origin(xyz=(0.020, 0.120, 0.005)),
        material=rubber,
        name="pedal",
    )
    brake_bar.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(0.0, 0.196, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=dark_steel,
        name="pivot_0",
    )
    brake_bar.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(0.0, -0.196, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=dark_steel,
        name="pivot_1",
    )

    for caster in casters:
        caster.visual(
            Cylinder(radius=0.04, length=0.020),
            origin=Origin(rpy=(1.5708, 0.0, 0.0)),
            material=rubber,
            name="wheel",
        )
        caster.visual(
            Box((0.010, 0.006, 0.006)),
            origin=Origin(xyz=(0.036, 0.0, 0.0)),
            material=charcoal,
            name="valve",
        )

    mast_slide = model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(-0.080, -0.245, 0.605)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=0.22,
        ),
    )
    model.articulation(
        "wing_mount",
        ArticulationType.FIXED,
        parent=mast,
        child=wing,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
    )
    top_hinge = model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=top,
        origin=Origin(xyz=(0.0, 0.115, 0.235)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=1.22,
        ),
    )
    brake_pivot = model.articulation(
        "brake_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=brake_bar,
        origin=Origin(xyz=(-0.060, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-0.20,
            upper=0.20,
        ),
    )

    for index, ((caster_x, caster_y), caster) in enumerate(zip(caster_positions, casters)):
        model.articulation(
            f"caster_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(caster_x, caster_y, 0.040)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=14.0),
        )

    mast.meta["qc_samples"] = [0.0, 0.10, mast_slide.motion_limits.upper]
    top.meta["qc_samples"] = [0.0, 0.75, top_hinge.motion_limits.upper]
    brake_bar.meta["qc_samples"] = [-0.20, 0.0, brake_pivot.motion_limits.upper]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    wing = object_model.get_part("wing")
    top = object_model.get_part("top")
    brake_bar = object_model.get_part("brake_bar")
    caster_0 = object_model.get_part("caster_0")

    mast_slide = object_model.get_articulation("mast_slide")
    top_hinge = object_model.get_articulation("top_hinge")
    brake_pivot = object_model.get_articulation("brake_pivot")
    caster_joints = [object_model.get_articulation(f"caster_{index}_spin") for index in range(4)]

    with ctx.pose({mast_slide: 0.0, top_hinge: 0.0}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="column",
            outer_elem="sleeve_wall_y1",
            margin=0.12,
            name="mast column stays under the sleeve head at rest",
        )
        ctx.expect_gap(
            base,
            mast,
            axis="x",
            positive_elem="sleeve_wall_x1",
            negative_elem="column",
            max_gap=0.001,
            max_penetration=0.0,
            name="mast bears against the outer sleeve guide wall",
        )
        ctx.expect_gap(
            mast,
            base,
            axis="x",
            positive_elem="column",
            negative_elem="sleeve_wall_x0",
            max_gap=0.001,
            max_penetration=0.0,
            name="mast bears against the inner sleeve guide wall",
        )
        ctx.expect_gap(
            base,
            mast,
            axis="y",
            positive_elem="sleeve_wall_y1",
            negative_elem="column",
            min_gap=0.003,
            max_gap=0.010,
            name="mast keeps clearance to the positive sleeve y wall",
        )
        ctx.expect_gap(
            mast,
            base,
            axis="y",
            positive_elem="column",
            negative_elem="sleeve_wall_y0",
            min_gap=0.003,
            max_gap=0.010,
            name="mast keeps clearance to the negative sleeve y wall",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="column",
            elem_b="sleeve_wall_x0",
            min_overlap=0.35,
            name="mast remains deeply inserted when lowered",
        )
        ctx.expect_gap(
            top,
            mast,
            axis="z",
            positive_elem="board",
            negative_elem="support_head",
            max_gap=0.002,
            max_penetration=0.0,
            name="main top sits on the support head when closed",
        )
        ctx.expect_overlap(
            top,
            mast,
            axes="x",
            elem_a="board",
            elem_b="support_head",
            min_overlap=0.50,
            name="main top is carried by the shared head across its length",
        )
        ctx.expect_gap(
            wing,
            mast,
            axis="z",
            positive_elem="board",
            negative_elem="support_head",
            max_gap=0.002,
            max_penetration=0.0,
            name="side wing rests on the same support head",
        )
        ctx.expect_overlap(
            wing,
            mast,
            axes="xy",
            elem_a="board",
            elem_b="support_head",
            min_overlap=0.12,
            name="side wing shares the support head footprint",
        )

    slide_limits = mast_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(mast)
        with ctx.pose({mast_slide: slide_limits.upper}):
            extended_pos = ctx.part_world_position(mast)
            ctx.expect_within(
                mast,
                base,
                axes="xy",
                inner_elem="column",
                outer_elem="sleeve_wall_y1",
                margin=0.12,
                name="mast column stays under the sleeve head when raised",
            )
            ctx.expect_gap(
                base,
                mast,
                axis="x",
                positive_elem="sleeve_wall_x1",
                negative_elem="column",
                max_gap=0.001,
                max_penetration=0.0,
                name="raised mast bears against the outer sleeve guide wall",
            )
            ctx.expect_gap(
                mast,
                base,
                axis="x",
                positive_elem="column",
                negative_elem="sleeve_wall_x0",
                max_gap=0.001,
                max_penetration=0.0,
                name="raised mast bears against the inner sleeve guide wall",
            )
            ctx.expect_gap(
                base,
                mast,
                axis="y",
                positive_elem="sleeve_wall_y1",
                negative_elem="column",
                min_gap=0.003,
                max_gap=0.010,
                name="raised mast keeps clearance to the positive sleeve y wall",
            )
            ctx.expect_gap(
                mast,
                base,
                axis="y",
                positive_elem="column",
                negative_elem="sleeve_wall_y0",
                min_gap=0.003,
                max_gap=0.010,
                name="raised mast keeps clearance to the negative sleeve y wall",
            )
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="column",
                elem_b="sleeve_wall_x0",
                min_overlap=0.17,
                name="mast keeps retained insertion at full height",
            )
        ctx.check(
            "mast raises upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    top_limits = top_hinge.motion_limits
    if top_limits is not None and top_limits.upper is not None:
        with ctx.pose({top_hinge: 0.0}):
            closed_aabb = ctx.part_element_world_aabb(top, elem="board")
        with ctx.pose({top_hinge: top_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(top, elem="board")
        closed_top = closed_aabb[1][2] if closed_aabb is not None else None
        open_top = open_aabb[1][2] if open_aabb is not None else None
        ctx.check(
            "main top tilts upward",
            closed_top is not None and open_top is not None and open_top > closed_top + 0.18,
            details=f"closed_max_z={closed_top}, open_max_z={open_top}",
        )

    brake_limits = brake_pivot.motion_limits
    if brake_limits is not None and brake_limits.lower is not None and brake_limits.upper is not None:
        with ctx.pose({brake_pivot: brake_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(brake_bar, elem="pedal")
        with ctx.pose({brake_pivot: brake_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(brake_bar, elem="pedal")
        low_top = low_aabb[1][2] if low_aabb is not None else None
        high_top = high_aabb[1][2] if high_aabb is not None else None
        ctx.check(
            "brake bar rotates across the lower frame",
            low_top is not None and high_top is not None and abs(high_top - low_top) > 0.03,
            details=f"lower_max_z={low_top}, upper_max_z={high_top}",
        )

    with ctx.pose({"caster_0_spin": 0.0}):
        valve_rest = ctx.part_element_world_aabb(caster_0, elem="valve")
    with ctx.pose({"caster_0_spin": 1.10}):
        valve_spun = ctx.part_element_world_aabb(caster_0, elem="valve")
    rest_x = valve_rest[1][0] if valve_rest is not None else None
    spun_x = valve_spun[1][0] if valve_spun is not None else None
    ctx.check(
        "caster wheel spins on its axle",
        rest_x is not None and spun_x is not None and abs(spun_x - rest_x) > 0.01,
        details=f"rest_max_x={rest_x}, spun_max_x={spun_x}",
    )

    continuous_ok = all(
        joint.articulation_type == ArticulationType.CONTINUOUS
        and joint.motion_limits is not None
        and joint.motion_limits.lower is None
        and joint.motion_limits.upper is None
        for joint in caster_joints
    )
    ctx.check(
        "all four casters use continuous spin joints",
        continuous_ok,
        details=str([(joint.name, joint.articulation_type, joint.motion_limits) for joint in caster_joints]),
    )

    return ctx.report()


object_model = build_object_model()
