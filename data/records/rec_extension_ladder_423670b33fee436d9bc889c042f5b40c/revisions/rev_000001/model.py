from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


ALUMINIUM = Material("brushed_aluminium", rgba=(0.78, 0.80, 0.78, 1.0))
DARK_ALUMINIUM = Material("dark_aluminium_edges", rgba=(0.48, 0.50, 0.50, 1.0))
BLACK_RUBBER = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))


RAIL_SPACING = 0.34
RAIL_Y = RAIL_SPACING / 2.0
RAIL_DEPTH = 0.034
RAIL_WIDTH = 0.028
STAGE_OFFSET_X = 0.055


def _add_stage_ladder_work(
    part,
    *,
    stage_length: float,
    rung_zs: tuple[float, ...],
    rail_prefix: str = "",
    add_lower_shoes: bool = False,
) -> None:
    """Add two narrow stiles and closely spaced rungs to one ladder stage."""

    for idx, y in enumerate((-RAIL_Y, RAIL_Y)):
        part.visual(
            Box((RAIL_DEPTH, RAIL_WIDTH, stage_length)),
            origin=Origin(xyz=(0.0, y, stage_length / 2.0)),
            material=ALUMINIUM,
            name=f"{rail_prefix}rail_{idx}",
        )
        # Slight darker side web makes the rectangular tube read as an aluminium
        # stile/channel rather than an unfeatured solid bar.
        part.visual(
            Box((0.004, RAIL_WIDTH + 0.002, stage_length - 0.035)),
            origin=Origin(xyz=(-RAIL_DEPTH / 2.0 - 0.001, y, stage_length / 2.0)),
            material=DARK_ALUMINIUM,
            name=f"{rail_prefix}rail_shadow_{idx}",
        )
        if add_lower_shoes:
            part.visual(
                Box((0.070, 0.060, 0.050)),
                origin=Origin(xyz=(-0.006, y, -0.010)),
                material=BLACK_RUBBER,
                name=f"ground_shoe_{idx}",
            )

    for idx, z in enumerate(rung_zs):
        part.visual(
            Cylinder(radius=0.013, length=RAIL_SPACING + 0.070),
            origin=Origin(xyz=(-0.015, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=ALUMINIUM,
            name=f"{rail_prefix}rung_{idx}",
        )


def _add_guide_collar(part, *, rail_y: float, target_x: float, z: float, name: str) -> None:
    """Add a U-shaped guide collar tied back to a parent stile.

    The moving fly rail runs in the clear channel centered at ``target_x`` and
    ``rail_y``.  The collar is made from small plates rather than a solid block,
    so the child rail can pass through without a hidden broad intersection.
    """

    # Root tab bonded to the fixed stile, stopping just before the moving rail.
    part.visual(
        Box((0.020, RAIL_WIDTH + 0.002, 0.110)),
        origin=Origin(xyz=(0.026, rail_y, z)),
        material=DARK_ALUMINIUM,
        name=f"{name}_mount",
    )

    for suffix, sign in (("inner", -1.0), ("outer", 1.0)):
        y = rail_y + sign * 0.029
        # Fork arm starts from the root tab before the sliding rail corridor.
        part.visual(
            Box((0.014, 0.022, 0.110)),
            origin=Origin(xyz=(0.033, rail_y + sign * 0.018, z)),
            material=DARK_ALUMINIUM,
            name=f"{name}_{suffix}_root",
        )
        # Long cheek beside the moving rail.
        part.visual(
            Box((0.045, 0.008, 0.110)),
            origin=Origin(xyz=(target_x - 0.002, y, z)),
            material=DARK_ALUMINIUM,
            name=f"{name}_{suffix}_cheek",
        )

    # Front bridge closes the guide visibly while staying in front of the rail.
    part.visual(
        Box((0.008, 0.066, 0.110)),
        origin=Origin(xyz=(target_x + 0.0245, rail_y, z)),
        material=DARK_ALUMINIUM,
        name=f"{name}_front",
    )


def _add_stage_guide_set(part, *, target_x: float, z_values: tuple[float, ...], prefix: str) -> None:
    for side_idx, y in enumerate((-RAIL_Y, RAIL_Y)):
        for z_idx, z in enumerate(z_values):
            _add_guide_collar(
                part,
                rail_y=y,
                target_x=target_x,
                z=z,
                name=f"{prefix}_guide_{side_idx}_{z_idx}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_confined_access_extension_ladder")

    base = model.part("base_stage")
    _add_stage_ladder_work(
        base,
        stage_length=2.45,
        rung_zs=(0.34, 0.74, 1.14, 1.54, 2.14),
        add_lower_shoes=True,
    )
    _add_stage_guide_set(base, target_x=STAGE_OFFSET_X, z_values=(1.92, 2.32), prefix="base")

    fly_0 = model.part("fly_stage_0")
    _add_stage_ladder_work(
        fly_0,
        stage_length=2.20,
        rung_zs=(0.30, 0.70, 1.10, 1.50, 1.90),
    )
    _add_stage_guide_set(fly_0, target_x=STAGE_OFFSET_X, z_values=(1.06, 1.38), prefix="fly")

    fly_1 = model.part("fly_stage_1")
    _add_stage_ladder_work(
        fly_1,
        stage_length=2.00,
        rung_zs=(0.26, 0.66, 1.06, 1.46, 1.78),
    )
    for idx, y in enumerate((-RAIL_Y, RAIL_Y)):
        # Top clevis tab for the rubber stand-off pivot.
        fly_1.visual(
            Box((0.038, 0.034, 0.060)),
            origin=Origin(xyz=(0.025, y, 1.995)),
            material=DARK_ALUMINIUM,
            name=f"tip_hinge_tab_{idx}",
        )

    for idx, y in enumerate((-RAIL_Y, RAIL_Y)):
        standoff = model.part(f"standoff_{idx}")
        standoff.visual(
            Cylinder(radius=0.012, length=0.058),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=DARK_ALUMINIUM,
            name="hinge_barrel",
        )
        standoff.visual(
            Box((0.034, 0.034, 0.025)),
            origin=Origin(xyz=(0.017, 0.0, 0.0)),
            material=DARK_ALUMINIUM,
            name="rubber_neck",
        )
        standoff.visual(
            Box((0.112, 0.070, 0.046)),
            origin=Origin(xyz=(0.078, 0.0, 0.0)),
            material=BLACK_RUBBER,
            name="rubber_pad",
        )

        model.articulation(
            f"fly_1_to_standoff_{idx}",
            ArticulationType.REVOLUTE,
            parent=fly_1,
            child=standoff,
            origin=Origin(xyz=(0.052, y, 2.000)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.45, upper=0.45),
        )

    model.articulation(
        "base_to_fly_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly_0,
        origin=Origin(xyz=(STAGE_OFFSET_X, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.75),
    )
    model.articulation(
        "fly_0_to_fly_1",
        ArticulationType.PRISMATIC,
        parent=fly_0,
        child=fly_1,
        origin=Origin(xyz=(STAGE_OFFSET_X, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.32, lower=0.0, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_stage")
    fly_0 = object_model.get_part("fly_stage_0")
    fly_1 = object_model.get_part("fly_stage_1")
    slide_0 = object_model.get_articulation("base_to_fly_0")
    slide_1 = object_model.get_articulation("fly_0_to_fly_1")

    prismatic = [joint for joint in object_model.articulations if joint.articulation_type == ArticulationType.PRISMATIC]
    revolute = [joint for joint in object_model.articulations if joint.articulation_type == ArticulationType.REVOLUTE]
    ctx.check(
        "two independent fly slides",
        len(prismatic) == 2 and {joint.name for joint in prismatic} == {"base_to_fly_0", "fly_0_to_fly_1"},
        details=f"prismatic joints={[joint.name for joint in prismatic]}",
    )
    ctx.check(
        "two pivoting upper stand-offs",
        len(revolute) == 2 and all(joint.motion_limits is not None for joint in revolute),
        details=f"revolute joints={[joint.name for joint in revolute]}",
    )

    # Each fly rail passes through named guide collars and remains retained at
    # full travel rather than simply translating away from its parent stage.
    with ctx.pose({slide_0: 0.0, slide_1: 0.0}):
        ctx.expect_overlap(
            fly_0,
            base,
            axes="z",
            elem_a="rail_0",
            elem_b="base_guide_0_1_front",
            min_overlap=0.08,
            name="fly 0 rail passes through base guide when collapsed",
        )
        ctx.expect_overlap(
            fly_1,
            fly_0,
            axes="z",
            elem_a="rail_0",
            elem_b="fly_guide_0_1_front",
            min_overlap=0.08,
            name="fly 1 rail passes through fly guide when collapsed",
        )

    fly_0_rest = ctx.part_world_position(fly_0)
    fly_1_rest = ctx.part_world_position(fly_1)
    with ctx.pose({slide_0: 0.75, slide_1: 0.65}):
        ctx.expect_overlap(
            fly_0,
            base,
            axes="z",
            elem_a="rail_0",
            elem_b="base_guide_0_0_front",
            min_overlap=0.08,
            name="fly 0 rail remains in base guide at extension",
        )
        ctx.expect_overlap(
            fly_1,
            fly_0,
            axes="z",
            elem_a="rail_0",
            elem_b="fly_guide_0_0_front",
            min_overlap=0.08,
            name="fly 1 rail remains in fly guide at extension",
        )
        fly_0_extended = ctx.part_world_position(fly_0)
        fly_1_extended = ctx.part_world_position(fly_1)

    ctx.check(
        "first fly extends upward",
        fly_0_rest is not None
        and fly_0_extended is not None
        and fly_0_extended[2] > fly_0_rest[2] + 0.70,
        details=f"rest={fly_0_rest}, extended={fly_0_extended}",
    )
    ctx.check(
        "second fly extends above first fly",
        fly_1_rest is not None
        and fly_1_extended is not None
        and fly_1_extended[2] > fly_1_rest[2] + 1.30,
        details=f"rest={fly_1_rest}, extended={fly_1_extended}",
    )

    return ctx.report()


object_model = build_object_model()
