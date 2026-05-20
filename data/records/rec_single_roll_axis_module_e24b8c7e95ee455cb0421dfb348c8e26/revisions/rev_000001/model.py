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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_roll_stage")

    dark_iron = model.material("dark_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("black_oxide", rgba=(0.01, 0.012, 0.014, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.05, 0.22, 0.55, 1.0))
    bolt_dark = model.material("socket_heads", rgba=(0.015, 0.015, 0.017, 1.0))
    index_white = model.material("index_white", rgba=(0.88, 0.88, 0.82, 1.0))

    tower = model.part("tower")

    # Heavy machine foot with a broad footprint and visible cap screws.
    tower.visual(
        Box((0.34, 0.24, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_iron,
        name="foot",
    )
    tower.visual(
        Box((0.18, 0.14, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=black,
        name="base_plinth",
    )
    for i, (x, y) in enumerate(
        ((-0.125, -0.085), (-0.125, 0.085), (0.125, -0.085), (0.125, 0.085))
    ):
        tower.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, y, 0.057), rpy=(0.0, 0.0, 0.0)),
            material=bolt_dark,
            name=f"foot_bolt_{i}",
        )

    # Short pedestal and compact roll-head support.  The bearing housing is
    # deliberately larger than the carried rotating flange.
    tower.visual(
        Cylinder(radius=0.040, length=0.255),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=brushed_steel,
        name="pedestal",
    )
    tower.visual(
        Cylinder(radius=0.060, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.326)),
        material=black,
        name="pedestal_collar",
    )
    tower.visual(
        Box((0.085, 0.130, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=black,
        name="head_block",
    )
    tower.visual(
        Cylinder(radius=0.083, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.382), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="bearing_housing",
    )
    tower.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(0.053, 0.0, 0.382), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="front_seal",
    )

    flange = model.part("flange")
    flange.visual(
        Cylinder(radius=0.025, length=0.023),
        origin=Origin(xyz=(0.0115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="shaft_stub",
    )
    flange.visual(
        Cylinder(radius=0.052, length=0.028),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue_anodized,
        name="carried_flange",
    )
    flange.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pilot_boss",
    )
    for i, (y, z) in enumerate(((0.030, 0.0), (-0.030, 0.0), (0.0, 0.030), (0.0, -0.030))):
        flange.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.045, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_dark,
            name=f"flange_bolt_{i}",
        )
    flange.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.045, 0.037, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=index_white,
        name="index_mark",
    )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flange,
        origin=Origin(xyz=(0.056, 0.0, 0.382)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0, lower=-pi, upper=pi),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    tower = object_model.get_part("tower")
    flange = object_model.get_part("flange")
    roll_axis = object_model.get_articulation("roll_axis")

    ctx.check(
        "single roll articulation",
        len(object_model.articulations) == 1 and roll_axis.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations!r}",
    )
    ctx.check(
        "shaft axis is horizontal",
        tuple(round(v, 6) for v in roll_axis.axis) == (1.0, 0.0, 0.0),
        details=f"axis={roll_axis.axis}",
    )
    ctx.expect_contact(
        flange,
        tower,
        elem_a="shaft_stub",
        elem_b="front_seal",
        contact_tol=0.001,
        name="rotating shaft seats on front seal",
    )
    ctx.expect_gap(
        flange,
        tower,
        axis="x",
        positive_elem="carried_flange",
        negative_elem="bearing_housing",
        min_gap=0.006,
        max_gap=0.040,
        name="flange stands proud of larger bearing housing",
    )
    ctx.expect_within(
        flange,
        tower,
        axes="yz",
        inner_elem="carried_flange",
        outer_elem="bearing_housing",
        margin=0.0,
        name="spinning member is smaller than support head",
    )

    rest_mark = ctx.part_element_world_aabb(flange, elem="index_mark")
    with ctx.pose({roll_axis: pi / 2.0}):
        turned_mark = ctx.part_element_world_aabb(flange, elem="index_mark")
        ctx.expect_contact(
            flange,
            tower,
            elem_a="shaft_stub",
            elem_b="front_seal",
            contact_tol=0.001,
            name="shaft remains seated while rolled",
        )

    def yz_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    rest_yz = yz_center(rest_mark)
    turned_yz = yz_center(turned_mark)
    ctx.check(
        "index mark rolls around shaft",
        rest_yz is not None
        and turned_yz is not None
        and abs(rest_yz[0] - turned_yz[0]) > 0.025
        and abs(rest_yz[1] - turned_yz[1]) > 0.015,
        details=f"rest={rest_yz}, turned={turned_yz}",
    )

    return ctx.report()


object_model = build_object_model()
