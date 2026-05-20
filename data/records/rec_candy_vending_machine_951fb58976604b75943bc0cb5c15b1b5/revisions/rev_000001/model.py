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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_candy_vending_machine")

    enamel_red = model.material("enamel_red", rgba=(0.74, 0.12, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.36, 0.40, 0.46, 0.35))
    clear_flap = model.material("clear_flap", rgba=(0.75, 0.80, 0.86, 0.32))
    plated_metal = model.material("plated_metal", rgba=(0.76, 0.77, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.180, 0.140, 0.200)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=enamel_red,
        name="lower_cabinet",
    )
    cabinet.visual(
        Box((0.180, 0.100, 0.090)),
        origin=Origin(xyz=(0.0, 0.020, 0.245)),
        material=enamel_red,
        name="upper_cabinet",
    )
    cabinet.visual(
        Box((0.180, 0.008, 0.126)),
        origin=Origin(xyz=(0.0, -0.050, 0.235), rpy=(-0.34, 0.0, 0.0)),
        material=enamel_red,
        name="sloped_panel",
    )
    cabinet.visual(
        Box((0.104, 0.004, 0.068)),
        origin=Origin(xyz=(0.0, -0.052, 0.244), rpy=(-0.34, 0.0, 0.0)),
        material=smoked_clear,
        name="front_window",
    )
    cabinet.visual(
        Box((0.152, 0.024, 0.068)),
        origin=Origin(xyz=(0.0, -0.058, 0.147)),
        material=charcoal,
        name="selection_head",
    )
    cabinet.visual(
        Box((0.086, 0.004, 0.076)),
        origin=Origin(xyz=(0.0, -0.072, 0.147)),
        material=plated_metal,
        name="selector_faceplate",
    )
    cabinet.visual(
        Box((0.036, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.074, 0.171)),
        material=plated_metal,
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.060, 0.036, 0.018)),
        origin=Origin(xyz=(0.0, -0.022, 0.105)),
        material=charcoal,
        name="dispense_throat",
    )
    cabinet.visual(
        Box((0.152, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.061, 0.018)),
        material=charcoal,
        name="pocket_sill",
    )
    cabinet.visual(
        Box((0.144, 0.082, 0.008)),
        origin=Origin(xyz=(0.0, -0.029, 0.022)),
        material=charcoal,
        name="pocket_floor",
    )
    cabinet.visual(
        Box((0.008, 0.082, 0.071)),
        origin=Origin(xyz=(-0.068, -0.029, 0.0545)),
        material=charcoal,
        name="pocket_left_wall",
    )
    cabinet.visual(
        Box((0.008, 0.082, 0.071)),
        origin=Origin(xyz=(0.068, -0.029, 0.0545)),
        material=charcoal,
        name="pocket_right_wall",
    )
    cabinet.visual(
        Box((0.144, 0.008, 0.071)),
        origin=Origin(xyz=(0.0, 0.008, 0.0545)),
        material=charcoal,
        name="pocket_back_wall",
    )
    cabinet.visual(
        Box((0.144, 0.056, 0.008)),
        origin=Origin(xyz=(0.0, -0.022, 0.093)),
        material=charcoal,
        name="pocket_roof",
    )
    cabinet.visual(
        Box((0.022, 0.022, 0.010)),
        origin=Origin(xyz=(-0.065, -0.045, 0.005)),
        material=rubber,
        name="foot_front_left",
    )
    cabinet.visual(
        Box((0.022, 0.022, 0.010)),
        origin=Origin(xyz=(0.065, -0.045, 0.005)),
        material=rubber,
        name="foot_front_right",
    )
    cabinet.visual(
        Box((0.022, 0.022, 0.010)),
        origin=Origin(xyz=(-0.065, 0.045, 0.005)),
        material=rubber,
        name="foot_rear_left",
    )
    cabinet.visual(
        Box((0.022, 0.022, 0.010)),
        origin=Origin(xyz=(0.065, 0.045, 0.005)),
        material=rubber,
        name="foot_rear_right",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.180, 0.140, 0.290)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    collection_flap = model.part("collection_flap")
    collection_flap.visual(
        Box((0.132, 0.005, 0.056)),
        origin=Origin(xyz=(0.0, -0.0025, 0.028)),
        material=clear_flap,
        name="flap_panel",
    )
    collection_flap.visual(
        Box((0.080, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.009, 0.046)),
        material=plated_metal,
        name="flap_pull",
    )
    collection_flap.inertial = Inertial.from_geometry(
        Box((0.132, 0.010, 0.056)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.005, 0.028)),
    )
    model.articulation(
        "cabinet_to_collection_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=collection_flap,
        origin=Origin(xyz=(0.0, -0.070, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plated_metal,
        name="selector_collar",
    )
    selector_knob.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="selector_disc",
    )
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plated_metal,
        name="selector_cap",
    )
    selector_knob.visual(
        Box((0.024, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=plated_metal,
        name="selector_handle",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.048, 0.032, 0.048)),
        mass=0.10,
        origin=Origin(xyz=(0.0, -0.013, 0.0)),
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.0, -0.074, 0.147)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=12.0,
        ),
    )

    refill_hatch = model.part("refill_hatch")
    refill_hatch.visual(
        Box((0.122, 0.005, 0.152)),
        origin=Origin(xyz=(-0.061, 0.0025, 0.076)),
        material=enamel_red,
        name="hatch_panel",
    )
    refill_hatch.visual(
        Box((0.024, 0.014, 0.060)),
        origin=Origin(xyz=(-0.103, 0.012, 0.080)),
        material=plated_metal,
        name="hatch_pull",
    )
    refill_hatch.inertial = Inertial.from_geometry(
        Box((0.126, 0.020, 0.152)),
        mass=0.28,
        origin=Origin(xyz=(-0.058, 0.006, 0.076)),
    )
    model.articulation(
        "cabinet_to_refill_hatch",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=refill_hatch,
        origin=Origin(xyz=(0.061, 0.070, 0.110)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.8,
            lower=0.0,
            upper=2.10,
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

    cabinet = object_model.get_part("cabinet")
    collection_flap = object_model.get_part("collection_flap")
    selector_knob = object_model.get_part("selector_knob")
    refill_hatch = object_model.get_part("refill_hatch")

    flap_joint = object_model.get_articulation("cabinet_to_collection_flap")
    knob_joint = object_model.get_articulation("cabinet_to_selector_knob")
    hatch_joint = object_model.get_articulation("cabinet_to_refill_hatch")

    ctx.check(
        "selector knob uses a continuous horizontal shaft",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in knob_joint.axis) == (0.0, 1.0, 0.0)
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=(
            f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, "
            f"limits={knob_joint.motion_limits}"
        ),
    )
    ctx.check(
        "collection flap has a bottom horizontal hinge",
        flap_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in flap_joint.axis) == (1.0, 0.0, 0.0)
        and flap_joint.motion_limits is not None
        and flap_joint.motion_limits.lower == 0.0
        and flap_joint.motion_limits.upper is not None
        and flap_joint.motion_limits.upper >= 1.3,
        details=f"type={flap_joint.articulation_type}, axis={flap_joint.axis}, limits={flap_joint.motion_limits}",
    )
    ctx.check(
        "rear refill hatch has a vertical side hinge",
        hatch_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in hatch_joint.axis) == (0.0, 0.0, -1.0)
        and hatch_joint.motion_limits is not None
        and hatch_joint.motion_limits.lower == 0.0
        and hatch_joint.motion_limits.upper is not None
        and hatch_joint.motion_limits.upper >= 2.0,
        details=f"type={hatch_joint.articulation_type}, axis={hatch_joint.axis}, limits={hatch_joint.motion_limits}",
    )

    with ctx.pose({flap_joint: 0.0, hatch_joint: 0.0}):
        ctx.expect_contact(
            collection_flap,
            cabinet,
            elem_a="flap_panel",
            elem_b="pocket_sill",
            name="collection flap seats on the pocket sill when closed",
        )
        ctx.expect_contact(
            selector_knob,
            cabinet,
            elem_a="selector_collar",
            elem_b="selector_faceplate",
            name="selector knob collar mounts against the head faceplate",
        )
        ctx.expect_gap(
            refill_hatch,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="hatch_panel",
            negative_elem="upper_cabinet",
            name="refill hatch sits flush on the rear upper cabinet",
        )

        flap_closed = ctx.part_world_aabb(collection_flap)
        hatch_closed = ctx.part_world_aabb(refill_hatch)

    with ctx.pose({flap_joint: 1.05}):
        flap_open = ctx.part_world_aabb(collection_flap)

    with ctx.pose({hatch_joint: 1.10}):
        hatch_open = ctx.part_world_aabb(refill_hatch)

    flap_swings_down = (
        flap_closed is not None
        and flap_open is not None
        and flap_open[1][2] < flap_closed[1][2] - 0.020
        and flap_open[0][1] < flap_closed[0][1] - 0.012
    )
    ctx.check(
        "collection flap opens downward and outward",
        flap_swings_down,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    hatch_swings_open = (
        hatch_closed is not None
        and hatch_open is not None
        and hatch_open[1][1] > hatch_closed[1][1] + 0.020
        and hatch_open[0][0] > hatch_closed[0][0] + 0.040
    )
    ctx.check(
        "rear refill hatch swings outward on its side hinge",
        hatch_swings_open,
        details=f"closed={hatch_closed}, open={hatch_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
