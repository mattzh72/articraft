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
    model = ArticulatedObject(name="hand_syringe")

    clear_poly = model.material("clear_poly", rgba=(0.86, 0.92, 0.98, 0.34))
    white_poly = model.material("white_poly", rgba=(0.97, 0.97, 0.96, 1.0))
    rubber = model.material("rubber", rgba=(0.17, 0.17, 0.18, 1.0))

    body = model.part("body")

    wall_radius = 0.0103
    for index in range(8):
        angle = index * math.pi / 4.0
        body.visual(
            Box((0.080, 0.0082, 0.0018)),
            origin=Origin(
                xyz=(0.0, wall_radius * math.cos(angle), wall_radius * math.sin(angle)),
                rpy=(angle - math.pi / 2.0, 0.0, 0.0),
            ),
            material=clear_poly,
            name=f"barrel_wall_{index}",
        )
    body.visual(
        Box((0.007, 0.024, 0.004)),
        origin=Origin(xyz=(-0.038, 0.022, 0.0)),
        material=clear_poly,
        name="finger_rest_left",
    )
    body.visual(
        Box((0.007, 0.024, 0.004)),
        origin=Origin(xyz=(-0.038, -0.022, 0.0)),
        material=clear_poly,
        name="finger_rest_right",
    )
    body.visual(
        Box((0.008, 0.011, 0.006)),
        origin=Origin(xyz=(-0.039, 0.014, -0.003)),
        material=clear_poly,
        name="left_tab_gusset",
    )
    body.visual(
        Box((0.008, 0.011, 0.006)),
        origin=Origin(xyz=(-0.039, -0.014, -0.003)),
        material=clear_poly,
        name="right_tab_gusset",
    )
    body.visual(
        Cylinder(radius=0.0110, length=0.012),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_poly,
        name="nozzle_hub",
    )
    body.visual(
        Cylinder(radius=0.0020, length=0.014),
        origin=Origin(xyz=(0.057, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_poly,
        name="nozzle_tip",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.112, 0.070, 0.036)),
        mass=0.055,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0032, length=0.124),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_poly,
        name="rod",
    )
    plunger.visual(
        Cylinder(radius=0.0094, length=0.013),
        origin=Origin(xyz=(0.073, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="stopper",
    )
    plunger.visual(
        Cylinder(radius=0.0058, length=0.012),
        origin=Origin(xyz=(-0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_poly,
        name="thumb_stem",
    )
    plunger.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(-0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_poly,
        name="thumb_plate",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.150, 0.034, 0.034)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.050,
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
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("body_to_plunger")
    upper = 0.050

    ctx.expect_origin_gap(
        body,
        plunger,
        axis="x",
        min_gap=0.035,
        max_gap=0.045,
        name="plunger frame is offset to the rear side of the barrel",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            plunger,
            body,
            axes="yz",
            inner_elem="stopper",
            margin=0.001,
            name="depressed stopper stays within barrel diameter",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="x",
            elem_a="stopper",
            min_overlap=0.010,
            name="depressed stopper remains inside the barrel",
        )
        depressed_pos = ctx.part_world_position(plunger)

    with ctx.pose({slide: upper}):
        ctx.expect_within(
            plunger,
            body,
            axes="yz",
            inner_elem="stopper",
            margin=0.001,
            name="withdrawn stopper stays within barrel diameter",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="x",
            elem_a="stopper",
            min_overlap=0.010,
            name="withdrawn stopper still retains insertion",
        )
        withdrawn_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger retracts backward along the barrel axis",
        depressed_pos is not None
        and withdrawn_pos is not None
        and withdrawn_pos[0] < depressed_pos[0] - 0.045,
        details=f"depressed={depressed_pos}, withdrawn={withdrawn_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
