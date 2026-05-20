from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_WIDTH = 0.40
BASE_DEPTH = 0.22
BASE_THICKNESS = 0.06
PEDESTAL_RADIUS = 0.040
PEDESTAL_HEIGHT = 0.030
POST_RADIUS = 0.020
POST_HEIGHT = 0.520
POST_SPACING = 0.240

CARRIAGE_WIDTH = 0.340
CARRIAGE_HEIGHT = 0.250
CARRIAGE_THICKNESS = 0.024
SLEEVE_OUTER_RADIUS = 0.045
SLEEVE_INNER_RADIUS = POST_RADIUS + 0.0015
SLEEVE_LENGTH = 0.180
HOME_SLEEVE_BOTTOM_CLEARANCE = 0.075
SLIDE_TRAVEL = 0.250


def _post_x_positions() -> tuple[float, float]:
    half = POST_SPACING / 2.0
    return (-half, half)


def _post_base_z() -> float:
    return BASE_THICKNESS + PEDESTAL_HEIGHT


def _carriage_home_z() -> float:
    carriage_lower_extent = max(CARRIAGE_HEIGHT / 2.0, SLEEVE_LENGTH / 2.0)
    return _base_frame_top_z() + HOME_SLEEVE_BOTTOM_CLEARANCE + carriage_lower_extent


def _base_frame_top_z() -> float:
    return max(
        BASE_THICKNESS,
        BASE_THICKNESS + PEDESTAL_HEIGHT,
        BASE_THICKNESS + 0.085,
        BASE_THICKNESS + 0.018,
    )


def _build_base_frame() -> cq.Workplane:
    left_x, right_x = _post_x_positions()

    slab = cq.Workplane("XY").box(
        BASE_WIDTH,
        BASE_DEPTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )

    pedestals = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .pushPoints([(left_x, 0.0), (right_x, 0.0)])
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
    )

    rear_rib = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .box(
            POST_SPACING + 0.100,
            0.038,
            0.085,
            centered=(True, True, False),
        )
        .translate((0.0, -0.055, 0.0))
    )

    front_pad = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .box(
            0.180,
            0.050,
            0.018,
            centered=(True, True, False),
        )
        .translate((0.0, 0.050, 0.0))
    )

    return slab.union(pedestals).union(rear_rib).union(front_pad)


def _build_carriage_body() -> cq.Workplane:
    left_x, right_x = _post_x_positions()

    back_plate = cq.Workplane("XY").box(
        CARRIAGE_WIDTH,
        0.018,
        CARRIAGE_HEIGHT,
    )

    center_block = cq.Workplane("XY").box(
        POST_SPACING + 0.090,
        0.052,
        0.110,
    )

    top_bridge = cq.Workplane("XY").box(
        POST_SPACING + 0.130,
        0.040,
        0.040,
    ).translate((0.0, 0.0, (CARRIAGE_HEIGHT / 2.0) - 0.040))
    bottom_bridge = cq.Workplane("XY").box(
        POST_SPACING + 0.130,
        0.040,
        0.040,
    ).translate((0.0, 0.0, -(CARRIAGE_HEIGHT / 2.0) + 0.040))

    front_mount = cq.Workplane("XY").box(
        0.220,
        0.058,
        0.070,
    ).translate((0.0, 0.012, -0.055))

    sleeve_housings = (
        cq.Workplane("XY")
        .pushPoints([(left_x, 0.0), (right_x, 0.0)])
        .circle(SLEEVE_OUTER_RADIUS)
        .extrude(SLEEVE_LENGTH, both=True)
    )

    body = (
        back_plate.union(center_block)
        .union(top_bridge)
        .union(bottom_bridge)
        .union(front_mount)
        .union(sleeve_housings)
    )

    post_clearance = (
        cq.Workplane("XY")
        .pushPoints([(left_x, 0.0), (right_x, 0.0)])
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(CARRIAGE_HEIGHT + 0.080, both=True)
    )
    body = body.cut(post_clearance)

    mounting_holes = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (-0.080, -0.070),
                (-0.080, 0.070),
                (0.080, -0.070),
                (0.080, 0.070),
            ]
        )
        .circle(0.008)
        .extrude(0.080, both=True)
    )
    body = body.cut(mounting_holes)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_post_lift_carriage")

    model.material("painted_base", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("guide_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("carriage_orange", rgba=(0.86, 0.46, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_frame(), "base_frame"),
        material="painted_base",
        name="base_frame",
    )
    left_x, right_x = _post_x_positions()
    base.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(left_x, 0.0, _post_base_z() + (POST_HEIGHT / 2.0))),
        material="guide_steel",
        name="left_post",
    )
    base.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(right_x, 0.0, _post_base_z() + (POST_HEIGHT / 2.0))),
        material="guide_steel",
        name="right_post",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, _post_base_z() + POST_HEIGHT)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, (_post_base_z() + POST_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(
            _build_carriage_body(),
            "carriage_body",
            tolerance=0.0003,
            angular_tolerance=0.05,
        ),
        material="carriage_orange",
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, 0.080, CARRIAGE_HEIGHT)),
        mass=8.5,
        origin=Origin(),
    )

    model.articulation(
        "lift_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, _carriage_home_z())),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=1800.0,
            velocity=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("lift_slide")

    ctx.allow_isolated_part(
        carriage,
        reason=(
            "The carriage is intentionally guided by twin round posts through "
            "running-clearance sleeve bores, so the modeled mechanism keeps a "
            "small non-contact fit clearance instead of a zero-clearance touch."
        ),
    )

    rest_position = ctx.part_world_position(carriage)

    limits = slide.motion_limits
    ctx.check(
        "single slide is vertical upward prismatic motion",
        slide.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 0.20,
        details=f"axis={slide.axis}, limits={limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem="carriage_body",
            negative_elem="base_frame",
            min_gap=0.020,
            name="carriage clears the heavy base frame at home",
        )
        ctx.expect_within(
            base,
            carriage,
            axes="xy",
            inner_elem="left_post",
            outer_elem="carriage_body",
            margin=0.020,
            name="left guide post stays inside the carriage sleeve envelope at home",
        )
        ctx.expect_within(
            base,
            carriage,
            axes="xy",
            inner_elem="right_post",
            outer_elem="carriage_body",
            margin=0.020,
            name="right guide post stays inside the carriage sleeve envelope at home",
        )
        ctx.expect_overlap(
            base,
            carriage,
            axes="z",
            elem_a="left_post",
            elem_b="carriage_body",
            min_overlap=0.140,
            name="left sleeve retains tall engagement on the guide post at home",
        )
        ctx.expect_overlap(
            base,
            carriage,
            axes="z",
            elem_a="right_post",
            elem_b="carriage_body",
            min_overlap=0.140,
            name="right sleeve retains tall engagement on the guide post at home",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            base,
            carriage,
            axes="xy",
            inner_elem="left_post",
            outer_elem="carriage_body",
            margin=0.020,
            name="left guide post stays inside the carriage sleeve envelope at full lift",
        )
        ctx.expect_within(
            base,
            carriage,
            axes="xy",
            inner_elem="right_post",
            outer_elem="carriage_body",
            margin=0.020,
            name="right guide post stays inside the carriage sleeve envelope at full lift",
        )
        ctx.expect_overlap(
            base,
            carriage,
            axes="z",
            elem_a="left_post",
            elem_b="carriage_body",
            min_overlap=0.080,
            name="left sleeve remains retained on the post at full lift",
        )
        ctx.expect_overlap(
            base,
            carriage,
            axes="z",
            elem_a="right_post",
            elem_b="carriage_body",
            min_overlap=0.080,
            name="right sleeve remains retained on the post at full lift",
        )
        raised_position = ctx.part_world_position(carriage)

    ctx.check(
        "carriage moves upward along the guide posts",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.20,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
