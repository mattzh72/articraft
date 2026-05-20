from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder(
    radius: float,
    length: float,
    start: tuple[float, float, float],
    direction: tuple[float, float, float],
):
    return cq.Workplane("XY").add(
        cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(*start),
            cq.Vector(*direction),
        )
    )


def _make_housing_mesh():
    """Tall cast-metal housing with real front bores, drawer cavity, and slot."""

    # Main casting plus a broad foot plinth.  Dimensions are desktop-sharpener
    # scale in meters: roughly 15 cm deep, 11 cm wide, and 16 cm tall.
    main = _cq_box((0.120, 0.090, 0.145), (0.000, 0.000, 0.0875))
    plinth = _cq_box((0.145, 0.112, 0.022), (0.000, 0.000, 0.011))
    crown = _cq_box((0.092, 0.074, 0.017), (0.006, 0.000, 0.166))

    housing = main.union(plinth).union(crown)

    # Raised annular pencil-port bezel on the front face.  The following through
    # cut leaves an open tunnel into the mechanism area instead of a capped disk.
    port_bezel = _cq_cylinder(0.027, 0.013, (-0.072, -0.010, 0.112), (1, 0, 0))
    port_hole_in_bezel = _cq_cylinder(0.0155, 0.017, (-0.074, -0.010, 0.112), (1, 0, 0))
    port_bezel = port_bezel.cut(port_hole_in_bezel)
    housing = housing.union(port_bezel)
    port_bore = _cq_cylinder(0.0155, 0.128, (-0.085, -0.010, 0.112), (1, 0, 0))
    housing = housing.cut(port_bore)

    # Rectangular, visibly recessed guide slot beside the pencil port.
    guide_slot = _cq_box((0.026, 0.013, 0.058), (-0.064, 0.034, 0.112))
    housing = housing.cut(guide_slot)

    # Open lower cavity for the shavings drawer.  The drawer part occupies this
    # void without needing a broad collision/overlap allowance.
    drawer_cavity = _cq_box((0.118, 0.079, 0.052), (-0.030, 0.000, 0.047))
    housing = housing.cut(drawer_cavity)

    # Side bearing boss for the crank axle, modeled as a real annular boss.
    side_boss = _cq_cylinder(0.023, 0.012, (0.030, 0.045, 0.096), (0, 1, 0))
    side_bore = _cq_cylinder(0.0108, 0.016, (0.030, 0.043, 0.096), (0, 1, 0))
    side_boss = side_boss.cut(side_bore)
    housing = housing.union(side_boss)

    return housing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_desktop_pencil_sharpener")

    enamel = model.material("dark_green_enamel", color=(0.07, 0.18, 0.13, 1.0))
    brass = model.material("aged_brass", color=(0.72, 0.55, 0.28, 1.0))
    black = model.material("blackened_steel", color=(0.025, 0.025, 0.022, 1.0))
    steel = model.material("brushed_steel", color=(0.58, 0.60, 0.57, 1.0))
    wood = model.material("dark_wood", color=(0.40, 0.20, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_housing_mesh(), "housing"),
        material=enamel,
        name="housing",
    )
    body.visual(
        Box((0.014, 0.004, 0.064)),
        origin=Origin(xyz=(-0.066, 0.022, 0.112)),
        material=enamel,
        name="guide_rail_0",
    )
    body.visual(
        Box((0.014, 0.004, 0.064)),
        origin=Origin(xyz=(-0.066, 0.046, 0.112)),
        material=enamel,
        name="guide_rail_1",
    )
    drawer = model.part("drawer")
    drawer.visual(
        Box((0.010, 0.084, 0.052)),
        origin=Origin(xyz=(-0.0115, 0.000, 0.000)),
        material=brass,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.094, 0.068, 0.006)),
        origin=Origin(xyz=(0.040, 0.000, -0.021)),
        material=black,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.094, 0.006, 0.032)),
        origin=Origin(xyz=(0.040, 0.0335, -0.008)),
        material=black,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.094, 0.006, 0.032)),
        origin=Origin(xyz=(0.040, -0.0335, -0.008)),
        material=black,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.006, 0.068, 0.032)),
        origin=Origin(xyz=(0.087, 0.000, -0.008)),
        material=black,
        name="tray_back",
    )
    drawer.visual(
        Cylinder(radius=0.0065, length=0.012),
        origin=Origin(xyz=(-0.012, 0.000, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="drawer_pull",
    )

    selector = model.part("selector")
    selector.visual(
        Box((0.010, 0.007, 0.018)),
        origin=Origin(xyz=(0.010, 0.000, 0.000)),
        material=steel,
        name="slot_tongue",
    )
    selector.visual(
        Box((0.007, 0.007, 0.010)),
        origin=Origin(xyz=(0.0015, 0.000, 0.000)),
        material=steel,
        name="slider_stem",
    )
    selector.visual(
        Box((0.008, 0.020, 0.016)),
        origin=Origin(xyz=(-0.006, 0.000, 0.000)),
        material=brass,
        name="slider_thumb",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0088, length=0.045),
        origin=Origin(xyz=(0.000, 0.0225, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="side_axle",
    )
    crank.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.000, 0.005, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hub_disk",
    )
    arm_angle = math.atan2(0.045, 0.035)
    arm_len = math.hypot(0.035, 0.045)
    crank.visual(
        Box((arm_len, 0.008, 0.010)),
        origin=Origin(xyz=(0.0175, 0.012, -0.0225), rpy=(0.0, arm_angle, 0.0)),
        material=brass,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.035, 0.016, -0.045), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_pin",
    )
    crank.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(0.035, 0.036, -0.045), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="wood_handle",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.066, 0.000, 0.047)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=0.055),
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.PRISMATIC,
        parent=body,
        child=selector,
        origin=Origin(xyz=(-0.071, 0.034, 0.112)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=-0.014, upper=0.014),
    )

    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.030, 0.057, 0.096)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
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

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    selector = object_model.get_part("selector")
    crank = object_model.get_part("crank")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    selector_slide = object_model.get_articulation("body_to_selector")
    crank_joint = object_model.get_articulation("body_to_crank")

    ctx.expect_overlap(
        drawer,
        body,
        axes="yz",
        elem_a="drawer_face",
        elem_b="housing",
        min_overlap=0.035,
        name="drawer face covers the front opening",
    )
    ctx.expect_within(
        selector,
        body,
        axes="yz",
        inner_elem="slot_tongue",
        outer_elem="housing",
        margin=0.004,
        name="selector tongue stays in the guide slot footprint",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.055}):
        drawer_out = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="housing",
            min_overlap=0.020,
            name="extended drawer remains retained in the body",
        )
    ctx.check(
        "drawer slides out from the front",
        drawer_rest is not None and drawer_out is not None and drawer_out[0] < drawer_rest[0] - 0.045,
        details=f"rest={drawer_rest}, extended={drawer_out}",
    )

    selector_rest = ctx.part_world_position(selector)
    with ctx.pose({selector_slide: 0.014}):
        selector_up = ctx.part_world_position(selector)
    ctx.check(
        "selector slider moves along the short slot",
        selector_rest is not None and selector_up is not None and selector_up[2] > selector_rest[2] + 0.010,
        details=f"rest={selector_rest}, upper={selector_up}",
    )

    crank_rest = ctx.part_world_aabb(crank)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        crank_quarter = ctx.part_world_aabb(crank)
    ctx.check(
        "crank rotates continuously around the side axle",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and crank_rest is not None
        and crank_quarter is not None
        and abs(crank_rest[0][2] - crank_quarter[0][2]) > 0.008,
        details=f"rest_aabb={crank_rest}, quarter_aabb={crank_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
