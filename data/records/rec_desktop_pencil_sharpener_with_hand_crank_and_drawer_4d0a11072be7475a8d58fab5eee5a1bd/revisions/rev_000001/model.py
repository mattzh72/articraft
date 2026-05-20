from __future__ import annotations

import math

import cadquery as cq
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


HOUSING_LENGTH = 0.160
HOUSING_WIDTH = 0.090
HOUSING_HEIGHT = 0.100
FRONT_X = -HOUSING_LENGTH / 2.0
BACK_X = HOUSING_LENGTH / 2.0


def _housing_shell() -> cq.Workplane:
    """Rounded desktop sharpener housing with visible front openings."""
    body = (
        cq.Workplane("XY")
        .box(HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_HEIGHT)
        .edges()
        .fillet(0.012)
    )

    # Lower crumb tray tunnel, cut in from the front face.
    tray_slot = (
        cq.Workplane("XY")
        .box(0.088, 0.076, 0.030)
        .translate((FRONT_X + 0.040, 0.0, -0.030))
    )
    body = body.cut(tray_slot.val())

    # Pencil entry bore on the front.  The decorative metal/black bezel visuals
    # below make the front read as a sharpener opening, while this true cutout
    # keeps the face from being a solid placeholder.
    entry_bore = (
        cq.Workplane("YZ")
        .center(-0.014, 0.012)
        .circle(0.016)
        .extrude(0.060)
        .translate((FRONT_X - 0.020, 0.0, 0.0))
    )
    body = body.cut(entry_bore.val())

    # Vertical guide pocket for the sharpening-mode selector beside the entry.
    selector_slot = (
        cq.Workplane("XY")
        .box(0.014, 0.012, 0.042)
        .translate((FRONT_X + 0.006, 0.026, 0.007))
    )
    body = body.cut(selector_slot.val())

    return body


def _tray_drawer() -> cq.Workplane:
    """A shallow hollow shaving tray with a proud front pull lip."""
    outer = cq.Workplane("XY").box(0.074, 0.064, 0.018).translate((0.036, 0.0, 0.0))
    hollow = cq.Workplane("XY").box(0.060, 0.050, 0.018).translate((0.040, 0.0, 0.005))
    basin = outer.cut(hollow)
    lip = cq.Workplane("XY").box(0.008, 0.084, 0.038).translate((-0.0035, 0.0, 0.0))
    return basin.union(lip)


def _rounded_selector_tab() -> cq.Workplane:
    return cq.Workplane("XY").box(0.010, 0.017, 0.012).edges().fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_pencil_sharpener")

    shell_mat = model.material("mint_plastic", rgba=(0.55, 0.78, 0.72, 1.0))
    dark_mat = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    tray_mat = model.material("smoked_tray", rgba=(0.16, 0.18, 0.20, 1.0))
    metal_mat = model.material("brushed_metal", rgba=(0.72, 0.70, 0.65, 1.0))
    red_mat = model.material("red_selector", rgba=(0.85, 0.10, 0.06, 1.0))
    rubber_mat = model.material("rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "rounded_housing", tolerance=0.0007),
        material=shell_mat,
        name="body_shell",
    )
    housing.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(FRONT_X - 0.002, -0.014, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="entry_bezel",
    )
    housing.visual(
        Cylinder(radius=0.0125, length=0.0015),
        origin=Origin(xyz=(FRONT_X - 0.0045, -0.014, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="entry_shadow",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(FRONT_X - 0.004, -0.014, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="inner_cutter",
    )
    housing.visual(
        Box((0.004, 0.003, 0.040)),
        origin=Origin(xyz=(FRONT_X - 0.001, 0.015, 0.007)),
        material=dark_mat,
        name="selector_rail_0",
    )
    housing.visual(
        Box((0.004, 0.003, 0.040)),
        origin=Origin(xyz=(FRONT_X - 0.001, 0.037, 0.007)),
        material=dark_mat,
        name="selector_rail_1",
    )
    housing.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(BACK_X + 0.003, 0.034, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="crank_bushing",
    )
    for idx, (x, y) in enumerate(
        ((-0.055, -0.030), (-0.055, 0.030), (0.055, -0.030), (0.055, 0.030))
    ):
        housing.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, y, -HOUSING_HEIGHT / 2.0 - 0.003)),
            material=rubber_mat,
            name=f"foot_{idx}",
        )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_drawer(), "sliding_tray", tolerance=0.0007),
        material=tray_mat,
        name="tray_basin",
    )
    model.articulation(
        "housing_to_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=tray,
        origin=Origin(xyz=(FRONT_X, 0.0, -0.030)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=0.055),
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_cadquery(_rounded_selector_tab(), "selector_tab", tolerance=0.0005),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=red_mat,
        name="selector_tab",
    )
    selector.visual(
        Box((0.006, 0.012, 0.018)),
        origin=Origin(xyz=(0.0015, 0.0, 0.0)),
        material=red_mat,
        name="selector_stem",
    )
    model.articulation(
        "housing_to_selector",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=selector,
        origin=Origin(xyz=(FRONT_X - 0.001, 0.026, 0.007)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=-0.010, upper=0.010),
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="crank_shaft",
    )
    crank.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="crank_hub",
    )
    crank.visual(
        Box((0.005, 0.010, 0.052)),
        origin=Origin(xyz=(0.026, 0.0, -0.026)),
        material=dark_mat,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(0.042, 0.0, -0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="crank_handle",
    )
    model.articulation(
        "housing_to_crank",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(BACK_X + 0.007, 0.034, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    tray = object_model.get_part("tray")
    selector = object_model.get_part("selector")
    crank = object_model.get_part("crank")
    tray_slide = object_model.get_articulation("housing_to_tray")
    selector_slide = object_model.get_articulation("housing_to_selector")
    crank_joint = object_model.get_articulation("housing_to_crank")

    ctx.allow_overlap(
        housing,
        tray,
        elem_a="body_shell",
        elem_b="tray_basin",
        reason=(
            "The sliding shaving tray is intentionally retained inside the "
            "rounded housing pocket; this scoped allowance covers the hidden "
            "drawer-in-shell insertion."
        ),
    )

    ctx.expect_overlap(
        tray,
        housing,
        axes="x",
        elem_a="tray_basin",
        elem_b="body_shell",
        min_overlap=0.040,
        name="tray is retained in housing when closed",
    )
    ctx.expect_contact(
        crank,
        housing,
        elem_a="crank_shaft",
        elem_b="crank_bushing",
        contact_tol=0.001,
        name="crank shaft seats on rear bushing",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.055}):
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            housing,
            axes="x",
            elem_a="tray_basin",
            elem_b="body_shell",
            min_overlap=0.012,
            name="extended tray keeps a hidden insertion length",
        )
    ctx.check(
        "tray slides out from lower front",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] < rest_tray_pos[0] - 0.045,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    rest_selector_pos = ctx.part_world_position(selector)
    with ctx.pose({selector_slide: 0.010}):
        raised_selector_pos = ctx.part_world_position(selector)
    ctx.check(
        "mode selector slides along short vertical guide",
        rest_selector_pos is not None
        and raised_selector_pos is not None
        and raised_selector_pos[2] > rest_selector_pos[2] + 0.008,
        details=f"rest={rest_selector_pos}, raised={raised_selector_pos}",
    )

    with ctx.pose({crank_joint: math.pi / 2.0}):
        ctx.expect_origin_distance(
            crank,
            housing,
            axes="xy",
            min_dist=0.020,
            max_dist=0.095,
            name="crank remains at rear corner while rotating",
        )

    return ctx.report()


object_model = build_object_model()
