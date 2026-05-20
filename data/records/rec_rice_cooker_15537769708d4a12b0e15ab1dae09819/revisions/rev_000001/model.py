from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_RADIUS = 0.118
BODY_RADIUS_INSET = 0.111
BODY_HEIGHT = 0.168
LOWER_BAND_HEIGHT = 0.038
LID_RADIUS = 0.122
LID_HEIGHT = 0.050
SELECTOR_CENTER_Z = 0.056
LATCH_CENTER_Z = 0.132
PANEL_FRONT_X = BODY_RADIUS + 0.016


def _body_shell_mesh():
    outer_profile = [
        (0.106, 0.000),
        (BODY_RADIUS_INSET, 0.006),
        (BODY_RADIUS_INSET, LOWER_BAND_HEIGHT),
        (BODY_RADIUS - 0.003, LOWER_BAND_HEIGHT + 0.006),
        (BODY_RADIUS, BODY_HEIGHT - 0.010),
        (BODY_RADIUS + 0.001, BODY_HEIGHT),
    ]
    inner_profile = [
        (0.094, 0.008),
        (0.103, 0.013),
        (0.103, LOWER_BAND_HEIGHT - 0.002),
        (BODY_RADIUS - 0.013, LOWER_BAND_HEIGHT + 0.008),
        (BODY_RADIUS - 0.010, BODY_HEIGHT - 0.004),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=56),
        "rice_cooker_body_shell",
    )


def _lid_shell_mesh():
    outer_profile = [
        (LID_RADIUS, 0.000),
        (LID_RADIUS - 0.004, 0.010),
        (0.094, 0.031),
        (0.055, 0.044),
        (0.000, LID_HEIGHT),
    ]
    inner_profile = [
        (LID_RADIUS - 0.014, 0.004),
        (LID_RADIUS - 0.019, 0.013),
        (0.080, 0.026),
        (0.040, 0.035),
        (0.000, 0.039),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=56),
        "rice_cooker_lid_shell",
    )


def _selector_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.033,
            0.018,
            body_style="skirted",
            top_diameter=0.028,
            skirt=KnobSkirt(0.040, 0.004, flare=0.03),
            grip=KnobGrip(style="fluted", count=14, depth=0.0008),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "rice_cooker_selector",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.95, 0.95, 0.93, 1.0))
    band_gray = model.material("band_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.72, 0.72, 0.70, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.82, 0.83, 0.84, 1.0))
    knob_black = model.material("knob_black", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.20, 0.20, 0.21, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=shell_white, name="body_shell")
    body.visual(
        Cylinder(radius=BODY_RADIUS_INSET + 0.003, length=LOWER_BAND_HEIGHT - 0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (LOWER_BAND_HEIGHT - 0.004))),
        material=band_gray,
        name="lower_band",
    )
    body.visual(
        Box((0.022, 0.056, 0.024)),
        origin=Origin(xyz=(BODY_RADIUS + 0.004, 0.0, LATCH_CENTER_Z)),
        material=trim_gray,
        name="latch_zone",
    )
    body.visual(
        Box((0.020, 0.066, 0.010)),
        origin=Origin(xyz=(BODY_RADIUS + 0.001, 0.0, LATCH_CENTER_Z - 0.012)),
        material=trim_gray,
        name="latch_shelf",
    )
    body.visual(
        Box((0.024, 0.094, 0.064)),
        origin=Origin(xyz=(BODY_RADIUS + 0.004, 0.0, 0.058)),
        material=panel_gray,
        name="control_panel",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(
            xyz=(BODY_RADIUS + 0.011, 0.0, SELECTOR_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_gray,
        name="dial_boss",
    )
    body.visual(
        Box((0.012, 0.092, 0.016)),
        origin=Origin(xyz=(-BODY_RADIUS - 0.008, 0.0, BODY_HEIGHT - 0.003)),
        material=trim_gray,
        name="hinge_housing",
    )
    body.visual(
        Box((0.030, 0.092, 0.008)),
        origin=Origin(xyz=(-BODY_RADIUS - 0.003, 0.0, BODY_HEIGHT - 0.010)),
        material=trim_gray,
        name="hinge_bridge",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.078),
        origin=Origin(
            xyz=(-BODY_RADIUS - 0.008, 0.0, BODY_HEIGHT + 0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_plastic,
        name="rear_hinge_cover",
    )

    lid = model.part("lid")
    lid.visual(
        _lid_shell_mesh(),
        origin=Origin(xyz=(LID_RADIUS, 0.0, 0.0)),
        material=shell_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.024, 0.096, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.010)),
        material=trim_gray,
        name="hinge_cap",
    )
    selector = model.part("selector")
    selector.visual(
        _selector_mesh(),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="selector_knob",
    )
    selector.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(
            xyz=(0.005, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_plastic,
        name="selector_shaft",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-BODY_RADIUS, 0.0, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(PANEL_FRONT_X, 0.0, SELECTOR_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    selector = object_model.get_part("selector")
    lid_hinge = object_model.get_articulation("body_to_lid")
    selector_joint = object_model.get_articulation("body_to_selector")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="body_shell",
        min_gap=0.0,
        max_gap=0.008,
        name="closed lid sits on the body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.18,
        name="closed lid covers the cooker opening",
    )
    ctx.expect_gap(
        selector,
        body,
        axis="x",
        positive_elem="selector_knob",
        negative_elem="control_panel",
        min_gap=0.0,
        max_gap=0.002,
        name="selector mounts directly against the front panel",
    )
    ctx.expect_overlap(
        selector,
        body,
        axes="yz",
        elem_a="selector_knob",
        elem_b="control_panel",
        min_overlap=0.028,
        name="selector sits within the control panel footprint",
    )

    body_latch_aabb = ctx.part_element_world_aabb(body, elem="latch_zone")
    selector_aabb = ctx.part_element_world_aabb(selector, elem="selector_knob")
    ctx.check(
        "selector is below the latch zone",
        body_latch_aabb is not None
        and selector_aabb is not None
        and selector_aabb[1][2] < body_latch_aabb[0][2] - 0.010,
        details=f"selector_aabb={selector_aabb!r}, latch_aabb={body_latch_aabb!r}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: math.radians(85.0)}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.020,
        details=f"closed={closed_lid_aabb!r}, opened={opened_lid_aabb!r}",
    )

    with ctx.pose({selector_joint: math.pi / 2.0}):
        ctx.expect_gap(
            selector,
            body,
            axis="x",
            positive_elem="selector_knob",
            negative_elem="control_panel",
            min_gap=0.0,
            max_gap=0.002,
            name="selector remains seated while rotated",
        )

    return ctx.report()


object_model = build_object_model()
