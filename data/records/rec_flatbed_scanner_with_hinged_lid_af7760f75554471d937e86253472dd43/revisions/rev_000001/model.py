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
    model = ArticulatedObject(name="film_negative_scanner")

    housing = model.material("housing", rgba=(0.18, 0.19, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.09, 0.10, 0.11, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.30, 0.31, 0.33, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.84, 0.92, 0.35))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body_length = 0.29
    body_width = 0.11
    body_height = 0.060
    half_len = body_length / 2.0
    half_width = body_width / 2.0

    body = model.part("body")
    body.visual(
        Box((body_length, body_width, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=housing,
        name="bottom_plate",
    )
    body.visual(
        Box((body_length, 0.008, 0.056)),
        origin=Origin(xyz=(0.0, half_width - 0.004, 0.028)),
        material=housing,
        name="left_side_wall",
    )
    body.visual(
        Box((body_length, 0.008, 0.056)),
        origin=Origin(xyz=(0.0, -half_width + 0.004, 0.028)),
        material=housing,
        name="right_side_wall",
    )
    body.visual(
        Box((0.014, body_width - 0.016, 0.056)),
        origin=Origin(xyz=(-half_len + 0.007, 0.0, 0.028)),
        material=housing,
        name="rear_wall",
    )
    body.visual(
        Box((0.014, body_width - 0.016, 0.016)),
        origin=Origin(xyz=(half_len - 0.007, 0.0, 0.047)),
        material=housing,
        name="front_slot_bridge",
    )
    body.visual(
        Box((0.272, 0.018, 0.008)),
        origin=Origin(xyz=(-0.001, 0.026, 0.056)),
        material=housing,
        name="left_platen_rail",
    )
    body.visual(
        Box((0.272, 0.018, 0.008)),
        origin=Origin(xyz=(-0.001, -0.026, 0.056)),
        material=housing,
        name="right_platen_rail",
    )
    body.visual(
        Box((0.070, 0.052, 0.008)),
        origin=Origin(xyz=(-0.095, 0.0, 0.056)),
        material=housing,
        name="rear_deck_bridge",
    )
    body.visual(
        Box((0.048, 0.052, 0.008)),
        origin=Origin(xyz=(0.112, 0.0, 0.056)),
        material=housing,
        name="front_deck_bridge",
    )
    body.visual(
        Box((0.180, 0.046, 0.002)),
        origin=Origin(xyz=(0.010, 0.0, 0.0565)),
        material=glass,
        name="glass_platen",
    )
    body.visual(
        Box((0.238, 0.008, 0.004)),
        origin=Origin(xyz=(-0.020, 0.028, 0.0315)),
        material=charcoal,
        name="left_tray_guide",
    )
    body.visual(
        Box((0.238, 0.008, 0.004)),
        origin=Origin(xyz=(-0.020, -0.028, 0.0315)),
        material=charcoal,
        name="right_tray_guide",
    )
    body.visual(
        Box((0.028, 0.090, 0.010)),
        origin=Origin(xyz=(-0.125, 0.0, 0.051)),
        material=charcoal,
        name="hinge_backing_strip",
    )
    body.visual(
        Box((0.016, 0.020, 0.003)),
        origin=Origin(xyz=(-0.100, half_width - 0.010, 0.0015)),
        material=rubber,
        name="left_rear_foot",
    )
    body.visual(
        Box((0.016, 0.020, 0.003)),
        origin=Origin(xyz=(0.100, half_width - 0.010, 0.0015)),
        material=rubber,
        name="left_front_foot",
    )
    body.visual(
        Box((0.016, 0.020, 0.003)),
        origin=Origin(xyz=(-0.100, -half_width + 0.010, 0.0015)),
        material=rubber,
        name="right_rear_foot",
    )
    body.visual(
        Box((0.016, 0.020, 0.003)),
        origin=Origin(xyz=(0.100, -half_width + 0.010, 0.0015)),
        material=rubber,
        name="right_front_foot",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.286, 0.114, 0.008)),
        origin=Origin(xyz=(0.143, 0.0, 0.004)),
        material=lid_finish,
        name="lid_panel",
    )
    lid.visual(
        Box((0.258, 0.004, 0.010)),
        origin=Origin(xyz=(0.133, 0.058, -0.004)),
        material=lid_finish,
        name="left_lid_skirt",
    )
    lid.visual(
        Box((0.258, 0.004, 0.010)),
        origin=Origin(xyz=(0.133, -0.058, -0.004)),
        material=lid_finish,
        name="right_lid_skirt",
    )
    lid.visual(
        Box((0.022, 0.084, 0.006)),
        origin=Origin(xyz=(0.275, 0.0, 0.004)),
        material=lid_finish,
        name="front_latch_lip",
    )
    lid.visual(
        Box((0.160, 0.044, 0.002)),
        origin=Origin(xyz=(0.128, 0.0, 0.0085)),
        material=glass,
        name="lid_window",
    )
    lid.visual(
        Box((0.010, 0.010, 0.010)),
        origin=Origin(xyz=(0.006, 0.038, -0.003)),
        material=lid_finish,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.010, 0.010, 0.010)),
        origin=Origin(xyz=(0.006, -0.038, -0.003)),
        material=lid_finish,
        name="right_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.005, 0.038, 0.0005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.005, -0.038, 0.0005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.286, 0.114, 0.018)),
        mass=0.55,
        origin=Origin(xyz=(0.143, 0.0, 0.002)),
    )

    tray = model.part("film_tray")
    tray.visual(
        Box((0.224, 0.004, 0.004)),
        origin=Origin(xyz=(-0.112, 0.028, 0.002)),
        material=tray_finish,
        name="left_tray_rail",
    )
    tray.visual(
        Box((0.224, 0.004, 0.004)),
        origin=Origin(xyz=(-0.112, -0.028, 0.002)),
        material=tray_finish,
        name="right_tray_rail",
    )
    tray.visual(
        Box((0.016, 0.060, 0.004)),
        origin=Origin(xyz=(0.002, 0.0, 0.002)),
        material=tray_finish,
        name="front_tray_crossbar",
    )
    tray.visual(
        Box((0.010, 0.060, 0.004)),
        origin=Origin(xyz=(-0.228, 0.0, 0.002)),
        material=tray_finish,
        name="rear_tray_crossbar",
    )
    tray.visual(
        Box((0.228, 0.006, 0.003)),
        origin=Origin(xyz=(-0.114, 0.013, 0.0015)),
        material=charcoal,
        name="upper_film_guide",
    )
    tray.visual(
        Box((0.228, 0.006, 0.003)),
        origin=Origin(xyz=(-0.114, -0.013, 0.0015)),
        material=charcoal,
        name="lower_film_guide",
    )
    tray.visual(
        Box((0.020, 0.048, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, 0.003)),
        material=tray_finish,
        name="tray_pull_tab",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.240, 0.060, 0.008)),
        mass=0.12,
        origin=Origin(xyz=(-0.110, 0.0, 0.003)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-half_len + 0.001, 0.0, body_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    model.articulation(
        "body_to_film_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(half_len, 0.0, 0.0335)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.15,
            lower=0.0,
            upper=0.115,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("film_tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_slide = object_model.get_articulation("body_to_film_tray")

    with ctx.pose({lid_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="glass_platen",
            min_gap=0.001,
            max_gap=0.006,
            name="closed lid sits just above the platen glass",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.08,
            name="closed lid covers the scanner body footprint",
        )
        ctx.expect_within(
            tray,
            body,
            axes="yz",
            margin=0.003,
            name="film tray sits centered inside the guide slot",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.18,
            name="film tray is substantially inserted at rest",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            positive_elem="glass_platen",
            min_gap=0.012,
            max_gap=0.025,
            name="film tray rides beneath the glass platen",
        )

    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: math.radians(70.0)}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid opens upward on the rear hinge line",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.08,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.115}):
        tray_extended = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            body,
            axes="yz",
            margin=0.003,
            name="extended tray stays aligned in the slot guides",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.06,
            name="extended tray keeps retained insertion inside the scanner",
        )

    ctx.check(
        "film tray extends outward from the front slot",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[0] > tray_rest[0] + 0.09,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
