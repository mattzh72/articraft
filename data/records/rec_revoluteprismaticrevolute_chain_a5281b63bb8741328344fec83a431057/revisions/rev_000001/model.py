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


STEEL = Material("satin_dark_steel", rgba=(0.11, 0.12, 0.13, 1.0))
CAST = Material("cast_graphite", rgba=(0.23, 0.25, 0.26, 1.0))
COVER = Material("blue_grey_powdercoat", rgba=(0.26, 0.34, 0.39, 1.0))
SLIDER = Material("brushed_aluminum", rgba=(0.68, 0.70, 0.68, 1.0))
RUBBER = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
MARK = Material("muted_yellow_mark", rgba=(0.78, 0.62, 0.16, 1.0))


def cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, pi / 2.0, 0.0))


def cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(-pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="side_hinged_inspection_boom",
        materials=[STEEL, CAST, COVER, SLIDER, RUBBER, MARK],
    )

    # Fixed compact wall bracket with a vertical side-swing pivot.
    bracket = model.part("mount_bracket")
    bracket.visual(
        Box((0.030, 0.220, 0.230)),
        origin=Origin(xyz=(-0.065, 0.0, 0.0)),
        material=CAST,
        name="wall_plate",
    )
    bracket.visual(
        Box((0.120, 0.220, 0.026)),
        origin=Origin(xyz=(0.000, 0.0, 0.061)),
        material=CAST,
        name="upper_cheek",
    )
    bracket.visual(
        Box((0.120, 0.220, 0.026)),
        origin=Origin(xyz=(0.000, 0.0, -0.061)),
        material=CAST,
        name="lower_cheek",
    )
    bracket.visual(
        Box((0.030, 0.150, 0.035)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0)),
        material=CAST,
        name="cheek_backbone",
    )
    bracket.visual(
        Cylinder(radius=0.010, length=0.158),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=STEEL,
        name="root_pin",
    )
    # Practical swing stops tied between the two cheeks, outside the bearing.
    for idx, y in enumerate((-0.106, 0.106)):
        bracket.visual(
            Box((0.020, 0.018, 0.126)),
            origin=Origin(xyz=(0.044, y, 0.0)),
            material=RUBBER,
            name=f"root_stop_{idx}",
        )
    # Four mounting bolt heads on the front of the wall plate.
    bolt_geom, bolt_rot = cyl_x(0.007, 0.006)
    for row, z in enumerate((-0.076, 0.076)):
        for col, y in enumerate((-0.074, 0.074)):
            bracket.visual(
                bolt_geom,
                origin=Origin(xyz=(-0.047, y, z), rpy=bolt_rot.rpy),
                material=STEEL,
                name=f"wall_bolt_{row}_{col}",
            )

    # Hinged outer boom.  The part frame is exactly on the vertical root pivot.
    outer = model.part("outer_boom")
    outer.visual(
        Cylinder(radius=0.030, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=STEEL,
        name="root_barrel",
    )
    outer.visual(
        Box((0.086, 0.110, 0.060)),
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
        material=COVER,
        name="root_neck",
    )
    outer.visual(
        Box((0.026, 0.030, 0.030)),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material=RUBBER,
        name="root_bumper",
    )
    # Covered sleeve around an open rectangular sliding channel.
    outer.visual(
        Box((0.500, 0.014, 0.060)),
        origin=Origin(xyz=(0.345, 0.052, 0.0)),
        material=COVER,
        name="side_rail_0",
    )
    outer.visual(
        Box((0.500, 0.014, 0.060)),
        origin=Origin(xyz=(0.345, -0.052, 0.0)),
        material=COVER,
        name="side_rail_1",
    )
    outer.visual(
        Box((0.500, 0.094, 0.012)),
        origin=Origin(xyz=(0.345, 0.0, 0.034)),
        material=COVER,
        name="top_cover",
    )
    outer.visual(
        Box((0.500, 0.094, 0.012)),
        origin=Origin(xyz=(0.345, 0.0, -0.034)),
        material=COVER,
        name="bottom_cover",
    )
    # Distal wiper/collar pieces leave the center channel clear.
    outer.visual(
        Box((0.030, 0.110, 0.014)),
        origin=Origin(xyz=(0.590, 0.0, 0.043)),
        material=STEEL,
        name="front_wiper_top",
    )
    outer.visual(
        Box((0.030, 0.110, 0.014)),
        origin=Origin(xyz=(0.590, 0.0, -0.043)),
        material=STEEL,
        name="front_wiper_bottom",
    )
    for row, y in enumerate((-0.026, 0.026)):
        for col, x in enumerate((0.190, 0.350, 0.510)):
            outer.visual(
                Cylinder(radius=0.005, length=0.004),
                origin=Origin(xyz=(x, y, 0.042)),
                material=STEEL,
                name=f"cover_screw_{row}_{col}",
            )
    for side, y in enumerate((-0.061, 0.061)):
        screw_geom, screw_rot = cyl_y(0.0045, 0.004)
        for col, x in enumerate((0.250, 0.470)):
            outer.visual(
                screw_geom,
                origin=Origin(xyz=(x, y, 0.0), rpy=screw_rot.rpy),
                material=STEEL,
                name=f"rail_screw_{side}_{col}",
            )

    # Sliding center extension: enough hidden length remains in the sleeve at
    # full travel, so the boom gains reach without adding extra links.
    extension = model.part("center_extension")
    extension.visual(
        Box((0.640, 0.048, 0.030)),
        origin=Origin(xyz=(-0.110, 0.0, 0.0)),
        material=SLIDER,
        name="center_member",
    )
    extension.visual(
        Box((0.580, 0.034, 0.006)),
        origin=Origin(xyz=(-0.110, 0.0, 0.018)),
        material=STEEL,
        name="slide_wear_top",
    )
    extension.visual(
        Box((0.500, 0.021, 0.022)),
        origin=Origin(xyz=(-0.070, 0.0345, 0.0)),
        material=STEEL,
        name="side_wear_0",
    )
    extension.visual(
        Box((0.500, 0.021, 0.022)),
        origin=Origin(xyz=(-0.070, -0.0345, 0.0)),
        material=STEEL,
        name="side_wear_1",
    )
    # Distal cross-pin yoke for the tilting head.
    extension.visual(
        Box((0.030, 0.104, 0.044)),
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        material=SLIDER,
        name="wrist_bridge",
    )
    extension.visual(
        Box((0.065, 0.018, 0.082)),
        origin=Origin(xyz=(0.235, 0.052, 0.0)),
        material=SLIDER,
        name="wrist_cheek_0",
    )
    extension.visual(
        Box((0.065, 0.018, 0.082)),
        origin=Origin(xyz=(0.235, -0.052, 0.0)),
        material=SLIDER,
        name="wrist_cheek_1",
    )
    wrist_pin_geom, wrist_pin_rot = cyl_y(0.006, 0.128)
    extension.visual(
        wrist_pin_geom,
        origin=Origin(xyz=(0.240, 0.0, 0.0), rpy=wrist_pin_rot.rpy),
        material=STEEL,
        name="wrist_pin",
    )
    cap_geom, cap_rot = cyl_y(0.012, 0.006)
    for idx, y in enumerate((-0.067, 0.067)):
        extension.visual(
            cap_geom,
            origin=Origin(xyz=(0.240, y, 0.0), rpy=cap_rot.rpy),
            material=STEEL,
            name=f"wrist_pin_cap_{idx}",
        )
    # Tip hard stops are tied across the cheeks but sit ahead of the pin path.
    extension.visual(
        Box((0.014, 0.088, 0.008)),
        origin=Origin(xyz=(0.272, 0.0, 0.044)),
        material=RUBBER,
        name="tip_stop_top",
    )
    extension.visual(
        Box((0.014, 0.088, 0.008)),
        origin=Origin(xyz=(0.272, 0.0, -0.044)),
        material=RUBBER,
        name="tip_stop_bottom",
    )

    # Small tilting head plate at the distal cross-pin.
    head = model.part("head_plate")
    head_barrel_geom, head_barrel_rot = cyl_y(0.018, 0.056)
    head.visual(
        head_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=head_barrel_rot.rpy),
        material=STEEL,
        name="head_barrel",
    )
    head.visual(
        Box((0.080, 0.052, 0.040)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=STEEL,
        name="wrist_cap",
    )
    head.visual(
        Box((0.014, 0.160, 0.110)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=COVER,
        name="face_plate",
    )
    head.visual(
        Box((0.020, 0.174, 0.010)),
        origin=Origin(xyz=(0.105, 0.0, 0.060)),
        material=STEEL,
        name="plate_top_lip",
    )
    head.visual(
        Box((0.020, 0.174, 0.010)),
        origin=Origin(xyz=(0.105, 0.0, -0.060)),
        material=STEEL,
        name="plate_bottom_lip",
    )
    head.visual(
        Box((0.020, 0.010, 0.120)),
        origin=Origin(xyz=(0.105, 0.086, 0.0)),
        material=STEEL,
        name="plate_side_lip_0",
    )
    head.visual(
        Box((0.020, 0.010, 0.120)),
        origin=Origin(xyz=(0.105, -0.086, 0.0)),
        material=STEEL,
        name="plate_side_lip_1",
    )
    fastener_geom, fastener_rot = cyl_x(0.0055, 0.004)
    for row, z in enumerate((-0.035, 0.035)):
        for col, y in enumerate((-0.053, 0.053)):
            head.visual(
                fastener_geom,
                origin=Origin(xyz=(0.114, y, z), rpy=fastener_rot.rpy),
                material=STEEL,
                name=f"head_fastener_{row}_{col}",
            )
    head.visual(
        Box((0.004, 0.090, 0.014)),
        origin=Origin(xyz=(0.116, 0.0, -0.050)),
        material=MARK,
        name="inspection_label",
    )

    root_joint = model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=-1.05, upper=1.05),
    )
    root_joint.meta["qc_samples"] = [-1.05, 0.0, 1.05]

    slide_joint = model.articulation(
        "center_slide",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=extension,
        origin=Origin(xyz=(0.560, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.300),
    )
    slide_joint.meta["qc_samples"] = [0.0, 0.150, 0.300]

    tip_joint = model.articulation(
        "tip_tilt",
        ArticulationType.REVOLUTE,
        parent=extension,
        child=head,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.70, upper=0.70),
    )
    tip_joint.meta["qc_samples"] = [-0.70, 0.0, 0.70]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("mount_bracket")
    outer = object_model.get_part("outer_boom")
    extension = object_model.get_part("center_extension")
    head = object_model.get_part("head_plate")
    root_joint = object_model.get_articulation("root_pivot")
    slide_joint = object_model.get_articulation("center_slide")
    tip_joint = object_model.get_articulation("tip_tilt")

    ctx.allow_overlap(
        bracket,
        outer,
        elem_a="root_pin",
        elem_b="root_barrel",
        reason="The compact pivot pin is intentionally captured inside the root bearing barrel.",
    )
    ctx.expect_within(
        bracket,
        outer,
        axes="xy",
        inner_elem="root_pin",
        outer_elem="root_barrel",
        margin=0.001,
        name="root pin is centered inside bearing barrel",
    )
    ctx.expect_overlap(
        bracket,
        outer,
        axes="z",
        elem_a="root_pin",
        elem_b="root_barrel",
        min_overlap=0.075,
        name="root bearing retains vertical pin engagement",
    )

    ctx.allow_overlap(
        extension,
        head,
        elem_a="wrist_pin",
        elem_b="head_barrel",
        reason="The distal cross-pin is intentionally captured inside the tilting head barrel.",
    )
    ctx.expect_within(
        extension,
        head,
        axes="xz",
        inner_elem="wrist_pin",
        outer_elem="head_barrel",
        margin=0.001,
        name="wrist pin is centered inside head barrel",
    )
    ctx.expect_overlap(
        extension,
        head,
        axes="y",
        elem_a="wrist_pin",
        elem_b="head_barrel",
        min_overlap=0.052,
        name="wrist cross-pin spans the head bearing",
    )

    # The center stage slides in a clear channel, not through the covers.
    for q, label in ((0.0, "collapsed"), (0.300, "extended")):
        with ctx.pose({slide_joint: q}):
            ctx.expect_overlap(
                extension,
                outer,
                axes="x",
                elem_a="center_member",
                elem_b="side_rail_0",
                min_overlap=0.150,
                name=f"{label} slider retains insertion in sleeve",
            )
            ctx.expect_gap(
                outer,
                extension,
                axis="y",
                positive_elem="side_rail_0",
                negative_elem="center_member",
                min_gap=0.012,
                name=f"{label} slider clears positive side rail",
            )
            ctx.expect_gap(
                extension,
                outer,
                axis="y",
                positive_elem="center_member",
                negative_elem="side_rail_1",
                min_gap=0.012,
                name=f"{label} slider clears negative side rail",
            )
            ctx.expect_gap(
                outer,
                extension,
                axis="z",
                positive_elem="top_cover",
                negative_elem="center_member",
                min_gap=0.010,
                name=f"{label} slider clears top cover",
            )
            ctx.expect_gap(
                extension,
                outer,
                axis="z",
                positive_elem="center_member",
                negative_elem="bottom_cover",
                min_gap=0.010,
                name=f"{label} slider clears bottom cover",
            )

    with ctx.pose({tip_joint: -0.70}):
        ctx.expect_gap(
            head,
            extension,
            axis="x",
            positive_elem="face_plate",
            negative_elem="wrist_cheek_0",
            min_gap=0.006,
            name="down-tilted head plate clears wrist cheek",
        )
    with ctx.pose({tip_joint: 0.70}):
        ctx.expect_gap(
            head,
            extension,
            axis="x",
            positive_elem="face_plate",
            negative_elem="wrist_cheek_1",
            min_gap=0.006,
            name="up-tilted head plate clears wrist cheek",
        )

    rest_tip = ctx.part_world_position(head)
    with ctx.pose({slide_joint: 0.300}):
        extended_tip = ctx.part_world_position(head)
    ctx.check(
        "center slide increases reach along the boom",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.260,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    with ctx.pose({root_joint: 1.05}):
        swung_tip = ctx.part_world_position(head)
    ctx.check(
        "root pivot swings the boom sideways",
        rest_tip is not None and swung_tip is not None and abs(swung_tip[1]) > 0.45,
        details=f"rest={rest_tip}, swung={swung_tip}",
    )

    return ctx.report()


object_model = build_object_model()
