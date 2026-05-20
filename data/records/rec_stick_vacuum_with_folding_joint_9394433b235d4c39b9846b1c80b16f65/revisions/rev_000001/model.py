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
)


CYL_Y = (math.pi / 2.0, 0.0, 0.0)
CYL_X = (0.0, math.pi / 2.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_reliability_stick_vacuum")

    enamel = model.material("warm_enamel", rgba=(0.86, 0.77, 0.58, 1.0))
    dark = model.material("dark_bakelite", rgba=(0.08, 0.075, 0.07, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.025, 0.027, 0.028, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.67, 0.64, 1.0))
    brass = model.material("aged_brass", rgba=(0.77, 0.55, 0.24, 1.0))
    amber = model.material("smoked_amber", rgba=(0.88, 0.55, 0.20, 0.45))
    blue = model.material("service_blue", rgba=(0.17, 0.28, 0.34, 1.0))

    body = model.part("body")

    # Exposed clevis around the folding pin: side cheeks, top bridge, collar,
    # and bolt heads read like a retrofit adapter plate rather than a molded toy.
    body.visual(
        Box((0.068, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, 0.047, 0.006)),
        material=steel,
        name="fold_left_cheek",
    )
    body.visual(
        Box((0.068, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, -0.047, 0.006)),
        material=steel,
        name="fold_right_cheek",
    )
    body.visual(
        Box((0.088, 0.112, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=steel,
        name="fold_bridge",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=steel,
        name="fold_collar",
    )
    for y in (-0.055, 0.055):
        for z in (-0.018, 0.030):
            body.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(0.001, y, z), rpy=CYL_Y),
                material=brass,
                name=f"fold_bolt_{'rear' if y < 0 else 'front'}_{0 if z < 0 else 1}",
            )

    # Slim spine and compact motor/dust body.
    body.visual(
        Cylinder(radius=0.023, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=steel,
        name="upper_spine",
    )
    body.visual(
        Cylinder(radius=0.056, length=0.230),
        origin=Origin(xyz=(0.046, 0.0, 0.275)),
        material=amber,
        name="dust_cup",
    )
    body.visual(
        Cylinder(radius=0.060, length=0.105),
        origin=Origin(xyz=(0.046, 0.0, 0.438)),
        material=enamel,
        name="motor_can",
    )
    body.visual(
        Box((0.056, 0.094, 0.030)),
        origin=Origin(xyz=(0.018, 0.0, 0.147)),
        material=steel,
        name="cup_lower_band",
    )
    body.visual(
        Box((0.062, 0.096, 0.026)),
        origin=Origin(xyz=(0.020, 0.0, 0.394)),
        material=steel,
        name="motor_band",
    )

    # Service hatches with bolted, legacy-style hardware.
    body.visual(
        Box((0.007, 0.058, 0.118)),
        origin=Origin(xyz=(0.102, 0.0, 0.280)),
        material=blue,
        name="cup_hatch",
    )
    for y in (-0.021, 0.021):
        for z in (0.237, 0.323):
            body.visual(
                Cylinder(radius=0.0042, length=0.006),
                origin=Origin(xyz=(0.106, y, z), rpy=CYL_X),
                material=brass,
                name=f"cup_screw_{0 if y < 0 else 1}_{0 if z < 0.28 else 1}",
            )
    body.visual(
        Box((0.007, 0.046, 0.050)),
        origin=Origin(xyz=(0.108, 0.0, 0.448)),
        material=dark,
        name="motor_hatch",
    )
    for y in (-0.016, 0.016):
        body.visual(
            Cylinder(radius=0.0037, length=0.006),
            origin=Origin(xyz=(0.112, y, 0.448), rpy=CYL_X),
            material=brass,
            name=f"motor_screw_{0 if y < 0 else 1}",
        )

    # Handle loop and pinned trigger bracket.
    body.visual(
        Box((0.038, 0.052, 0.235)),
        origin=Origin(xyz=(-0.118, 0.0, 0.455)),
        material=dark,
        name="rear_grip",
    )
    body.visual(
        Box((0.150, 0.046, 0.034)),
        origin=Origin(xyz=(-0.052, 0.0, 0.573)),
        material=dark,
        name="top_handle",
    )
    body.visual(
        Box((0.128, 0.052, 0.028)),
        origin=Origin(xyz=(-0.054, 0.0, 0.350)),
        material=dark,
        name="lower_handle",
    )
    body.visual(
        Box((0.032, 0.010, 0.026)),
        origin=Origin(xyz=(-0.055, 0.028, 0.325)),
        material=steel,
        name="trigger_left_ear",
    )
    body.visual(
        Box((0.032, 0.010, 0.026)),
        origin=Origin(xyz=(-0.055, -0.028, 0.325)),
        material=steel,
        name="trigger_right_ear",
    )

    # Pragmatic ribs and strap-like retro reinforcements where the body meets
    # the fold adapter.
    body.visual(
        Box((0.014, 0.095, 0.105)),
        origin=Origin(xyz=(-0.037, 0.0, 0.110)),
        material=steel,
        name="rear_gusset",
    )
    body.visual(
        Box((0.014, 0.095, 0.105)),
        origin=Origin(xyz=(0.037, 0.0, 0.110)),
        material=steel,
        name="front_gusset",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.0065, length=0.046),
        origin=Origin(rpy=CYL_Y),
        material=steel,
        name="trigger_pin",
    )
    trigger.visual(
        Box((0.018, 0.024, 0.082)),
        origin=Origin(xyz=(-0.006, 0.0, -0.044)),
        material=rubber,
        name="trigger_blade",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.024, length=0.082),
        origin=Origin(rpy=CYL_Y),
        material=steel,
        name="fold_barrel",
    )
    wand.visual(
        Box((0.056, 0.050, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=enamel,
        name="fold_adapter",
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
        material=steel,
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.025, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=enamel,
        name="upper_clamp",
    )
    wand.visual(
        Cylinder(radius=0.025, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.640)),
        material=enamel,
        name="lower_clamp",
    )
    wand.visual(
        Box((0.006, 0.032, 0.118)),
        origin=Origin(xyz=(0.0195, 0.0, -0.340)),
        material=blue,
        name="wand_service_hatch",
    )
    for y in (-0.012, 0.012):
        for z in (-0.385, -0.295):
            wand.visual(
                Cylinder(radius=0.0032, length=0.0045),
                origin=Origin(xyz=(0.022, y, z), rpy=CYL_X),
                material=brass,
                name=f"wand_screw_{0 if y < 0 else 1}_{0 if z < -0.34 else 1}",
            )

    # Nozzle yoke on the wand: two cheek plates and a bridge just above the
    # trunnion leave actual swing clearance around the floor-head pivot.
    wand.visual(
        Box((0.064, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, 0.045, -0.720)),
        material=steel,
        name="nozzle_left_cheek",
    )
    wand.visual(
        Box((0.064, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, -0.045, -0.720)),
        material=steel,
        name="nozzle_right_cheek",
    )
    wand.visual(
        Box((0.072, 0.108, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.666)),
        material=steel,
        name="nozzle_bridge",
    )
    for y in (-0.053, 0.053):
        wand.visual(
            Cylinder(radius=0.0055, length=0.008),
            origin=Origin(xyz=(0.0, y, -0.720), rpy=CYL_Y),
            material=brass,
            name=f"nozzle_bolt_{0 if y < 0 else 1}",
        )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.019, length=0.078),
        origin=Origin(rpy=CYL_Y),
        material=steel,
        name="nozzle_trunnion",
    )
    floor_head.visual(
        Box((0.050, 0.050, 0.038)),
        origin=Origin(xyz=(0.026, 0.0, -0.037)),
        material=enamel,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.150, 0.320, 0.046)),
        origin=Origin(xyz=(0.060, 0.0, -0.078)),
        material=enamel,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.024, 0.340, 0.036)),
        origin=Origin(xyz=(0.143, 0.0, -0.082)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.070, 0.270, 0.010)),
        origin=Origin(xyz=(0.043, 0.0, -0.105)),
        material=dark,
        name="suction_slot",
    )
    floor_head.visual(
        Cylinder(radius=0.017, length=0.258),
        origin=Origin(xyz=(0.075, 0.0, -0.101), rpy=CYL_Y),
        material=brass,
        name="brush_roll",
    )
    for y in (-0.151, 0.151):
        floor_head.visual(
            Cylinder(radius=0.019, length=0.012),
            origin=Origin(xyz=(-0.020, y, -0.101), rpy=CYL_Y),
            material=rubber,
            name=f"rear_wheel_{0 if y < 0 else 1}",
        )
        floor_head.visual(
            Box((0.020, 0.012, 0.016)),
            origin=Origin(xyz=(-0.015, y * 0.97, -0.092)),
            material=steel,
            name=f"wheel_bracket_{0 if y < 0 else 1}",
        )
    floor_head.visual(
        Box((0.006, 0.092, 0.038)),
        origin=Origin(xyz=(0.135, 0.0, -0.054)),
        material=blue,
        name="head_service_hatch",
    )
    for y in (-0.036, 0.036):
        floor_head.visual(
            Cylinder(radius=0.0032, length=0.004),
            origin=Origin(xyz=(0.138, y, -0.054), rpy=CYL_X),
            material=brass,
            name=f"head_screw_{0 if y < 0 else 1}",
        )

    model.articulation(
        "body_to_trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(-0.055, 0.0, 0.325)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=0.28),
    )
    model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.3, lower=0.0, upper=1.75),
    )
    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.720)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    trigger = object_model.get_part("trigger")
    fold = object_model.get_articulation("body_to_wand")
    nozzle = object_model.get_articulation("wand_to_floor_head")
    trigger_joint = object_model.get_articulation("body_to_trigger")

    ctx.check("major_parts_present", all([body, wand, floor_head, trigger]))
    ctx.check("fold_joint_present", fold is not None)
    ctx.check("nozzle_joint_present", nozzle is not None)
    ctx.check("trigger_joint_present", trigger_joint is not None)

    if body is not None and wand is not None:
        ctx.expect_gap(
            body,
            wand,
            axis="y",
            positive_elem="fold_left_cheek",
            negative_elem="fold_barrel",
            max_penetration=0.001,
            max_gap=0.003,
            name="fold left cheek supports pin",
        )
        ctx.expect_gap(
            wand,
            body,
            axis="y",
            positive_elem="fold_barrel",
            negative_elem="fold_right_cheek",
            max_penetration=0.001,
            max_gap=0.003,
            name="fold right cheek supports pin",
        )
        ctx.expect_overlap(
            body,
            wand,
            axes="xz",
            elem_a="fold_left_cheek",
            elem_b="fold_barrel",
            min_overlap=0.020,
            name="fold cheek brackets barrel",
        )

    if wand is not None and floor_head is not None:
        ctx.expect_gap(
            wand,
            floor_head,
            axis="y",
            positive_elem="nozzle_left_cheek",
            negative_elem="nozzle_trunnion",
            max_penetration=0.001,
            max_gap=0.003,
            name="nozzle left cheek supports trunnion",
        )
        ctx.expect_gap(
            floor_head,
            wand,
            axis="y",
            positive_elem="nozzle_trunnion",
            negative_elem="nozzle_right_cheek",
            max_penetration=0.001,
            max_gap=0.003,
            name="nozzle right cheek supports trunnion",
        )
        ctx.expect_overlap(
            wand,
            floor_head,
            axes="xz",
            elem_a="nozzle_left_cheek",
            elem_b="nozzle_trunnion",
            min_overlap=0.018,
            name="nozzle cheek brackets trunnion",
        )

    if fold is not None and floor_head is not None:
        rest_aabb = ctx.part_world_aabb(floor_head)
        rest_center_x = None
        if rest_aabb is not None:
            rest_center_x = (float(rest_aabb[0][0]) + float(rest_aabb[1][0])) / 2.0
        with ctx.pose({fold: 1.20}):
            folded_aabb = ctx.part_world_aabb(floor_head)
            folded_center_x = None
            if folded_aabb is not None:
                folded_center_x = (float(folded_aabb[0][0]) + float(folded_aabb[1][0])) / 2.0
            ctx.check(
                "fold joint swings wand",
                rest_center_x is not None
                and folded_center_x is not None
                and folded_center_x < rest_center_x - 0.20,
                details=f"rest_center_x={rest_center_x}, folded_center_x={folded_center_x}",
            )

    if nozzle is not None and floor_head is not None:
        rest_aabb = ctx.part_world_aabb(floor_head)
        rest_min_z = float(rest_aabb[0][2]) if rest_aabb is not None else None
        with ctx.pose({nozzle: 0.50}):
            tipped_aabb = ctx.part_world_aabb(floor_head)
            tipped_min_z = float(tipped_aabb[0][2]) if tipped_aabb is not None else None
            ctx.check(
                "nozzle pitch tips head",
                rest_min_z is not None
                and tipped_min_z is not None
                and tipped_min_z < rest_min_z - 0.010,
                details=f"rest_min_z={rest_min_z}, tipped_min_z={tipped_min_z}",
            )

    return ctx.report()


object_model = build_object_model()
