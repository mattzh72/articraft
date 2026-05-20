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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

SCRIPT_PATH = getattr(globals().get("__spec__"), "origin", __file__)
HERE = SCRIPT_PATH.rsplit("/", 1)[0] if "/" in SCRIPT_PATH else "."


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_turntable")

    plinth_mat = model.material("plinth_black", rgba=(0.09, 0.10, 0.11, 1.0))
    deck_mat = model.material("deck_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    platter_mat = model.material("platter_metal", rgba=(0.63, 0.64, 0.66, 1.0))
    vinyl_mat = model.material("vinyl_black", rgba=(0.04, 0.04, 0.05, 1.0))
    label_mat = model.material("record_label", rgba=(0.76, 0.18, 0.18, 1.0))
    arm_mat = model.material("tonearm_silver", rgba=(0.72, 0.73, 0.75, 1.0))
    slot_mat = model.material("slot_black", rgba=(0.03, 0.03, 0.03, 1.0))
    accent_mat = model.material("accent_red", rgba=(0.86, 0.14, 0.16, 1.0))
    cap_mat = model.material("cap_gray", rgba=(0.58, 0.60, 0.63, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.401, 0.35, 0.038)),
        origin=Origin(xyz=(-0.0295, 0.0, 0.019)),
        material=plinth_mat,
        name="plinth_main",
    )
    body.visual(
        Box((0.041, 0.35, 0.038)),
        origin=Origin(xyz=(0.2095, 0.0, 0.019)),
        material=plinth_mat,
        name="plinth_right",
    )
    body.visual(
        Box((0.018, 0.15, 0.028)),
        origin=Origin(xyz=(0.18, -0.01, 0.014)),
        material=plinth_mat,
        name="pitch_slot_bridge",
    )
    body.visual(
        Box((0.391, 0.33, 0.008)),
        origin=Origin(xyz=(-0.0245, 0.0, 0.042)),
        material=deck_mat,
        name="deck_left",
    )
    body.visual(
        Box((0.031, 0.33, 0.008)),
        origin=Origin(xyz=(0.2045, 0.0, 0.042)),
        material=deck_mat,
        name="deck_right",
    )
    body.visual(
        Box((0.018, 0.08, 0.008)),
        origin=Origin(xyz=(0.18, -0.125, 0.042)),
        material=deck_mat,
        name="pitch_slot_front_cap",
    )
    body.visual(
        Box((0.018, 0.10, 0.008)),
        origin=Origin(xyz=(0.18, 0.115, 0.042)),
        material=deck_mat,
        name="pitch_slot_rear_cap",
    )
    body.visual(
        Box((0.010, 0.15, 0.0015)),
        origin=Origin(xyz=(0.18, -0.01, 0.02875)),
        material=slot_mat,
        name="pitch_slot_floor",
    )
    body.visual(
        Cylinder(radius=0.133, length=0.006),
        origin=Origin(xyz=(-0.065, 0.0, 0.049)),
        material=platter_mat,
        name="platter_well",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.147, 0.11, 0.049)),
        material=platter_mat,
        name="tonearm_pedestal",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(-0.19, -0.118, 0.048)),
        material=accent_mat,
        name="start_button",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.035, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=platter_mat,
        name="platter_hub",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=platter_mat,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.136, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, 0.0112)),
        material=vinyl_mat,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.04, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0130)),
        material=label_mat,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.0045, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0136)),
        material=cap_mat,
        name="center_cap",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=platter_mat,
        name="pivot_housing",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.225),
        material=arm_mat,
        name="arm_tube",
        origin=Origin(
            xyz=(0.048, -0.102, 0.021),
            rpy=(0.0, math.pi / 2.0, -1.131),
        ),
    )
    tonearm.visual(
        Box((0.034, 0.012, 0.005)),
        origin=Origin(xyz=(0.102, -0.217, 0.021), rpy=(0.0, 0.0, -1.131)),
        material=deck_mat,
        name="headshell",
    )
    tonearm.visual(
        Box((0.010, 0.008, 0.006)),
        origin=Origin(xyz=(0.107, -0.228, 0.0185), rpy=(0.0, 0.0, -1.131)),
        material=slot_mat,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.008, length=0.038),
        origin=Origin(
            xyz=(-0.008, 0.017, 0.0195),
            rpy=(0.0, math.pi / 2.0, -1.131),
        ),
        material=platter_mat,
        name="counterweight",
    )

    fader = model.part("pitch_fader")
    fader.visual(
        Box((0.006, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=slot_mat,
        name="slider_stem",
    )
    fader.visual(
        Box((0.016, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=cap_mat,
        name="slider_cap",
    )
    fader.visual(
        Box((0.002, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=accent_mat,
        name="slider_ridge",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=platter,
        origin=Origin(xyz=(-0.065, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=20.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tonearm,
        origin=Origin(xyz=(0.147, 0.11, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-1.45,
            upper=0.05,
        ),
    )
    model.articulation(
        "pitch_fader_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=fader,
        origin=Origin(xyz=(0.18, -0.01, 0.0295)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.25,
            lower=-0.055,
            upper=0.055,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    body = object_model.get_part("body")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    fader = object_model.get_part("pitch_fader")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")
    pitch_fader_slide = object_model.get_articulation("pitch_fader_slide")

    platter_well = body.get_visual("platter_well")
    tonearm_pedestal = body.get_visual("tonearm_pedestal")
    slot_floor = body.get_visual("pitch_slot_floor")
    platter_disc = platter.get_visual("platter_disc")
    platter_hub = platter.get_visual("platter_hub")
    record_disc = platter.get_visual("record_disc")
    pivot_housing = tonearm.get_visual("pivot_housing")
    headshell = tonearm.get_visual("headshell")
    cartridge = tonearm.get_visual("cartridge")
    slider_stem = fader.get_visual("slider_stem")
    slider_cap = fader.get_visual("slider_cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    ctx.expect_within(platter, body, axes="xy", inner_elem=platter_disc)
    ctx.expect_contact(platter, body, elem_a=platter_hub, elem_b=platter_well)
    ctx.expect_origin_distance(platter, body, axes="y", max_dist=0.02)

    ctx.expect_contact(tonearm, body, elem_a=pivot_housing, elem_b=tonearm_pedestal)
    ctx.expect_gap(
        tonearm,
        platter,
        axis="x",
        min_gap=0.04,
        positive_elem=headshell,
        negative_elem=record_disc,
    )

    ctx.expect_contact(fader, body, elem_a=slider_stem, elem_b=slot_floor)
    ctx.expect_within(fader, body, axes="xy", inner_elem=slider_stem, outer_elem=slot_floor)
    ctx.expect_overlap(fader, body, axes="y", elem_a=slider_cap, elem_b=slot_floor, min_overlap=0.014)

    with ctx.pose({platter_spin: math.pi / 2.0}):
        ctx.expect_contact(platter, body, elem_a=platter_hub, elem_b=platter_well)
        ctx.expect_within(platter, body, axes="xy", inner_elem=record_disc)

    with ctx.pose({tonearm_swing: tonearm_swing.motion_limits.lower}):
        ctx.expect_contact(tonearm, body, elem_a=pivot_housing, elem_b=tonearm_pedestal)
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a=cartridge,
            elem_b=record_disc,
            min_overlap=0.004,
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            min_gap=0.0005,
            max_gap=0.004,
            positive_elem=cartridge,
            negative_elem=record_disc,
        )

    with ctx.pose({pitch_fader_slide: pitch_fader_slide.motion_limits.upper}):
        ctx.expect_contact(fader, body, elem_a=slider_stem, elem_b=slot_floor)
        ctx.expect_within(fader, body, axes="xy", inner_elem=slider_stem, outer_elem=slot_floor)
        ctx.fail_if_parts_overlap_in_current_pose(name="pitch_fader_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="pitch_fader_upper_no_floating")

    with ctx.pose({pitch_fader_slide: pitch_fader_slide.motion_limits.lower}):
        ctx.expect_contact(fader, body, elem_a=slider_stem, elem_b=slot_floor)
        ctx.expect_within(fader, body, axes="xy", inner_elem=slider_stem, outer_elem=slot_floor)
        ctx.fail_if_parts_overlap_in_current_pose(name="pitch_fader_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="pitch_fader_lower_no_floating")

    with ctx.pose({tonearm_swing: tonearm_swing.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="tonearm_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="tonearm_lower_no_floating")

    with ctx.pose({tonearm_swing: tonearm_swing.motion_limits.upper}):
        ctx.expect_gap(
            tonearm,
            platter,
            axis="x",
            min_gap=0.05,
            positive_elem=headshell,
            negative_elem=record_disc,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="tonearm_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="tonearm_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
