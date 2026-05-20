from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_dj_controller")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal", rgba=(0.055, 0.058, 0.065, 1.0))
    graphite = model.material("graphite", rgba=(0.11, 0.115, 0.125, 1.0))
    rubber = model.material("ribbed_rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    dark_slot = model.material("slot_black", rgba=(0.0, 0.0, 0.0, 1.0))
    fader_white = model.material("warm_white", rgba=(0.86, 0.84, 0.78, 1.0))
    handle_mat = model.material("rubberized_handle", rgba=(0.02, 0.022, 0.024, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.78, 0.34, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=charcoal,
        name="main_case",
    )
    housing.visual(
        Box((0.74, 0.30, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=graphite,
        name="top_plate",
    )
    # Subtle rubber bumpers around the molded body make the wide portable case
    # read less like a plain box while staying one continuous supported housing.
    housing.visual(
        Box((0.80, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.178, 0.034)),
        material=matte_black,
        name="rear_bumper",
    )
    housing.visual(
        Box((0.80, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.178, 0.034)),
        material=matte_black,
        name="front_bumper",
    )
    housing.visual(
        Box((0.018, 0.34, 0.030)),
        origin=Origin(xyz=(0.399, 0.0, 0.034)),
        material=matte_black,
        name="side_bumper_0",
    )
    housing.visual(
        Box((0.018, 0.34, 0.030)),
        origin=Origin(xyz=(-0.399, 0.0, 0.034)),
        material=matte_black,
        name="side_bumper_1",
    )

    # Stationary recessed-looking rings under the spinning jog platters.
    for idx, x in enumerate((-0.245, 0.245)):
        housing.visual(
            Cylinder(radius=0.116, length=0.004),
            origin=Origin(xyz=(x, -0.005, 0.073)),
            material=dark_slot,
            name=f"platter_recess_{idx}",
        )
        housing.visual(
            Cylinder(radius=0.080, length=0.003),
            origin=Origin(xyz=(x, -0.005, 0.0765)),
            material=matte_black,
            name=f"shadow_ring_{idx}",
        )

    housing.visual(
        Box((0.205, 0.265, 0.004)),
        origin=Origin(xyz=(0.0, -0.005, 0.073)),
        material=matte_black,
        name="center_mixer_panel",
    )
    housing.visual(
        Box((0.220, 0.020, 0.003)),
        origin=Origin(xyz=(0.0, -0.105, 0.0765)),
        material=dark_slot,
        name="crossfader_slot",
    )
    for idx, x in enumerate((-0.070, 0.070)):
        housing.visual(
            Box((0.020, 0.190, 0.003)),
            origin=Origin(xyz=(x, 0.040, 0.0765)),
            material=dark_slot,
            name=f"volume_slot_{idx}",
        )
        housing.visual(
            Box((0.040, 0.006, 0.002)),
            origin=Origin(xyz=(x, 0.122, 0.076)),
            material=brushed,
            name=f"volume_scale_hi_{idx}",
        )
        housing.visual(
            Box((0.040, 0.006, 0.002)),
            origin=Origin(xyz=(x, -0.060, 0.076)),
            material=brushed,
            name=f"volume_scale_lo_{idx}",
        )

    # Low hinge saddles fixed to the rear top edge; the movable handle barrels
    # ride just above them on the shared hinge axis.
    for idx, x in enumerate((-0.310, 0.310)):
        housing.visual(
            Box((0.082, 0.040, 0.012)),
            origin=Origin(xyz=(x, 0.158, 0.074)),
            material=matte_black,
            name=f"handle_hinge_saddle_{idx}",
        )
        housing.visual(
            Cylinder(radius=0.006, length=0.050),
            origin=Origin(xyz=(x, 0.158, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed,
            name=f"handle_pin_hint_{idx}",
        )

    # Two independent continuous jog-wheel parts on vertical axles.
    for idx, x in enumerate((-0.245, 0.245)):
        platter = model.part(f"platter_{idx}")
        platter.visual(
            Cylinder(radius=0.104, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=rubber,
            name="outer_platter",
        )
        platter.visual(
            Cylinder(radius=0.084, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
            material=brushed,
            name="top_disc",
        )
        platter.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.028)),
            material=graphite,
            name="center_hub",
        )
        platter.visual(
            Box((0.012, 0.058, 0.003)),
            origin=Origin(xyz=(0.0, 0.053, 0.0265)),
            material=fader_white,
            name="timing_mark",
        )
        model.articulation(
            f"housing_to_platter_{idx}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=platter,
            origin=Origin(xyz=(x, -0.005, 0.075)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.012, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brushed,
        name="slot_stem",
    )
    crossfader.visual(
        Box((0.052, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=fader_white,
        name="fader_cap",
    )
    model.articulation(
        "housing_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, -0.105, 0.077)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.45, lower=-0.080, upper=0.080),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    for idx, x in enumerate((-0.070, 0.070)):
        fader = model.part(f"volume_fader_{idx}")
        fader.visual(
            Box((0.008, 0.012, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=brushed,
            name="slot_stem",
        )
        fader.visual(
            Box((0.040, 0.028, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, 0.017)),
            material=fader_white,
            name="fader_cap",
        )
        model.articulation(
            f"housing_to_volume_fader_{idx}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x, 0.040, 0.077)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.35, lower=-0.065, upper=0.065),
            motion_properties=MotionProperties(damping=0.08, friction=0.03),
        )

    handle = model.part("carry_handle")
    for x in (-0.310, 0.310):
        handle.visual(
            Cylinder(radius=0.011, length=0.060),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed,
            name=f"hinge_barrel_{0 if x < 0 else 1}",
        )
        handle.visual(
            Cylinder(radius=0.010, length=0.150),
            origin=Origin(xyz=(x, 0.075, 0.012), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=handle_mat,
            name=f"handle_arm_{0 if x < 0 else 1}",
        )
    handle.visual(
        Cylinder(radius=0.014, length=0.620),
        origin=Origin(xyz=(0.0, 0.150, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_mat,
        name="grip_bar",
    )
    model.articulation(
        "housing_to_carry_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        origin=Origin(xyz=(0.0, 0.158, 0.103)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.2, lower=0.0, upper=1.45),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    crossfader = object_model.get_part("crossfader")
    cross_joint = object_model.get_articulation("housing_to_crossfader")
    handle = object_model.get_part("carry_handle")
    handle_joint = object_model.get_articulation("housing_to_carry_handle")

    for idx in (0, 1):
        platter = object_model.get_part(f"platter_{idx}")
        ctx.expect_overlap(
            platter,
            housing,
            axes="xy",
            elem_a="outer_platter",
            elem_b=f"platter_recess_{idx}",
            min_overlap=0.18,
            name=f"platter_{idx} sits over its recessed well",
        )
        ctx.expect_gap(
            platter,
            housing,
            axis="z",
            positive_elem="outer_platter",
            negative_elem=f"platter_recess_{idx}",
            min_gap=0.0005,
            max_gap=0.004,
            name=f"platter_{idx} has bearing clearance above the deck",
        )

    ctx.expect_within(
        crossfader,
        housing,
        axes="xy",
        inner_elem="slot_stem",
        outer_elem="crossfader_slot",
        margin=0.0,
        name="crossfader stem starts inside the horizontal slot",
    )
    rest_cross = ctx.part_world_position(crossfader)
    with ctx.pose({cross_joint: 0.080}):
        ctx.expect_within(
            crossfader,
            housing,
            axes="xy",
            inner_elem="slot_stem",
            outer_elem="crossfader_slot",
            margin=0.0,
            name="crossfader stem stays retained at end travel",
        )
        moved_cross = ctx.part_world_position(crossfader)
    ctx.check(
        "crossfader translates to the right along its slot",
        rest_cross is not None and moved_cross is not None and moved_cross[0] > rest_cross[0] + 0.070,
        details=f"rest={rest_cross}, moved={moved_cross}",
    )

    for idx in (0, 1):
        fader = object_model.get_part(f"volume_fader_{idx}")
        joint = object_model.get_articulation(f"housing_to_volume_fader_{idx}")
        ctx.expect_within(
            fader,
            housing,
            axes="xy",
            inner_elem="slot_stem",
            outer_elem=f"volume_slot_{idx}",
            margin=0.0,
            name=f"volume_fader_{idx} stem starts inside its vertical slot",
        )
        rest_pos = ctx.part_world_position(fader)
        with ctx.pose({joint: 0.065}):
            ctx.expect_within(
                fader,
                housing,
                axes="xy",
                inner_elem="slot_stem",
                outer_elem=f"volume_slot_{idx}",
                margin=0.0,
                name=f"volume_fader_{idx} stem remains in its slot at high setting",
            )
            moved_pos = ctx.part_world_position(fader)
        ctx.check(
            f"volume_fader_{idx} translates upward along its slot",
            rest_pos is not None and moved_pos is not None and moved_pos[1] > rest_pos[1] + 0.055,
            details=f"rest={rest_pos}, moved={moved_pos}",
        )

    def _center_xz(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[2] + hi[2]) * 0.5)

    for idx in (0, 1):
        platter = object_model.get_part(f"platter_{idx}")
        joint = object_model.get_articulation(f"housing_to_platter_{idx}")
        mark_rest = _center_xz(ctx.part_element_world_aabb(platter, elem="timing_mark"))
        with ctx.pose({joint: math.pi / 2.0}):
            mark_rotated = _center_xz(ctx.part_element_world_aabb(platter, elem="timing_mark"))
        ctx.check(
            f"platter_{idx} timing mark rotates about the axle",
            mark_rest is not None
            and mark_rotated is not None
            and abs(mark_rotated[0] - mark_rest[0]) > 0.040,
            details=f"rest={mark_rest}, rotated={mark_rotated}",
        )

    handle_rest = ctx.part_world_position(handle)
    with ctx.pose({handle_joint: 1.20}):
        grip_aabb = ctx.part_element_world_aabb(handle, elem="grip_bar")
        handle_lifted = ctx.part_world_position(handle)
    ctx.check(
        "carry handle folds upward on rear top hinges",
        handle_rest is not None
        and handle_lifted is not None
        and grip_aabb is not None
        and grip_aabb[0][2] > 0.20,
        details=f"rest={handle_rest}, lifted={handle_lifted}, grip_aabb={grip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
