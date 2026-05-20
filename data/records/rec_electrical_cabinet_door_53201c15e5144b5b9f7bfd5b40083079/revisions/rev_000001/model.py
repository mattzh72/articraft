from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pole_mounted_weatherproof_enclosure")

    grp_shell = model.material("grp_shell", rgba=(0.84, 0.86, 0.82, 1.0))
    grp_lid = model.material("grp_lid", rgba=(0.88, 0.89, 0.86, 1.0))
    galvanized = model.material("galvanized", rgba=(0.57, 0.60, 0.62, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.20, 0.21, 0.20, 1.0))

    pole_radius = 0.07
    pole_height = 2.4

    body_w = 0.62
    body_d = 0.30
    body_h = 0.22
    wall_t = 0.010
    floor_t = 0.012

    lid_w = 0.66
    lid_d = 0.34
    lid_top_t = 0.008
    lid_skirt_t = 0.008
    lid_skirt_h = 0.050
    hinge_axis_z = body_h + 0.017

    support = model.part("support")
    support.visual(
        Cylinder(radius=pole_radius, length=pole_height),
        origin=Origin(xyz=(0.0, 0.0, pole_height / 2.0)),
        material=galvanized,
        name="pole",
    )
    support.visual(
        Box((0.46, 0.010, 0.22)),
        origin=Origin(xyz=(0.0, 0.120, 1.33)),
        material=galvanized,
        name="mount_plate",
    )
    support.visual(
        Box((0.090, 0.120, 0.042)),
        origin=Origin(xyz=(-0.145, 0.060, 1.35)),
        material=galvanized,
        name="lower_left_arm",
    )
    support.visual(
        Box((0.090, 0.120, 0.042)),
        origin=Origin(xyz=(0.145, 0.060, 1.35)),
        material=galvanized,
        name="lower_right_arm",
    )
    support.visual(
        Box((0.090, 0.120, 0.042)),
        origin=Origin(xyz=(-0.145, 0.060, 1.59)),
        material=galvanized,
        name="upper_left_arm",
    )
    support.visual(
        Box((0.090, 0.120, 0.042)),
        origin=Origin(xyz=(0.145, 0.060, 1.59)),
        material=galvanized,
        name="upper_right_arm",
    )
    support.visual(
        Box((0.22, 0.016, 0.080)),
        origin=Origin(xyz=(0.0, 0.004, 1.35)),
        material=galvanized,
        name="lower_clamp_band",
    )
    support.visual(
        Box((0.22, 0.016, 0.080)),
        origin=Origin(xyz=(0.0, 0.004, 1.59)),
        material=galvanized,
        name="upper_clamp_band",
    )

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=grp_shell,
        name="floor",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-(body_w / 2.0) + wall_t / 2.0, 0.0, body_h / 2.0)),
        material=grp_shell,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=((body_w / 2.0) - wall_t / 2.0, 0.0, body_h / 2.0)),
        material=grp_shell,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, (body_d / 2.0) - wall_t / 2.0, body_h / 2.0)),
        material=grp_shell,
        name="front_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -(body_d / 2.0) + wall_t / 2.0, body_h / 2.0)),
        material=grp_shell,
        name="back_wall",
    )
    body.visual(
        Box((body_w - 0.10, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, (body_d / 2.0) - 0.012, body_h - 0.006)),
        material=gasket_dark,
        name="front_gasket_strip",
    )
    body.visual(
        Box((0.018, body_d - 0.04, 0.012)),
        origin=Origin(
            xyz=(-(body_w / 2.0) + 0.014, 0.0, body_h - 0.006),
        ),
        material=gasket_dark,
        name="left_gasket_strip",
    )
    body.visual(
        Box((0.018, body_d - 0.04, 0.012)),
        origin=Origin(
            xyz=((body_w / 2.0) - 0.014, 0.0, body_h - 0.006),
        ),
        material=gasket_dark,
        name="right_gasket_strip",
    )
    body.visual(
        Box((0.070, 0.020, 0.028)),
        origin=Origin(xyz=(-0.220, -0.158, 0.209)),
        material=grp_shell,
        name="left_hinge_pedestal",
    )
    body.visual(
        Box((0.070, 0.020, 0.028)),
        origin=Origin(xyz=(0.220, -0.158, 0.209)),
        material=grp_shell,
        name="right_hinge_pedestal",
    )
    body.visual(
        Box((0.090, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.156, body_h - 0.020)),
        material=galvanized,
        name="latch_keeper",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_top_t)),
        origin=Origin(xyz=(0.0, lid_d / 2.0, -0.011)),
        material=grp_lid,
        name="lid_top",
    )
    lid.visual(
        Box((lid_skirt_t, lid_d - 0.014, lid_skirt_h)),
        origin=Origin(
            xyz=(-(lid_w / 2.0) + lid_skirt_t / 2.0, (lid_d - 0.014) / 2.0, -0.040),
        ),
        material=grp_lid,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_skirt_t, lid_d - 0.014, lid_skirt_h)),
        origin=Origin(
            xyz=((lid_w / 2.0) - lid_skirt_t / 2.0, (lid_d - 0.014) / 2.0, -0.040),
        ),
        material=grp_lid,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_w - 2.0 * lid_skirt_t, lid_skirt_t, lid_skirt_h)),
        origin=Origin(xyz=(0.0, lid_d - (lid_skirt_t / 2.0), -0.040)),
        material=grp_lid,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_w - 0.10, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.018, -0.012)),
        material=grp_lid,
        name="rear_stiffener",
    )
    lid.visual(
        Box((0.090, 0.016, 0.060)),
        origin=Origin(xyz=(0.0, lid_d + 0.008, -0.034)),
        material=grp_lid,
        name="latch_mount",
    )
    lid.visual(
        Box((0.014, 0.008, 0.020)),
        origin=Origin(xyz=(-0.028, lid_d + 0.020, -0.017)),
        material=grp_lid,
        name="left_latch_ear",
    )
    lid.visual(
        Box((0.014, 0.008, 0.020)),
        origin=Origin(xyz=(0.028, lid_d + 0.020, -0.017)),
        material=grp_lid,
        name="right_latch_ear",
    )
    lid.visual(
        Box((0.030, 0.016, 0.018)),
        origin=Origin(xyz=(-0.220, 0.008, -0.006)),
        material=grp_lid,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.030, 0.016, 0.018)),
        origin=Origin(xyz=(0.220, 0.008, -0.006)),
        material=grp_lid,
        name="right_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(-0.220, -0.005, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=grp_lid,
        name="left_hinge_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(0.220, -0.005, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=grp_lid,
        name="right_hinge_knuckle",
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=latch_finish,
        name="pivot_hub",
    )
    latch.visual(
        Box((0.078, 0.014, 0.086)),
        origin=Origin(xyz=(0.0, 0.012, -0.050)),
        material=latch_finish,
        name="handle_paddle",
    )
    latch.visual(
        Box((0.052, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, -0.014)),
        material=latch_finish,
        name="handle_neck",
    )
    latch.visual(
        Box((0.042, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.006, -0.012)),
        material=latch_finish,
        name="cam_lobe",
    )

    model.articulation(
        "support_to_body",
        ArticulationType.FIXED,
        parent=support,
        child=body,
        origin=Origin(xyz=(0.0, 0.275, 1.25)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -(body_d / 2.0), hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, lid_d + 0.032, -0.017)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
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
    support = object_model.get_part("support")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_joint = object_model.get_articulation("lid_to_latch")

    ctx.check(
        "all prompt-critical parts exist",
        all(part is not None for part in (support, body, lid, latch)),
    )
    ctx.expect_contact(body, support, name="body mounts to the pole support")

    with ctx.pose({lid_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.24,
            name="closed lid covers the enclosure opening",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_top",
            negative_elem="front_gasket_strip",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed lid sits down onto the weather seal band",
        )

        closed_lid_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
        closed_latch = ctx.part_element_world_aabb(latch, elem="handle_paddle")

    with ctx.pose({lid_hinge: 1.20, latch_joint: 0.0}):
        open_lid_front = ctx.part_element_world_aabb(lid, elem="front_skirt")

    with ctx.pose({lid_hinge: 0.0, latch_joint: 1.00}):
        open_latch = ctx.part_element_world_aabb(latch, elem="handle_paddle")

    lid_lifts = (
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[1][2] > closed_lid_front[1][2] + 0.14
    )
    ctx.check(
        "lid front edge swings upward on the rear hinge",
        lid_lifts,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )

    latch_cams_out = (
        closed_latch is not None
        and open_latch is not None
        and open_latch[1][1] > closed_latch[1][1] + 0.035
    )
    ctx.check(
        "compression latch handle cams outward when released",
        latch_cams_out,
        details=f"closed={closed_latch}, open={open_latch}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
