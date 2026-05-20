from __future__ import annotations

from math import cos, pi, sin

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
    model = ArticulatedObject(name="home_care_overbed_table")

    laminate = model.material("warm_laminate", rgba=(0.74, 0.58, 0.38, 1.0))
    dark_edge = model.material("dark_edge_banding", rgba=(0.18, 0.15, 0.12, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.70, 0.72, 0.72, 1.0))
    gray = model.material("medical_gray", rgba=(0.55, 0.57, 0.58, 1.0))
    black = model.material("soft_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    knob_plastic = model.material("knob_gray_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    clip_metal = model.material("spring_clip_metal", rgba=(0.42, 0.44, 0.45, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.90, 0.080, 0.060)),
        origin=Origin(xyz=(0.08, 0.0, 0.045)),
        material=gray,
        name="offset_floor_beam",
    )
    base.visual(
        Box((0.150, 0.580, 0.055)),
        origin=Origin(xyz=(-0.32, 0.0, 0.0475)),
        material=gray,
        name="pedestal_cross_foot",
    )
    base.visual(
        Box((0.095, 0.440, 0.034)),
        origin=Origin(xyz=(0.46, 0.0, 0.092)),
        material=gray,
        name="patient_caster_crossbar",
    )
    # Four walls make the fixed lower sleeve a real hollow tube for the
    # prismatic upper post, instead of hiding a solid interpenetrating proxy.
    base.visual(
        Box((0.012, 0.075, 0.545)),
        origin=Origin(xyz=(-0.3515, 0.0, 0.3475)),
        material=chrome,
        name="outer_sleeve_side_0",
    )
    base.visual(
        Box((0.012, 0.075, 0.545)),
        origin=Origin(xyz=(-0.2885, 0.0, 0.3475)),
        material=chrome,
        name="outer_sleeve_side_1",
    )
    base.visual(
        Box((0.075, 0.012, 0.545)),
        origin=Origin(xyz=(-0.32, -0.0315, 0.3475)),
        material=chrome,
        name="outer_sleeve_face_0",
    )
    base.visual(
        Box((0.075, 0.012, 0.545)),
        origin=Origin(xyz=(-0.32, 0.0315, 0.3475)),
        material=chrome,
        name="outer_sleeve_face_1",
    )
    base.visual(
        Box((0.105, 0.105, 0.014)),
        origin=Origin(xyz=(-0.32, 0.0, 0.075)),
        material=gray,
        name="sleeve_socket_plate",
    )
    for y in (-0.23, 0.23):
        base.visual(
            Cylinder(radius=0.030, length=0.018),
            origin=Origin(xyz=(-0.36, y, 0.012), rpy=(pi / 2, 0.0, 0.0)),
            material=black,
            name=f"rear_glide_{0 if y < 0 else 1}",
        )
    caster_mounts = (
        (-0.18, "caster_fork_inner_0", "caster_fork_outer_0", "caster_fork_bridge_0"),
        (0.18, "caster_fork_inner_1", "caster_fork_outer_1", "caster_fork_bridge_1"),
    )
    for y, inner_name, outer_name, bridge_name in caster_mounts:
        # Fixed caster forks cradle, but do not intersect, the rotating wheels.
        base.visual(
            Box((0.065, 0.006, 0.060)),
            origin=Origin(xyz=(0.46, y - 0.018, 0.061)),
            material=gray,
            name=inner_name,
        )
        base.visual(
            Box((0.065, 0.006, 0.060)),
            origin=Origin(xyz=(0.46, y + 0.018, 0.061)),
            material=gray,
            name=outer_name,
        )
        base.visual(
            Box((0.065, 0.042, 0.008)),
            origin=Origin(xyz=(0.46, y, 0.095)),
            material=gray,
            name=bridge_name,
        )
        base.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(0.46, y - 0.021, 0.043), rpy=(pi / 2, 0.0, 0.0)),
            material=chrome,
            name=f"caster_axle_cap_inner_{0 if y < 0 else 1}",
        )
        base.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(0.46, y + 0.021, 0.043), rpy=(pi / 2, 0.0, 0.0)),
            material=chrome,
            name=f"caster_axle_cap_outer_{0 if y < 0 else 1}",
        )

    collar = model.part("height_collar")
    collar.visual(
        Box((0.020, 0.115, 0.065)),
        origin=Origin(xyz=(-0.0475, 0.0, 0.0)),
        material=gray,
        name="collar_side_0",
    )
    collar.visual(
        Box((0.020, 0.115, 0.065)),
        origin=Origin(xyz=(0.0475, 0.0, 0.0)),
        material=gray,
        name="collar_side_1",
    )
    collar.visual(
        Box((0.115, 0.020, 0.065)),
        origin=Origin(xyz=(0.0, -0.0475, 0.0)),
        material=gray,
        name="collar_face_0",
    )
    collar.visual(
        Box((0.115, 0.020, 0.065)),
        origin=Origin(xyz=(0.0, 0.0475, 0.0)),
        material=gray,
        name="collar_face_1",
    )
    collar.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(0.0, -0.075, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=chrome,
        name="threaded_hub",
    )

    knob = model.part("collar_knob")
    knob.visual(
        Cylinder(radius=0.022, length=0.029),
        origin=Origin(xyz=(0.0, -0.0145, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=knob_plastic,
        name="knob_core",
    )
    for index in range(6):
        angle = index * pi / 3.0
        knob.visual(
            Cylinder(radius=0.0075, length=0.031),
            origin=Origin(
                xyz=(0.020 * cos(angle), -0.0155, 0.020 * sin(angle)),
                rpy=(pi / 2, 0.0, 0.0),
            ),
            material=knob_plastic,
            name=f"knob_lobe_{index}",
        )
    knob.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, -0.0315, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=gray,
        name="knob_face_cap",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.051, 0.051, 0.640)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="inner_post",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.820, 0.420, 0.032)),
        origin=Origin(xyz=(0.455, 0.0, 0.016)),
        material=laminate,
        name="laminate_top",
    )
    tray.visual(
        Box((0.830, 0.018, 0.044)),
        origin=Origin(xyz=(0.455, -0.219, 0.022)),
        material=dark_edge,
        name="front_edge_band",
    )
    tray.visual(
        Box((0.830, 0.014, 0.036)),
        origin=Origin(xyz=(0.455, 0.217, 0.018)),
        material=dark_edge,
        name="rear_edge_band",
    )
    tray.visual(
        Box((0.018, 0.420, 0.036)),
        origin=Origin(xyz=(0.045, 0.0, 0.018)),
        material=dark_edge,
        name="pedestal_edge_band",
    )
    tray.visual(
        Box((0.018, 0.420, 0.036)),
        origin=Origin(xyz=(0.865, 0.0, 0.018)),
        material=dark_edge,
        name="patient_edge_band",
    )
    tray.visual(
        Box((0.455, 0.045, 0.036)),
        origin=Origin(xyz=(0.250, 0.0, -0.018)),
        material=chrome,
        name="cantilever_arm",
    )
    tray.visual(
        Box((0.006, 0.065, 0.180)),
        origin=Origin(xyz=(0.0255, 0.0, -0.020)),
        material=chrome,
        name="mast_mount_plate",
    )
    tray.visual(
        Box((0.035, 0.018, 0.020)),
        origin=Origin(xyz=(0.235, -0.226, 0.042)),
        material=clip_metal,
        name="clip_hinge_block_0",
    )
    tray.visual(
        Box((0.035, 0.018, 0.020)),
        origin=Origin(xyz=(0.675, -0.226, 0.042)),
        material=clip_metal,
        name="clip_hinge_block_1",
    )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(
        Cylinder(radius=0.005, length=0.620),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=clip_metal,
        name="hinge_pin",
    )
    for index, x in enumerate((-0.220, 0.220)):
        clip_bar.visual(
            Box((0.025, 0.068, 0.006)),
            origin=Origin(xyz=(x, 0.034, 0.002)),
            material=clip_metal,
            name=f"short_support_{index}",
        )
    clip_bar.visual(
        Box((0.600, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.076, 0.012)),
        material=clip_metal,
        name="chart_clip_bar",
    )

    for index, y in enumerate((-0.18, 0.18)):
        caster = model.part(f"front_caster_{index}")
        caster.visual(
            Cylinder(radius=0.030, length=0.024),
            origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
            material=black,
            name="rubber_tire",
        )
        caster.visual(
            Cylinder(radius=0.014, length=0.030),
            origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
            material=chrome,
            name="wheel_hub",
        )
        model.articulation(
            f"base_to_front_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(0.46, y, 0.043)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=18.0),
        )

    model.articulation(
        "base_to_height_collar",
        ArticulationType.FIXED,
        parent=base,
        child=collar,
        origin=Origin(xyz=(-0.32, 0.0, 0.620)),
    )
    model.articulation(
        "collar_to_knob",
        ArticulationType.CONTINUOUS,
        parent=collar,
        child=knob,
        origin=Origin(xyz=(0.0, -0.0925, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(-0.32, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.22),
    )
    model.articulation(
        "mast_to_tray",
        ArticulationType.FIXED,
        parent=mast,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
    )
    model.articulation(
        "tray_to_clip_bar",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(0.455, -0.226, 0.057)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.15),
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

    base = object_model.get_part("base")
    collar = object_model.get_part("height_collar")
    knob = object_model.get_part("collar_knob")
    mast = object_model.get_part("mast")
    tray = object_model.get_part("tray")
    clip_bar = object_model.get_part("clip_bar")
    caster_0 = object_model.get_part("front_caster_0")
    caster_1 = object_model.get_part("front_caster_1")

    height_slide = object_model.get_articulation("base_to_mast")
    knob_spin = object_model.get_articulation("collar_to_knob")
    clip_hinge = object_model.get_articulation("tray_to_clip_bar")
    caster_spin_0 = object_model.get_articulation("base_to_front_caster_0")
    caster_spin_1 = object_model.get_articulation("base_to_front_caster_1")

    ctx.check(
        "primary mechanisms are articulated",
        height_slide.articulation_type == ArticulationType.PRISMATIC
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and clip_hinge.articulation_type == ArticulationType.REVOLUTE
        and caster_spin_0.articulation_type == ArticulationType.CONTINUOUS
        and caster_spin_1.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected sliding post, continuous knob, continuous caster wheels, and hinged clip bar.",
    )
    ctx.expect_contact(
        collar,
        base,
        contact_tol=0.001,
        elem_a="collar_side_0",
        elem_b="outer_sleeve_side_0",
        name="height collar bears on the lower sleeve",
    )
    ctx.expect_contact(
        knob,
        collar,
        contact_tol=0.001,
        elem_a="knob_core",
        elem_b="threaded_hub",
        name="knob seats on threaded hub",
    )
    ctx.expect_contact(
        clip_bar,
        tray,
        contact_tol=0.001,
        elem_a="hinge_pin",
        elem_b="clip_hinge_block_0",
        name="clip bar rests on hinge blocks",
    )
    ctx.expect_contact(
        tray,
        mast,
        contact_tol=0.001,
        elem_a="mast_mount_plate",
        elem_b="inner_post",
        name="cantilever tray mount touches sliding post",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="x",
        min_overlap=0.35,
        elem_a="laminate_top",
        elem_b="offset_floor_beam",
        name="tray reaches over patient-side base footprint",
    )
    ctx.expect_overlap(
        caster_0,
        base,
        axes="xy",
        min_overlap=0.020,
        elem_a="rubber_tire",
        elem_b="caster_fork_bridge_0",
        name="front caster 0 is tucked under the patient crossbar",
    )
    ctx.expect_overlap(
        caster_1,
        base,
        axes="xy",
        min_overlap=0.020,
        elem_a="rubber_tire",
        elem_b="caster_fork_bridge_1",
        name="front caster 1 is tucked under the patient crossbar",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        min_overlap=0.08,
        elem_a="inner_post",
        elem_b="outer_sleeve_side_0",
        name="low post position remains inserted in sleeve",
    )

    low_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({height_slide: 0.22}):
        high_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            min_overlap=0.08,
            elem_a="inner_post",
            elem_b="outer_sleeve_side_0",
            name="raised post remains inserted in sleeve",
        )
    ctx.check(
        "height slide raises the tray",
        low_tray_pos is not None
        and high_tray_pos is not None
        and high_tray_pos[2] > low_tray_pos[2] + 0.20,
        details=f"low={low_tray_pos}, high={high_tray_pos}",
    )

    closed_clip_box = ctx.part_element_world_aabb(clip_bar, elem="chart_clip_bar")
    closed_clip_z = (
        (closed_clip_box[0][2] + closed_clip_box[1][2]) / 2.0
        if closed_clip_box is not None
        else None
    )
    with ctx.pose({clip_hinge: 0.80}):
        raised_clip_box = ctx.part_element_world_aabb(clip_bar, elem="chart_clip_bar")
        raised_clip_z = (
            (raised_clip_box[0][2] + raised_clip_box[1][2]) / 2.0
            if raised_clip_box is not None
            else None
        )
    ctx.check(
        "clip bar pivots upward from the tray rim",
        closed_clip_z is not None
        and raised_clip_z is not None
        and raised_clip_z > closed_clip_z + 0.04,
        details=f"closed_z={closed_clip_z}, raised_z={raised_clip_z}",
    )

    return ctx.report()


object_model = build_object_model()
