from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
MESH_DIR = ASSETS.mesh_dir


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_d = depth / 2.0
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_office_chair", assets=ASSETS)

    aluminum = model.material("aluminum", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.14, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.27, 0.29, 1.0))
    soft_black = model.material("soft_black", rgba=(0.09, 0.09, 0.10, 1.0))

    outer_sleeve = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.048, segments=48),
            [_circle_profile(0.035, segments=48)],
            height=0.22,
            center=True,
        ),
        MESH_DIR / "chair_outer_sleeve.obj",
    )
    hub_shell_ring = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.072, segments=48),
            [_circle_profile(0.038, segments=48)],
            height=0.05,
            center=True,
        ),
        MESH_DIR / "chair_hub_shell.obj",
    )
    hub_cap_ring = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.056, segments=48),
            [_circle_profile(0.038, segments=48)],
            height=0.07,
            center=True,
        ),
        MESH_DIR / "chair_hub_cap.obj",
    )
    guide_channel = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(0.052, 0.094),
            [_rect_profile(0.034, 0.076)],
            height=0.16,
            center=True,
        ),
        MESH_DIR / "chair_arm_guide.obj",
    )
    leg_tip_bracket = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(0.050, 0.046),
            [_circle_profile(0.014, segments=32)],
            height=0.038,
            center=True,
        ),
        MESH_DIR / "chair_leg_tip_bracket.obj",
    )

    base = model.part("base")
    base.visual(
        hub_shell_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_metal,
        name="hub_shell",
    )
    base.visual(
        hub_cap_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=aluminum,
        name="hub_cap",
    )
    base.visual(
        outer_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=aluminum,
        name="outer_sleeve",
    )
    for index, angle in enumerate((0.0, 72.0, 144.0, 216.0, 288.0), start=1):
        yaw = math.radians(angle)
        rib_radius = 0.21
        spine_radius = 0.16
        tip_radius = 0.382
        base.visual(
            Box((0.31, 0.055, 0.028)),
            origin=Origin(
                xyz=(rib_radius * math.cos(yaw), rib_radius * math.sin(yaw), 0.06),
                rpy=(0.0, 0.0, yaw),
            ),
            material=aluminum,
            name=f"leg_rib_{index}",
        )
        base.visual(
            Box((0.20, 0.028, 0.018)),
            origin=Origin(
                xyz=(spine_radius * math.cos(yaw), spine_radius * math.sin(yaw), 0.082),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_metal,
            name=f"leg_spine_{index}",
        )
        base.visual(
            Box((0.036, 0.052, 0.044)),
            origin=Origin(
                xyz=(tip_radius * math.cos(yaw), tip_radius * math.sin(yaw), 0.107),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_metal,
            name=f"leg_tip_{index}",
        )
        base.visual(
            leg_tip_bracket,
            origin=Origin(
                xyz=(0.382 * math.cos(yaw), 0.382 * math.sin(yaw), 0.090),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_metal,
            name=f"leg_tip_bracket_{index}",
        )

    seat_frame = model.part("seat_frame")
    seat_frame.visual(
        Cylinder(radius=0.031, length=0.31),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=dark_metal,
        name="inner_post",
    )
    seat_frame.visual(
        Box((0.42, 0.34, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=aluminum,
        name="seat_plate",
    )
    seat_frame.visual(
        Box((0.48, 0.12, 0.008)),
        origin=Origin(xyz=(0.0, 0.12, 0.318)),
        material=aluminum,
        name="front_plate_flange",
    )
    seat_frame.visual(
        Box((0.24, 0.10, 0.010)),
        origin=Origin(xyz=(0.0, -0.14, 0.314)),
        material=aluminum,
        name="rear_bridge",
    )
    seat_frame.visual(
        Box((0.030, 0.28, 0.05)),
        origin=Origin(xyz=(-0.214, 0.0, 0.298)),
        material=dark_metal,
        name="left_side_frame",
    )
    seat_frame.visual(
        Box((0.030, 0.28, 0.05)),
        origin=Origin(xyz=(0.214, 0.0, 0.298)),
        material=dark_metal,
        name="right_side_frame",
    )
    seat_frame.visual(
        guide_channel,
        origin=Origin(xyz=(-0.245, 0.0, 0.28)),
        material=dark_metal,
        name="left_guide_channel",
    )
    seat_frame.visual(
        guide_channel,
        origin=Origin(xyz=(0.245, 0.0, 0.28)),
        material=dark_metal,
        name="right_guide_channel",
    )
    seat_frame.visual(
        Box((0.18, 0.12, 0.09)),
        origin=Origin(xyz=(0.0, -0.18, 0.27)),
        material=dark_metal,
        name="housing_shell",
    )
    seat_frame.visual(
        Cylinder(radius=0.02, length=0.16),
        origin=Origin(xyz=(0.0, -0.22, 0.305), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="housing_cap",
    )

    seat_cushion = model.part("seat_cushion")
    seat_cushion.visual(
        Box((0.46, 0.50, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=charcoal,
        name="seat_pad",
    )

    tilt_lock_knob = model.part("tilt_lock_knob")
    tilt_lock_knob.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="knob_body",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.14, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, -0.03, 0.04)),
        material=graphite,
        name="hinge_block",
    )
    backrest.visual(
        Box((0.10, 0.06, 0.20)),
        origin=Origin(xyz=(0.0, -0.04, 0.16)),
        material=dark_metal,
        name="back_spine",
    )
    backrest.visual(
        Box((0.44, 0.08, 0.66)),
        origin=Origin(xyz=(0.0, -0.07, 0.49), rpy=(-0.10, 0.0, 0.0)),
        material=charcoal,
        name="back_panel",
    )
    backrest.visual(
        Box((0.34, 0.09, 0.11)),
        origin=Origin(xyz=(0.0, -0.08, 0.84), rpy=(-0.10, 0.0, 0.0)),
        material=charcoal,
        name="headrest_cap",
    )

    lumbar_panel = model.part("lumbar_panel")
    lumbar_panel.visual(
        Box((0.28, 0.036, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="lumbar_pad",
    )

    left_arm = model.part("left_arm")
    left_arm.visual(
        Box((0.032, 0.072, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_metal,
        name="left_post",
    )
    left_arm.visual(
        Box((0.056, 0.05, 0.04)),
        origin=Origin(xyz=(0.028, 0.0, 0.25)),
        material=dark_metal,
        name="left_neck",
    )
    left_arm.visual(
        Box((0.30, 0.09, 0.036)),
        origin=Origin(xyz=(0.07, 0.0, 0.288)),
        material=charcoal,
        name="left_arm_pad",
    )

    right_arm = model.part("right_arm")
    right_arm.visual(
        Box((0.032, 0.072, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_metal,
        name="right_post",
    )
    right_arm.visual(
        Box((0.056, 0.05, 0.04)),
        origin=Origin(xyz=(-0.028, 0.0, 0.25)),
        material=dark_metal,
        name="right_neck",
    )
    right_arm.visual(
        Box((0.30, 0.09, 0.036)),
        origin=Origin(xyz=(-0.07, 0.0, 0.288)),
        material=charcoal,
        name="right_arm_pad",
    )

    caster_parts = []
    for index in range(1, 6):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.009, length=0.062),
            origin=Origin(xyz=(0.0, 0.0, -0.031)),
            material=dark_metal,
            name=f"caster_stem_{index}",
        )
        caster.visual(
            Box((0.024, 0.064, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.052)),
            material=graphite,
            name=f"caster_bridge_{index}",
        )
        caster.visual(
            Box((0.008, 0.012, 0.046)),
            origin=Origin(xyz=(0.0, -0.022, -0.075)),
            material=graphite,
            name=f"caster_left_fork_{index}",
        )
        caster.visual(
            Box((0.008, 0.012, 0.046)),
            origin=Origin(xyz=(0.0, 0.022, -0.075)),
            material=graphite,
            name=f"caster_right_fork_{index}",
        )
        caster.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.0, -0.018, -0.091), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=soft_black,
            name=f"caster_wheel_inner_{index}",
        )
        caster.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.0, 0.018, -0.091), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=soft_black,
            name=f"caster_wheel_outer_{index}",
        )
        caster_parts.append(caster)

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=base,
        child=seat_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.10),
    )
    model.articulation(
        "seat_frame_to_cushion",
        ArticulationType.FIXED,
        parent=seat_frame,
        child=seat_cushion,
        origin=Origin(xyz=(0.0, 0.02, 0.321)),
    )
    model.articulation(
        "seat_frame_to_tilt_lock_knob",
        ArticulationType.FIXED,
        parent=seat_frame,
        child=tilt_lock_knob,
        origin=Origin(xyz=(0.09, -0.16, 0.255)),
    )
    model.articulation(
        "back_tilt",
        ArticulationType.REVOLUTE,
        parent=seat_frame,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.24, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.4, lower=0.0, upper=0.42),
    )
    model.articulation(
        "backrest_to_lumbar_panel",
        ArticulationType.FIXED,
        parent=backrest,
        child=lumbar_panel,
        origin=Origin(xyz=(0.0, -0.02, 0.37)),
    )
    model.articulation(
        "left_arm_slide",
        ArticulationType.PRISMATIC,
        parent=seat_frame,
        child=left_arm,
        origin=Origin(xyz=(-0.245, 0.0, 0.21)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.08),
    )
    model.articulation(
        "right_arm_slide",
        ArticulationType.PRISMATIC,
        parent=seat_frame,
        child=right_arm,
        origin=Origin(xyz=(0.245, 0.0, 0.21)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.08),
    )
    for index, angle in enumerate((0.0, 72.0, 144.0, 216.0, 288.0), start=1):
        yaw = math.radians(angle)
        model.articulation(
            f"caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster_parts[index - 1],
            origin=Origin(xyz=(0.382 * math.cos(yaw), 0.382 * math.sin(yaw), 0.085)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    seat_frame = object_model.get_part("seat_frame")
    seat_cushion = object_model.get_part("seat_cushion")
    tilt_lock_knob = object_model.get_part("tilt_lock_knob")
    backrest = object_model.get_part("backrest")
    lumbar_panel = object_model.get_part("lumbar_panel")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")

    seat_height = object_model.get_articulation("seat_height")
    back_tilt = object_model.get_articulation("back_tilt")
    left_arm_slide = object_model.get_articulation("left_arm_slide")
    right_arm_slide = object_model.get_articulation("right_arm_slide")
    caster_swivel_1 = object_model.get_articulation("caster_swivel_1")

    hub_shell = base.get_visual("hub_shell")
    outer_sleeve = base.get_visual("outer_sleeve")
    seat_plate = seat_frame.get_visual("seat_plate")
    housing_shell = seat_frame.get_visual("housing_shell")
    left_guide_channel = seat_frame.get_visual("left_guide_channel")
    right_guide_channel = seat_frame.get_visual("right_guide_channel")
    inner_post = seat_frame.get_visual("inner_post")
    seat_pad = seat_cushion.get_visual("seat_pad")
    knob_body = tilt_lock_knob.get_visual("knob_body")
    hinge_block = backrest.get_visual("hinge_block")
    back_panel = backrest.get_visual("back_panel")
    headrest_cap = backrest.get_visual("headrest_cap")
    lumbar_pad = lumbar_panel.get_visual("lumbar_pad")
    left_post = left_arm.get_visual("left_post")
    right_post = right_arm.get_visual("right_post")
    left_arm_pad = left_arm.get_visual("left_arm_pad")
    right_arm_pad = right_arm.get_visual("right_arm_pad")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.04)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(seat_cushion, seat_frame, axes="xy", min_overlap=0.14)
    ctx.expect_gap(
        seat_cushion,
        seat_frame,
        axis="z",
        positive_elem=seat_pad,
        negative_elem=seat_plate,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_within(
        seat_frame,
        base,
        axes="xy",
        inner_elem=inner_post,
        outer_elem=outer_sleeve,
    )
    ctx.expect_gap(
        seat_cushion,
        base,
        axis="z",
        positive_elem=seat_pad,
        negative_elem=hub_shell,
        min_gap=0.25,
    )
    ctx.expect_contact(backrest, seat_frame, elem_a=hinge_block, elem_b=housing_shell)
    ctx.expect_origin_distance(backrest, seat_cushion, axes="x", max_dist=0.03)
    ctx.expect_gap(
        backrest,
        seat_cushion,
        axis="z",
        positive_elem=headrest_cap,
        negative_elem=seat_pad,
        min_gap=0.34,
    )
    ctx.expect_contact(lumbar_panel, backrest, elem_a=lumbar_pad, elem_b=back_panel)
    ctx.expect_gap(
        lumbar_panel,
        seat_cushion,
        axis="z",
        positive_elem=lumbar_pad,
        negative_elem=seat_pad,
        min_gap=0.10,
    )
    ctx.expect_contact(tilt_lock_knob, seat_frame, elem_a=knob_body, elem_b=housing_shell)
    ctx.expect_gap(
        tilt_lock_knob,
        seat_frame,
        axis="x",
        positive_elem=knob_body,
        negative_elem=housing_shell,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        seat_cushion,
        tilt_lock_knob,
        axis="z",
        positive_elem=seat_pad,
        negative_elem=knob_body,
        min_gap=0.04,
    )
    ctx.expect_within(
        left_arm,
        seat_frame,
        axes="xy",
        inner_elem=left_post,
        outer_elem=left_guide_channel,
    )
    ctx.expect_within(
        right_arm,
        seat_frame,
        axes="xy",
        inner_elem=right_post,
        outer_elem=right_guide_channel,
    )
    ctx.expect_gap(
        left_arm,
        seat_cushion,
        axis="z",
        positive_elem=left_arm_pad,
        negative_elem=seat_pad,
        min_gap=0.045,
    )
    ctx.expect_gap(
        right_arm,
        seat_cushion,
        axis="z",
        positive_elem=right_arm_pad,
        negative_elem=seat_pad,
        min_gap=0.045,
    )
    for index in range(1, 6):
        caster = object_model.get_part(f"caster_{index}")
        leg_tip = base.get_visual(f"leg_tip_{index}")
        caster_stem = caster.get_visual(f"caster_stem_{index}")
        ctx.expect_contact(
            base,
            caster,
            elem_a=leg_tip,
            elem_b=caster_stem,
            name=f"caster_{index}_mounted_under_leg_tip",
        )

    with ctx.pose(
        {
            seat_height: 0.09,
            back_tilt: 0.35,
            left_arm_slide: 0.08,
            right_arm_slide: 0.08,
            caster_swivel_1: 1.20,
        }
    ):
        ctx.expect_within(
            seat_frame,
            base,
            axes="xy",
            inner_elem=inner_post,
            outer_elem=outer_sleeve,
        )
        ctx.expect_gap(
            seat_cushion,
            base,
            axis="z",
            positive_elem=seat_pad,
            negative_elem=hub_shell,
            min_gap=0.34,
        )
        ctx.expect_contact(backrest, seat_frame, elem_a=hinge_block, elem_b=housing_shell)
        ctx.expect_gap(
            backrest,
            seat_cushion,
            axis="z",
            positive_elem=headrest_cap,
            negative_elem=seat_pad,
            min_gap=0.34,
        )
        ctx.expect_within(
            left_arm,
            seat_frame,
            axes="xy",
            inner_elem=left_post,
            outer_elem=left_guide_channel,
        )
        ctx.expect_within(
            right_arm,
            seat_frame,
            axes="xy",
            inner_elem=right_post,
            outer_elem=right_guide_channel,
        )
        ctx.expect_contact(
            base,
            object_model.get_part("caster_1"),
            elem_a=base.get_visual("leg_tip_1"),
            elem_b=object_model.get_part("caster_1").get_visual("caster_stem_1"),
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
