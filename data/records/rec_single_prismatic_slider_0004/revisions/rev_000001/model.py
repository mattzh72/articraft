from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_slider", assets=ASSETS)

    steel = model.material("steel", rgba=(0.42, 0.45, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    polymer = model.material("polymer", rgba=(0.12, 0.12, 0.13, 1.0))

    foundation_size = (0.270, 0.060, 0.008)
    rail_size = (0.215, 0.022, 0.016)
    stop_size = (0.012, 0.040, 0.030)

    body_length = 0.055
    body_width = 0.048
    body_height = 0.033
    body_x_center = -0.0025
    body_bottom_z = 0.010
    channel_size = (0.050, 0.030, 0.022)
    end_plate_size = (0.005, 0.044, 0.026)
    guide_block_size = (0.014, 0.004, 0.012)

    base = model.part("base")
    base.visual(
        Box(foundation_size),
        origin=Origin(xyz=(0.0, 0.0, foundation_size[2] / 2.0)),
        material=dark_steel,
        name="foundation",
    )
    base.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.0, 0.0, foundation_size[2] + rail_size[2] / 2.0)),
        material=steel,
        name="rail",
    )
    base.visual(
        Box(stop_size),
        origin=Origin(xyz=(-0.1135, 0.0, stop_size[2] / 2.0)),
        material=dark_steel,
        name="left_stop",
    )
    base.visual(
        Box(stop_size),
        origin=Origin(xyz=(0.1135, 0.0, stop_size[2] / 2.0)),
        material=dark_steel,
        name="right_stop",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.270, 0.060, 0.030)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    body_shell = (
        cq.Workplane("XY")
        .box(body_length, body_width, body_height)
        .cut(
            cq.Workplane("XY")
            .box(body_length + 0.002, channel_size[1], channel_size[2])
            .translate((0.0, 0.0, -body_height / 2.0 + channel_size[2] / 2.0))
        )
        .edges("|Z")
        .fillet(0.002)
        .translate((body_x_center, 0.0, body_bottom_z + body_height / 2.0))
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(body_shell, "carriage_body.obj", assets=ASSETS),
        material=aluminum,
        name="body",
    )
    carriage.visual(
        Box((end_plate_size[0], end_plate_size[1], 0.018)),
        origin=Origin(xyz=(0.0275, 0.0, 0.034)),
        material=aluminum,
        name="end_plate",
    )
    carriage.visual(
        Box(guide_block_size),
        origin=Origin(xyz=(-0.015, 0.013, 0.018)),
        material=polymer,
        name="guide_block_front_right",
    )
    carriage.visual(
        Box(guide_block_size),
        origin=Origin(xyz=(0.012, 0.013, 0.018)),
        material=polymer,
        name="guide_block_rear_right",
    )
    carriage.visual(
        Box(guide_block_size),
        origin=Origin(xyz=(-0.015, -0.013, 0.018)),
        material=polymer,
        name="guide_block_front_left",
    )
    carriage.visual(
        Box(guide_block_size),
        origin=Origin(xyz=(0.012, -0.013, 0.018)),
        material=polymer,
        name="guide_block_rear_left",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.060, 0.048, 0.033)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, body_bottom_z + body_height / 2.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=0.150,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("base_to_carriage")

    foundation = base.get_visual("foundation")
    rail = base.get_visual("rail")
    left_stop = base.get_visual("left_stop")
    right_stop = base.get_visual("right_stop")

    body = carriage.get_visual("body")
    end_plate = carriage.get_visual("end_plate")
    guide_block_front_right = carriage.get_visual("guide_block_front_right")
    guide_block_rear_right = carriage.get_visual("guide_block_rear_right")
    guide_block_front_left = carriage.get_visual("guide_block_front_left")
    guide_block_rear_left = carriage.get_visual("guide_block_rear_left")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        carriage,
        base,
        elem_a=guide_block_front_right,
        elem_b=rail,
        name="front_right_guide_block_contacts_rail",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a=guide_block_rear_right,
        elem_b=rail,
        name="rear_right_guide_block_contacts_rail",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a=guide_block_front_left,
        elem_b=rail,
        name="front_left_guide_block_contacts_rail",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a=guide_block_rear_left,
        elem_b=rail,
        name="rear_left_guide_block_contacts_rail",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem=body,
        negative_elem=foundation,
        min_gap=0.0015,
        max_gap=0.0025,
        name="carriage_body_clears_foundation",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a=body,
        elem_b=rail,
        min_overlap=0.040,
        name="carriage_overlaps_rail_lengthwise",
    )
    ctx.expect_origin_gap(
        base,
        carriage,
        axis="x",
        min_gap=0.074,
        max_gap=0.076,
        name="rest_pose_carriage_origin_is_75mm_left_of_base",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="x",
        positive_elem=body,
        negative_elem=left_stop,
        min_gap=0.002,
        max_gap=0.0035,
        name="rest_pose_left_stop_clearance",
    )
    ctx.expect_gap(
        base,
        carriage,
        axis="x",
        positive_elem=right_stop,
        negative_elem=end_plate,
        min_gap=0.151,
        max_gap=0.156,
        name="rest_pose_right_stop_standoff",
    )

    with ctx.pose({slide: 0.150}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a=guide_block_front_right,
            elem_b=rail,
            name="front_right_guide_block_contacts_rail_at_full_stroke",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_a=guide_block_front_left,
            elem_b=rail,
            name="front_left_guide_block_contacts_rail_at_full_stroke",
        )
        ctx.expect_origin_gap(
            carriage,
            base,
            axis="x",
            min_gap=0.074,
            max_gap=0.076,
            name="full_stroke_carriage_origin_is_75mm_right_of_base",
        )
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem=right_stop,
            negative_elem=end_plate,
            min_gap=0.002,
            max_gap=0.0035,
            name="full_stroke_right_stop_clearance",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="x",
            positive_elem=body,
            negative_elem=left_stop,
            min_gap=0.152,
            max_gap=0.156,
            name="full_stroke_left_stop_standoff",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
