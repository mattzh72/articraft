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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BODY_W = 0.340
BODY_D = 0.235
BODY_CORE_T = 0.032
BEZEL_H = 0.002
BODY_T = BODY_CORE_T + BEZEL_H

BED_W = 0.230
BED_D = 0.175
FRAME_T = 0.010
GLASS_W = BED_W - (2.0 * FRAME_T)
GLASS_D = BED_D - (2.0 * FRAME_T)
GLASS_T = 0.0015

HINGE_RADIUS = 0.0035
HINGE_Y = (-BODY_D / 2.0) + 0.008
HINGE_Z = BODY_CORE_T + HINGE_RADIUS
HINGE_OUTER_LEN = 0.0060
HINGE_INNER_LEN = 0.0080
HINGE_SPACING = 0.007
HINGE_XS = (-0.120, 0.120)

LID_W = 0.334
LID_D = 0.220
LID_T = 0.012
LID_REAR_OFFSET = 0.006
LID_Z_CENTER = (LID_T / 2.0) - 0.0015

GUIDE_W = 0.280
GUIDE_D = 0.082
GUIDE_T = 0.003
GUIDE_REAR_OFFSET = 0.0015
GUIDE_HINGE_Y = 0.0045
GUIDE_RADIUS = 0.0022
GUIDE_SUPPORT_T = 0.002
GUIDE_SUPPORT_Z_CENTER = 0.0
GUIDE_Z_CENTER = (GUIDE_T / 2.0) - GUIDE_RADIUS
GUIDE_HINGE_Z = LID_Z_CENTER + (LID_T / 2.0) + GUIDE_RADIUS - 0.0007
GUIDE_OUTER_LEN = 0.0032
GUIDE_INNER_LEN = 0.0038
GUIDE_SPACING = 0.0035
GUIDE_HINGE_XS = (-0.074, 0.074)


def _add_x_cylinder(part, *, radius, length, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_flatbed_scanner", assets=ASSETS)

    shell = model.material("scanner_shell", rgba=(0.86, 0.87, 0.89, 1.0))
    trim = model.material("scanner_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    hinge_trim = model.material("hinge_trim", rgba=(0.28, 0.30, 0.33, 1.0))
    glass = model.material("scan_glass", rgba=(0.65, 0.79, 0.88, 0.50))
    guide_surface = model.material("guide_surface", rgba=(0.93, 0.94, 0.95, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_CORE_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_CORE_T / 2.0)),
        material=shell,
        name="body_shell",
    )

    frame_z = BODY_CORE_T + (BEZEL_H / 2.0)
    bed_center_y = -0.002
    body.visual(
        Box((BED_W, FRAME_T, BEZEL_H)),
        origin=Origin(xyz=(0.0, bed_center_y + ((BED_D - FRAME_T) / 2.0), frame_z)),
        material=trim,
        name="front_bezel",
    )
    body.visual(
        Box((BED_W, FRAME_T, BEZEL_H)),
        origin=Origin(xyz=(0.0, bed_center_y - ((BED_D - FRAME_T) / 2.0), frame_z)),
        material=trim,
        name="rear_bezel",
    )
    body.visual(
        Box((FRAME_T, BED_D, BEZEL_H)),
        origin=Origin(xyz=(((BED_W - FRAME_T) / 2.0), bed_center_y, frame_z)),
        material=trim,
        name="right_bezel",
    )
    body.visual(
        Box((FRAME_T, BED_D, BEZEL_H)),
        origin=Origin(xyz=((-((BED_W - FRAME_T) / 2.0)), bed_center_y, frame_z)),
        material=trim,
        name="left_bezel",
    )
    body.visual(
        Box((GLASS_W, GLASS_D, GLASS_T)),
        origin=Origin(xyz=(0.0, bed_center_y, BODY_CORE_T - (GLASS_T / 2.0))),
        material=glass,
        name="scan_glass",
    )
    body.visual(
        Box((0.160, 0.016, 0.0016)),
        origin=Origin(xyz=(0.0, (BODY_D / 2.0) - 0.030, BODY_CORE_T + 0.0008)),
        material=trim,
        name="control_strip",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.0016),
        origin=Origin(xyz=(0.128, (BODY_D / 2.0) - 0.030, BODY_CORE_T + 0.0008)),
        material=hinge_trim,
        name="power_button",
    )

    for side, hinge_x in zip(("left", "right"), HINGE_XS):
        body.visual(
            Box((0.020, 0.006, 0.003)),
            origin=Origin(xyz=(hinge_x, HINGE_Y - 0.008, BODY_CORE_T - 0.0015)),
            material=hinge_trim,
            name=f"{side}_hinge_pedestal",
        )
        _add_x_cylinder(
            body,
            radius=HINGE_RADIUS,
            length=HINGE_OUTER_LEN,
            xyz=(hinge_x - HINGE_SPACING, HINGE_Y, HINGE_Z),
            material=hinge_trim,
            name=f"{side}_base_knuckle_outboard",
        )
        _add_x_cylinder(
            body,
            radius=HINGE_RADIUS,
            length=HINGE_OUTER_LEN,
            xyz=(hinge_x + HINGE_SPACING, HINGE_Y, HINGE_Z),
            material=hinge_trim,
            name=f"{side}_base_knuckle_inboard",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_T)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, BODY_T / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, LID_REAR_OFFSET + (LID_D / 2.0), LID_Z_CENTER)),
        material=shell,
        name="lid_shell",
    )
    lid.visual(
        Box((0.200, 0.006, 0.004)),
        origin=Origin(
            xyz=(0.0, LID_REAR_OFFSET + LID_D - 0.003, 0.002),
        ),
        material=trim,
        name="lid_front_edge",
    )
    lid.visual(
        Box((0.118, 0.012, 0.0015)),
        origin=Origin(
            xyz=(0.0, LID_REAR_OFFSET + LID_D - 0.016, LID_Z_CENTER + (LID_T / 2.0) - 0.00075),
        ),
        material=trim,
        name="lid_finger_strip",
    )

    for side, hinge_x in zip(("left", "right"), HINGE_XS):
        lid.visual(
            Box((0.014, 0.014, 0.007)),
            origin=Origin(xyz=(hinge_x, 0.004, 0.0)),
            material=hinge_trim,
            name=f"{side}_lid_hinge_arm",
        )
        _add_x_cylinder(
            lid,
            radius=HINGE_RADIUS,
            length=HINGE_INNER_LEN,
            xyz=(hinge_x, 0.0, 0.0),
            material=hinge_trim,
            name=f"{side}_lid_knuckle",
        )

    for side, hinge_x in zip(("left", "right"), GUIDE_HINGE_XS):
        _add_x_cylinder(
            lid,
            radius=GUIDE_RADIUS,
            length=GUIDE_OUTER_LEN,
            xyz=(hinge_x - GUIDE_SPACING, GUIDE_HINGE_Y, GUIDE_HINGE_Z),
            material=hinge_trim,
            name=f"{side}_lid_guide_knuckle_outboard",
        )
        _add_x_cylinder(
            lid,
            radius=GUIDE_RADIUS,
            length=GUIDE_OUTER_LEN,
            xyz=(hinge_x + GUIDE_SPACING, GUIDE_HINGE_Y, GUIDE_HINGE_Z),
            material=hinge_trim,
            name=f"{side}_lid_guide_knuckle_inboard",
        )

    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_T)),
        mass=0.7,
        origin=Origin(xyz=(0.0, LID_REAR_OFFSET + (LID_D / 2.0), LID_Z_CENTER)),
    )

    guide = model.part("paper_guide")
    guide.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_T)),
        origin=Origin(xyz=(0.0, GUIDE_REAR_OFFSET - (GUIDE_D / 2.0), GUIDE_Z_CENTER)),
        material=guide_surface,
        name="guide_panel",
    )
    guide.visual(
        Box((0.168, 0.006, 0.0044)),
        origin=Origin(xyz=(0.0, -0.003, GUIDE_SUPPORT_Z_CENTER)),
        material=hinge_trim,
        name="guide_hinge_spine",
    )
    guide.visual(
        Box((0.248, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, GUIDE_REAR_OFFSET - (GUIDE_D - 0.003), 0.006)),
        material=trim,
        name="guide_front_lip",
    )
    guide.visual(
        Box((0.006, 0.058, 0.014)),
        origin=Origin(xyz=((GUIDE_W / 2.0) - 0.003, GUIDE_REAR_OFFSET - 0.039, 0.007)),
        material=trim,
        name="guide_right_rail",
    )
    guide.visual(
        Box((0.006, 0.058, 0.014)),
        origin=Origin(xyz=((-((GUIDE_W / 2.0) - 0.003)), GUIDE_REAR_OFFSET - 0.039, 0.007)),
        material=trim,
        name="guide_left_rail",
    )

    for side, hinge_x in zip(("left", "right"), GUIDE_HINGE_XS):
        _add_x_cylinder(
            guide,
            radius=GUIDE_RADIUS,
            length=GUIDE_INNER_LEN,
            xyz=(hinge_x, 0.0, 0.0),
            material=hinge_trim,
            name=f"{side}_guide_knuckle",
        )

    guide.inertial = Inertial.from_geometry(
        Box((GUIDE_W, GUIDE_D, 0.016)),
        mass=0.16,
        origin=Origin(xyz=(0.0, GUIDE_REAR_OFFSET - (GUIDE_D / 2.0), 0.006)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "lid_to_guide",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=guide,
        origin=Origin(xyz=(0.0, GUIDE_HINGE_Y, GUIDE_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    guide = object_model.get_part("paper_guide")
    lid_hinge = object_model.get_articulation("body_to_lid")
    guide_hinge = object_model.get_articulation("lid_to_guide")

    body_shell = body.get_visual("body_shell")
    front_bezel = body.get_visual("front_bezel")
    scan_glass = body.get_visual("scan_glass")
    left_base_knuckle = body.get_visual("left_base_knuckle_inboard")
    right_base_knuckle = body.get_visual("right_base_knuckle_outboard")

    lid_shell = lid.get_visual("lid_shell")
    lid_front_edge = lid.get_visual("lid_front_edge")
    left_lid_knuckle = lid.get_visual("left_lid_knuckle")
    right_lid_knuckle = lid.get_visual("right_lid_knuckle")
    left_lid_guide_knuckle = lid.get_visual("left_lid_guide_knuckle_inboard")
    right_lid_guide_knuckle = lid.get_visual("right_lid_guide_knuckle_inboard")

    guide_panel = guide.get_visual("guide_panel")
    guide_front_lip = guide.get_visual("guide_front_lip")
    left_guide_knuckle = guide.get_visual("left_guide_knuckle")
    right_guide_knuckle = guide.get_visual("right_guide_knuckle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_within(
        body,
        body,
        axes="xy",
        inner_elem=scan_glass,
        outer_elem=body_shell,
        name="glass_sits_within_thin_body",
    )
    with ctx.pose({lid_hinge: 0.0, guide_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            elem_a=lid_shell,
            elem_b=body_shell,
            name="lid_covers_scan_bed",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=lid_shell,
            negative_elem=front_bezel,
            name="lid_seats_on_bezel",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=left_lid_knuckle,
            elem_b=left_base_knuckle,
            name="left_knuckle_hinge_is_engaged",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=right_lid_knuckle,
            elem_b=right_base_knuckle,
            name="right_knuckle_hinge_is_engaged",
        )
        ctx.expect_origin_distance(
            lid,
            body,
            axes="x",
            max_dist=0.002,
            name="lid_stays_centered_on_body",
        )
        ctx.expect_within(
            guide,
            lid,
            axes="x",
            inner_elem=guide_panel,
            outer_elem=lid_shell,
            name="paper_guide_fits_within_lid_width",
        )
        ctx.expect_gap(
            lid,
            guide,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=lid_shell,
            negative_elem=guide_panel,
            name="paper_guide_stows_behind_lid",
        )
        ctx.expect_contact(
            guide,
            lid,
            elem_a=left_guide_knuckle,
            elem_b=left_lid_guide_knuckle,
            name="left_paper_guide_hinge_is_engaged",
        )
        ctx.expect_contact(
            guide,
            lid,
            elem_a=right_guide_knuckle,
            elem_b=right_lid_guide_knuckle,
            name="right_paper_guide_hinge_is_engaged",
        )
        ctx.expect_origin_distance(
            guide,
            lid,
            axes="x",
            max_dist=0.002,
            name="paper_guide_is_centered_on_lid",
        )

    with ctx.pose({lid_hinge: 1.1, guide_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_open_pose_no_floating")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.085,
            positive_elem=lid_front_edge,
            negative_elem=scan_glass,
            name="lid_front_lifts_clear_when_opened",
        )

    with ctx.pose({lid_hinge: 0.0, guide_hinge: 1.2}):
        ctx.fail_if_parts_overlap_in_current_pose(name="guide_raised_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="guide_raised_pose_no_floating")
        ctx.expect_gap(
            guide,
            lid,
            axis="z",
            min_gap=0.055,
            positive_elem=guide_front_lip,
            negative_elem=lid_shell,
            name="paper_guide_swings_up_for_feed_support",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
