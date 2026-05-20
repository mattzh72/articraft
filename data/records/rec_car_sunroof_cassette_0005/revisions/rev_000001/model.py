from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

_ORIG_GETCWD = os.getcwd
_ORIG_CHDIR = os.chdir


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        _ORIG_CHDIR("/")
        return "/"


os.getcwd = _safe_getcwd
_safe_getcwd()

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="popup_sunroof_cassette", assets=ASSETS)

    frame_finish = model.material("frame_finish", rgba=(0.19, 0.20, 0.22, 1.0))
    track_finish = model.material("track_finish", rgba=(0.12, 0.13, 0.15, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.18, 0.26, 0.31, 0.48))
    gasket_black = model.material("gasket_black", rgba=(0.05, 0.05, 0.06, 1.0))
    mechanism = model.material("mechanism", rgba=(0.53, 0.55, 0.58, 1.0))

    frame_width = 0.88
    frame_length = 1.00
    frame_height = 0.032
    bezel_thickness = 0.008
    side_rail_width = 0.08
    front_header_depth = 0.08
    rear_crossbar_depth = 0.06
    cassette_floor_depth = 0.32
    cassette_floor_thickness = 0.006
    cassette_floor_front_y = 0.16
    cassette_floor_center_y = cassette_floor_front_y + (cassette_floor_depth * 0.5)
    hinge_axis_y = (-frame_length * 0.5) + front_header_depth
    rear_crossbar_center_y = (frame_length * 0.5) - (rear_crossbar_depth * 0.5)
    side_rail_center_x = (frame_width * 0.5) - (side_rail_width * 0.5)
    opening_width = frame_width - (2.0 * side_rail_width)
    opening_length = frame_length - front_header_depth - rear_crossbar_depth
    opening_center_y = 0.5 * (rear_crossbar_depth - front_header_depth)

    glass_width = 0.70
    glass_length = 0.70
    glass_thickness = 0.008

    slide_width = 0.58
    slide_length = 0.16
    slide_travel = 0.12

    frame_outer_profile = rounded_rect_profile(frame_width, frame_length, radius=0.045)
    frame_inner_profile = [
        (x, y + opening_center_y)
        for x, y in rounded_rect_profile(opening_width, opening_length, radius=0.032)
    ]
    frame_bezel = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            frame_outer_profile,
            [frame_inner_profile],
            bezel_thickness,
            center=False,
        ),
        "assets/meshes/frame_bezel.obj",
    )
    glass_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(glass_width, glass_length, radius=0.055),
            glass_thickness,
        ),
        "assets/meshes/glass_panel.obj",
    )

    frame = model.part("frame")
    frame.visual(
        frame_bezel,
        origin=Origin(xyz=(0.0, 0.0, frame_height - bezel_thickness)),
        material=frame_finish,
        name="frame_bezel",
    )
    frame.visual(
        Box((side_rail_width, frame_length, frame_height - bezel_thickness)),
        origin=Origin(xyz=(-side_rail_center_x, 0.0, (frame_height - bezel_thickness) * 0.5)),
        material=frame_finish,
        name="left_rail",
    )
    frame.visual(
        Box((side_rail_width, frame_length, frame_height - bezel_thickness)),
        origin=Origin(xyz=(side_rail_center_x, 0.0, (frame_height - bezel_thickness) * 0.5)),
        material=frame_finish,
        name="right_rail",
    )
    frame.visual(
        Box((frame_width, front_header_depth, 0.012)),
        origin=Origin(
            xyz=(0.0, (-frame_length * 0.5) + (front_header_depth * 0.5), 0.026)
        ),
        material=frame_finish,
        name="front_header",
    )
    frame.visual(
        Box((frame_width, rear_crossbar_depth, 0.018)),
        origin=Origin(xyz=(0.0, rear_crossbar_center_y, 0.009)),
        material=frame_finish,
        name="rear_crossbar",
    )
    frame.visual(
        Box((0.60, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, hinge_axis_y + 0.012, 0.026)),
        material=track_finish,
        name="hinge_seat",
    )
    frame.visual(
        Box((opening_width, cassette_floor_depth, cassette_floor_thickness)),
        origin=Origin(
            xyz=(0.0, cassette_floor_center_y, cassette_floor_thickness * 0.5)
        ),
        material=track_finish,
        name="cassette_floor",
    )
    frame.visual(
        Box((0.03, cassette_floor_depth, 0.018)),
        origin=Origin(xyz=(-0.31, cassette_floor_center_y, 0.015)),
        material=track_finish,
        name="left_guide",
    )
    frame.visual(
        Box((0.03, cassette_floor_depth, 0.018)),
        origin=Origin(xyz=(0.31, cassette_floor_center_y, 0.015)),
        material=track_finish,
        name="right_guide",
    )
    frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_length, frame_height)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, frame_height * 0.5)),
    )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        glass_shell,
        origin=Origin(xyz=(0.0, glass_length * 0.5, 0.0)),
        material=glass_tint,
        name="glass_shell",
    )
    glass_panel.visual(
        Box((0.60, 0.032, 0.004)),
        origin=Origin(xyz=(0.0, 0.016, -0.002)),
        material=mechanism,
        name="hinge_leaf",
    )
    glass_panel.visual(
        Box((0.74, 0.035, 0.002)),
        origin=Origin(xyz=(0.0, 0.0175, -0.001)),
        material=gasket_black,
        name="front_gasket",
    )
    glass_panel.visual(
        Box((0.20, 0.10, 0.008)),
        origin=Origin(xyz=(0.0, 0.65, -0.004)),
        material=mechanism,
        name="rear_support_pad",
    )
    glass_panel.inertial = Inertial.from_geometry(
        Box((glass_width, glass_length, 0.018)),
        mass=2.7,
        origin=Origin(xyz=(0.0, glass_length * 0.5, 0.009)),
    )

    rear_slide = model.part("rear_slide")
    rear_slide.visual(
        Box((slide_width, slide_length, 0.008)),
        origin=Origin(xyz=(0.0, slide_length * 0.5, 0.012)),
        material=mechanism,
        name="slide_plate",
    )
    rear_slide.visual(
        Box((0.04, slide_length, 0.012)),
        origin=Origin(xyz=(-0.27, slide_length * 0.5, 0.006)),
        material=track_finish,
        name="left_runner",
    )
    rear_slide.visual(
        Box((0.04, slide_length, 0.012)),
        origin=Origin(xyz=(0.27, slide_length * 0.5, 0.006)),
        material=track_finish,
        name="right_runner",
    )
    rear_slide.visual(
        Box((0.22, 0.08, 0.008)),
        origin=Origin(xyz=(0.0, 0.05, 0.014)),
        material=mechanism,
        name="lift_pad",
    )
    rear_slide.inertial = Inertial.from_geometry(
        Box((slide_width, slide_length, 0.022)),
        mass=1.0,
        origin=Origin(xyz=(0.0, slide_length * 0.5, 0.011)),
    )

    model.articulation(
        "front_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=glass_panel,
        origin=Origin(xyz=(0.0, hinge_axis_y, frame_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=0.35,
        ),
    )
    model.articulation(
        "rear_slide_track",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rear_slide,
        origin=Origin(xyz=(0.0, cassette_floor_front_y, cassette_floor_thickness)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.20,
            lower=0.0,
            upper=slide_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    glass_panel = object_model.get_part("glass_panel")
    rear_slide = object_model.get_part("rear_slide")
    front_hinge = object_model.get_articulation("front_hinge")
    rear_slide_track = object_model.get_articulation("rear_slide_track")

    hinge_seat = frame.get_visual("hinge_seat")
    cassette_floor = frame.get_visual("cassette_floor")
    rear_crossbar = frame.get_visual("rear_crossbar")
    front_header = frame.get_visual("front_header")
    left_guide = frame.get_visual("left_guide")
    right_guide = frame.get_visual("right_guide")
    glass_shell = glass_panel.get_visual("glass_shell")
    hinge_leaf = glass_panel.get_visual("hinge_leaf")
    rear_support_pad = glass_panel.get_visual("rear_support_pad")
    front_gasket = glass_panel.get_visual("front_gasket")
    lift_pad = rear_slide.get_visual("lift_pad")
    slide_plate = rear_slide.get_visual("slide_plate")
    left_runner = rear_slide.get_visual("left_runner")
    right_runner = rear_slide.get_visual("right_runner")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=24)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check(
        "front_hinge_axis_is_widthwise",
        tuple(front_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected hinge axis (1, 0, 0), got {front_hinge.axis}",
    )
    ctx.check(
        "rear_slide_axis_is_longitudinal",
        tuple(rear_slide_track.axis) == (0.0, 1.0, 0.0),
        details=f"expected slide axis (0, 1, 0), got {rear_slide_track.axis}",
    )

    ctx.expect_contact(glass_panel, frame, elem_a=hinge_leaf, elem_b=hinge_seat)
    ctx.expect_contact(glass_panel, frame, elem_a=front_gasket, elem_b=front_header)
    ctx.expect_contact(glass_panel, rear_slide, elem_a=rear_support_pad, elem_b=lift_pad)
    ctx.expect_within(rear_slide, frame, axes="xy", outer_elem=cassette_floor, margin=0.001)
    ctx.expect_contact(rear_slide, frame, elem_a=left_runner, elem_b=cassette_floor)
    ctx.expect_contact(rear_slide, frame, elem_a=right_runner, elem_b=cassette_floor)
    ctx.expect_gap(
        rear_slide,
        frame,
        axis="x",
        min_gap=0.004,
        positive_elem=left_runner,
        negative_elem=left_guide,
        name="left_runner_clear_of_left_guide",
    )
    ctx.expect_gap(
        frame,
        rear_slide,
        axis="x",
        min_gap=0.004,
        positive_elem=right_guide,
        negative_elem=right_runner,
        name="right_runner_clear_of_right_guide",
    )

    front_limits = front_hinge.motion_limits
    slide_limits = rear_slide_track.motion_limits

    if front_limits is not None and front_limits.upper is not None:
        with ctx.pose({front_hinge: front_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="front_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="front_hinge_upper_no_floating")
            ctx.expect_gap(
                glass_panel,
                rear_slide,
                axis="z",
                min_gap=0.18,
                positive_elem=rear_support_pad,
                negative_elem=lift_pad,
                name="rear_of_glass_lifts_clear",
            )

    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({rear_slide_track: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_slide_upper_no_floating")
            ctx.expect_within(
                rear_slide,
                frame,
                axes="xy",
                outer_elem=cassette_floor,
                margin=0.001,
                name="rear_slide_stays_in_cassette",
            )
            ctx.expect_gap(
                frame,
                rear_slide,
                axis="y",
                max_gap=0.001,
                max_penetration=0.0001,
                positive_elem=rear_crossbar,
                negative_elem=slide_plate,
                name="rear_slide_stops_at_crossbar",
            )
            ctx.expect_gap(
                rear_slide,
                glass_panel,
                axis="y",
                min_gap=0.008,
                positive_elem=lift_pad,
                negative_elem=rear_support_pad,
                name="rear_slide_retracts_behind_glass",
            )

    if (
        front_limits is not None
        and front_limits.upper is not None
        and slide_limits is not None
        and slide_limits.upper is not None
    ):
        with ctx.pose(
            {
                front_hinge: front_limits.upper,
                rear_slide_track: slide_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="combined_open_pose_no_overlap"
            )
            ctx.fail_if_isolated_parts(name="combined_open_pose_no_floating")
            ctx.expect_gap(
                glass_panel,
                rear_slide,
                axis="z",
                min_gap=0.18,
                positive_elem=rear_support_pad,
                negative_elem=lift_pad,
                name="combined_pose_glass_clear_of_slider",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
