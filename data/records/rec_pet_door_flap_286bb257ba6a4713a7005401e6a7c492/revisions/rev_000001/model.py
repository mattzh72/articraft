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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_cat_flap")

    frame_white = model.material("frame_white", rgba=(0.92, 0.92, 0.90, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.76, 0.77, 0.78, 1.0))
    hinge_grey = model.material("hinge_grey", rgba=(0.40, 0.42, 0.45, 1.0))
    flap_clear = model.material("flap_clear", rgba=(0.70, 0.82, 0.90, 0.35))
    flap_edge = model.material("flap_edge", rgba=(0.66, 0.67, 0.70, 1.0))
    lock_dark = model.material("lock_dark", rgba=(0.24, 0.26, 0.30, 1.0))

    outer_width = 0.26
    outer_height = 0.305
    frame_depth = 0.05
    opening_width = 0.17
    opening_height = 0.19
    side_width = (outer_width - opening_width) * 0.5
    sill_height = 0.055
    header_height = outer_height - opening_height - sill_height
    opening_top = sill_height + opening_height
    hinge_z = opening_top - 0.001

    fixed_frame = model.part("fixed_frame")
    fixed_frame.visual(
        Box((side_width, frame_depth, outer_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 + side_width * 0.5), 0.0, outer_height * 0.5)),
        material=frame_white,
        name="left_jamb",
    )
    fixed_frame.visual(
        Box((side_width, frame_depth, outer_height)),
        origin=Origin(xyz=((opening_width * 0.5 + side_width * 0.5), 0.0, outer_height * 0.5)),
        material=frame_white,
        name="right_jamb",
    )
    fixed_frame.visual(
        Box((opening_width, frame_depth, header_height)),
        origin=Origin(xyz=(0.0, 0.0, outer_height - header_height * 0.5)),
        material=frame_white,
        name="top_header",
    )
    fixed_frame.visual(
        Box((opening_width, frame_depth, sill_height)),
        origin=Origin(xyz=(0.0, 0.0, sill_height * 0.5)),
        material=frame_white,
        name="bottom_sill",
    )

    fixed_frame.visual(
        Box((0.052, 0.006, 0.315)),
        origin=Origin(xyz=(-0.104, 0.028, 0.1575)),
        material=trim_grey,
        name="left_front_trim",
    )
    fixed_frame.visual(
        Box((0.031, 0.006, 0.315)),
        origin=Origin(xyz=(0.1145, 0.028, 0.1575)),
        material=trim_grey,
        name="right_front_trim",
    )
    fixed_frame.visual(
        Box((0.144, 0.006, 0.070)),
        origin=Origin(xyz=(-0.013, 0.028, 0.2775)),
        material=trim_grey,
        name="top_front_trim_left",
    )
    fixed_frame.visual(
        Box((0.031, 0.006, 0.070)),
        origin=Origin(xyz=(0.1145, 0.028, 0.2775)),
        material=trim_grey,
        name="top_front_trim_right",
    )
    fixed_frame.visual(
        Box((0.17, 0.006, 0.065)),
        origin=Origin(xyz=(0.0, 0.028, 0.0325)),
        material=trim_grey,
        name="bottom_front_trim",
    )
    fixed_frame.visual(
        Box((0.150, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.031, 0.255)),
        material=trim_grey,
        name="hinge_hood",
    )

    guide_center_x = 0.091
    guide_center_z = 0.170
    guide_height = 0.23
    fixed_frame.visual(
        Box((0.024, 0.003, guide_height)),
        origin=Origin(xyz=(guide_center_x, 0.0265, guide_center_z)),
        material=hinge_grey,
        name="guide_backer",
    )
    fixed_frame.visual(
        Box((0.003, 0.003, guide_height)),
        origin=Origin(xyz=(0.0805, 0.035, guide_center_z)),
        material=hinge_grey,
        name="guide_inner_lip",
    )
    fixed_frame.visual(
        Box((0.003, 0.003, guide_height)),
        origin=Origin(xyz=(0.1015, 0.035, guide_center_z)),
        material=hinge_grey,
        name="guide_outer_lip",
    )
    fixed_frame.visual(
        Box((0.024, 0.011, 0.004)),
        origin=Origin(xyz=(guide_center_x, 0.031, 0.287)),
        material=hinge_grey,
        name="guide_top_cap",
    )
    fixed_frame.visual(
        Box((0.024, 0.011, 0.004)),
        origin=Origin(xyz=(guide_center_x, 0.031, 0.053)),
        material=hinge_grey,
        name="guide_bottom_cap",
    )
    fixed_frame.inertial = Inertial.from_geometry(
        Box((0.275, 0.06, 0.315)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.005, 0.1575)),
    )

    flap = model.part("flap")
    flap_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.164, 0.182, 0.014), 0.0035),
        "cat_flap_plate",
    )
    flap.visual(
        flap_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.091), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=flap_clear,
        name="flap_plate",
    )
    flap.visual(
        Box((0.168, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=flap_edge,
        name="top_rail",
    )
    flap.visual(
        Box((0.164, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.177)),
        material=flap_edge,
        name="bottom_rail",
    )
    flap.visual(
        Box((0.007, 0.008, 0.170)),
        origin=Origin(xyz=(-0.0785, 0.0, -0.091)),
        material=flap_edge,
        name="left_edge_rail",
    )
    flap.visual(
        Box((0.007, 0.008, 0.170)),
        origin=Origin(xyz=(0.0785, 0.0, -0.091)),
        material=flap_edge,
        name="right_edge_rail",
    )
    for index, x_pos in enumerate((-0.055, 0.0, 0.055)):
        flap.visual(
            Cylinder(radius=0.003, length=0.028),
            origin=Origin(xyz=(x_pos, 0.005, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_grey,
            name=f"hinge_knuckle_{index}",
        )
    flap.inertial = Inertial.from_geometry(
        Box((0.170, 0.012, 0.182)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.091)),
    )

    security_panel = model.part("security_panel")
    security_panel.visual(
        Box((0.018, 0.0025, 0.130)),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=lock_dark,
        name="lockout_blade",
    )
    security_panel.visual(
        Box((0.016, 0.007, 0.012)),
        origin=Origin(xyz=(0.002, 0.0075, 0.006)),
        material=lock_dark,
        name="lock_pull",
    )
    security_panel.visual(
        Box((0.006, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0035, 0.006)),
        material=lock_dark,
        name="lock_pull_stem",
    )
    security_panel.visual(
        Box((0.006, 0.003, 0.040)),
        origin=Origin(xyz=(0.0, -0.0015, -0.035)),
        material=lock_dark,
        name="guide_runner_upper",
    )
    security_panel.visual(
        Box((0.006, 0.003, 0.040)),
        origin=Origin(xyz=(0.0, -0.0015, -0.095)),
        material=lock_dark,
        name="guide_runner_lower",
    )
    security_panel.inertial = Inertial.from_geometry(
        Box((0.022, 0.012, 0.142)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.004, -0.059)),
    )

    flap_hinge = model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=fixed_frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=-1.15,
            upper=1.15,
        ),
    )

    security_slide = model.articulation(
        "frame_to_security_panel",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=security_panel,
        origin=Origin(xyz=(0.089, 0.031, 0.246)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.18,
            lower=0.0,
            upper=0.055,
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

    frame = object_model.get_part("fixed_frame")
    flap = object_model.get_part("flap")
    security_panel = object_model.get_part("security_panel")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    security_slide = object_model.get_articulation("frame_to_security_panel")

    with ctx.pose({flap_hinge: 0.0, security_slide: 0.0}):
        ctx.expect_gap(
            flap,
            frame,
            axis="x",
            positive_elem="flap_plate",
            negative_elem="left_jamb",
            min_gap=0.002,
            max_gap=0.006,
            name="left flap clearance inside frame",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="x",
            positive_elem="right_jamb",
            negative_elem="flap_plate",
            min_gap=0.002,
            max_gap=0.006,
            name="right flap clearance inside frame",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="z",
            positive_elem="top_header",
            negative_elem="flap_plate",
            min_gap=0.0005,
            max_gap=0.010,
            name="top flap clearance below hinge header",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="z",
            positive_elem="flap_plate",
            negative_elem="bottom_sill",
            min_gap=0.004,
            max_gap=0.012,
            name="bottom flap clearance above sill",
        )

    flap_closed = ctx.part_element_world_aabb(flap, elem="flap_plate")
    with ctx.pose({flap_hinge: 0.90}):
        flap_open_forward = ctx.part_element_world_aabb(flap, elem="flap_plate")
    with ctx.pose({flap_hinge: -0.90}):
        flap_open_backward = ctx.part_element_world_aabb(flap, elem="flap_plate")

    ctx.check(
        "flap opens outward toward +y",
        flap_closed is not None
        and flap_open_forward is not None
        and flap_open_forward[1][1] > flap_closed[1][1] + 0.06,
        details=f"closed={flap_closed}, outward={flap_open_forward}",
    )
    ctx.check(
        "flap swings inward toward -y",
        flap_closed is not None
        and flap_open_backward is not None
        and flap_open_backward[0][1] < flap_closed[0][1] - 0.06,
        details=f"closed={flap_closed}, inward={flap_open_backward}",
    )

    slide_rest = ctx.part_world_position(security_panel)
    with ctx.pose({security_slide: 0.055}):
        slide_locked = ctx.part_world_position(security_panel)
        ctx.expect_overlap(
            security_panel,
            flap,
            axes="z",
            elem_a="lockout_blade",
            elem_b="flap_plate",
            min_overlap=0.10,
            name="lockout blade overlaps flap height when lowered",
        )
        ctx.expect_overlap(
            security_panel,
            flap,
            axes="x",
            elem_a="lockout_blade",
            elem_b="flap_plate",
            min_overlap=0.001,
            name="lockout blade intrudes slightly across flap edge",
        )
        ctx.expect_gap(
            security_panel,
            flap,
            axis="y",
            positive_elem="lockout_blade",
            negative_elem="flap_plate",
            min_gap=0.020,
            max_gap=0.040,
            name="lockout blade stays in front of closed flap",
        )

    ctx.check(
        "security panel slides downward in side guides",
        slide_rest is not None
        and slide_locked is not None
        and slide_locked[2] < slide_rest[2] - 0.045
        and abs(slide_locked[0] - slide_rest[0]) < 0.001
        and abs(slide_locked[1] - slide_rest[1]) < 0.001,
        details=f"rest={slide_rest}, locked={slide_locked}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
