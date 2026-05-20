from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(profile: list[tuple[float, float]], dy: float) -> list[tuple[float, float]]:
    return [(x, y + dy) for x, y in profile]


def _vertical_extrusion(geometry):
    """Convert an XY-profile / Z-extrusion mesh into local XZ / Y thickness."""
    return geometry.rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_cat_flap")

    cream_plastic = model.material("warm_ivory_plastic", rgba=(0.86, 0.82, 0.72, 1.0))
    dark_rubber = model.material("dark_gray_rubber", rgba=(0.04, 0.045, 0.045, 1.0))
    smoke_clear = model.material("smoked_translucent_flap", rgba=(0.23, 0.32, 0.36, 0.42))
    graphite = model.material("graphite_hardware", rgba=(0.14, 0.14, 0.13, 1.0))
    red_lock = model.material("red_security_slider", rgba=(0.62, 0.06, 0.045, 1.0))
    screw_mat = model.material("brushed_screw_heads", rgba=(0.55, 0.55, 0.50, 1.0))

    opening_w = 0.300
    opening_h = 0.340
    frame_outer_w = 0.440
    frame_outer_h = 0.500
    frame_depth = 0.035
    frame_front_y = frame_depth / 2.0

    frame = model.part("frame")
    frame_bezel = BezelGeometry(
        (opening_w, opening_h),
        (frame_outer_w, frame_outer_h),
        frame_depth,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.035,
        outer_corner_radius=0.045,
    )
    frame.visual(
        mesh_from_geometry(frame_bezel, "rounded_surround_frame"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream_plastic,
        name="rounded_surround_frame",
    )

    # Screw caps are seated into the front trim.
    for idx, (x, z) in enumerate(((-0.175, 0.205), (0.175, 0.205), (-0.175, -0.205), (0.175, -0.205))):
        frame.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(x, frame_front_y + 0.002, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=screw_mat,
            name=f"screw_cap_{idx}",
        )

    # Top hinge shroud: the fixed pin is carried on a small molded shelf.
    hinge_axis_z = opening_h / 2.0 + 0.010
    hinge_axis_y = frame_front_y + 0.018
    frame.visual(
        Box((0.305, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, frame_front_y + 0.008, hinge_axis_z + 0.006)),
        material=cream_plastic,
        name="hinge_shelf",
    )
    frame.visual(
        Cylinder(radius=0.0065, length=0.300),
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="top_hinge_pin",
    )

    # Side guide rails for the sliding lockout panel.
    guide_center_x = opening_w / 2.0 + 0.037
    guide_y = frame_front_y + 0.010
    guide_z = -0.010
    guide_h = 0.355
    guide_depth = 0.017
    rail_w = 0.006
    rail_offset = 0.018
    for name, x in (("inner_guide_rail", guide_center_x - rail_offset), ("outer_guide_rail", guide_center_x + rail_offset)):
        frame.visual(
            Box((rail_w, guide_depth, guide_h)),
            origin=Origin(xyz=(x, guide_y, guide_z)),
            material=cream_plastic,
            name=name,
        )
    for name, z in (("upper_guide_stop", guide_z + guide_h / 2.0 + 0.006), ("lower_guide_stop", guide_z - guide_h / 2.0 - 0.006)):
        frame.visual(
            Box((0.060, guide_depth, 0.012)),
            origin=Origin(xyz=(guide_center_x, guide_y, z)),
            material=cream_plastic,
            name=name,
        )

    # A small printed lock mark on the fixed frame makes the slider's purpose clear.
    frame.visual(
        Box((0.022, 0.0025, 0.030)),
        origin=Origin(xyz=(guide_center_x, frame_front_y + 0.00125, -0.125)),
        material=graphite,
        name="lock_mark",
    )

    flap = model.part("flap")
    flap_w = 0.255
    flap_h = 0.300
    flap_center_z = -0.164
    flap_top_clearance = -0.014
    rim_outer = _shift_profile(rounded_rect_profile(flap_w, flap_h, 0.030, corner_segments=8), flap_center_z)
    rim_inner = _shift_profile(rounded_rect_profile(0.215, 0.250, 0.020, corner_segments=8), flap_center_z)
    flap_rim = _vertical_extrusion(
        ExtrudeWithHolesGeometry(rim_outer, [rim_inner], 0.014, cap=True, center=True)
    )
    flap.visual(
        mesh_from_geometry(flap_rim, "flap_rim"),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=dark_rubber,
        name="flap_rim",
    )
    clear_panel_profile = _shift_profile(rounded_rect_profile(0.220, 0.255, 0.020, corner_segments=8), flap_center_z)
    clear_panel = _vertical_extrusion(ExtrudeGeometry(clear_panel_profile, 0.006, cap=True, center=True))
    flap.visual(
        mesh_from_geometry(clear_panel, "smoked_flexible_panel"),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=smoke_clear,
        name="smoked_panel",
    )
    flap.visual(
        Box((0.210, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, flap_top_clearance - flap_h + 0.004)),
        material=dark_rubber,
        name="bottom_sweep",
    )
    flap.visual(
        Box((0.235, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.014, -0.010)),
        material=dark_rubber,
        name="hinge_leaf",
    )

    security_panel = model.part("security_panel")
    security_panel.visual(
        Box((0.030, 0.008, 0.170)),
        origin=Origin(),
        material=red_lock,
        name="sliding_lockout_panel",
    )
    security_panel.visual(
        Box((0.040, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, 0.050)),
        material=graphite,
        name="thumb_grip",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.15),
    )

    model.articulation(
        "frame_to_security_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=security_panel,
        origin=Origin(xyz=(guide_center_x, guide_y, 0.045)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=0.0, upper=0.110),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    security_panel = object_model.get_part("security_panel")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    lock_slide = object_model.get_articulation("frame_to_security_panel")

    ctx.expect_within(
        flap,
        frame,
        axes="x",
        inner_elem="flap_rim",
        outer_elem="rounded_surround_frame",
        margin=0.002,
        name="flap fits laterally inside the fixed frame",
    )
    ctx.expect_gap(
        security_panel,
        frame,
        axis="y",
        positive_elem="sliding_lockout_panel",
        negative_elem="rounded_surround_frame",
        min_gap=0.002,
        max_gap=0.030,
        name="lockout slider rides proud of the front frame face",
    )
    ctx.expect_overlap(
        security_panel,
        frame,
        axes="z",
        elem_a="sliding_lockout_panel",
        elem_b="inner_guide_rail",
        min_overlap=0.140,
        name="lockout panel remains captured in the vertical side guide",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_rim")
    with ctx.pose({flap_hinge: 1.05}):
        open_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_rim")
    ctx.check(
        "flap swings outward and upward about the top horizontal hinge",
        rest_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > rest_panel_aabb[1][1] + 0.12
        and open_panel_aabb[0][2] > rest_panel_aabb[0][2] + 0.09,
        details=f"rest={rest_panel_aabb}, open={open_panel_aabb}",
    )

    rest_lock_pos = ctx.part_world_position(security_panel)
    with ctx.pose({lock_slide: 0.100}):
        lowered_lock_pos = ctx.part_world_position(security_panel)
        ctx.expect_overlap(
            security_panel,
            frame,
            axes="z",
            elem_a="sliding_lockout_panel",
            elem_b="inner_guide_rail",
            min_overlap=0.140,
            name="lowered lockout panel stays in its guide",
        )
    ctx.check(
        "security panel translates downward along its side guides",
        rest_lock_pos is not None
        and lowered_lock_pos is not None
        and lowered_lock_pos[2] < rest_lock_pos[2] - 0.08,
        details=f"rest={rest_lock_pos}, lowered={lowered_lock_pos}",
    )

    return ctx.report()


object_model = build_object_model()
