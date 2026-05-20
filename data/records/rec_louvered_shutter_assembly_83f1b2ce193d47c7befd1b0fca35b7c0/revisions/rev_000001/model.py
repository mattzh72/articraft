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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="storm_shutter_panel")

    trim_white = model.material("trim_white", rgba=(0.90, 0.90, 0.87, 1.0))
    reveal_shadow = model.material("reveal_shadow", rgba=(0.11, 0.11, 0.13, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.31, 0.36, 0.35, 1.0))
    slat_paint = model.material("slat_paint", rgba=(0.77, 0.79, 0.78, 1.0))
    hardware = model.material("hardware", rgba=(0.56, 0.58, 0.60, 1.0))

    leaf_width = 0.72
    leaf_height = 1.34
    stile_width = 0.082
    rail_height = 0.082
    front_frame_depth = 0.026
    full_rail_depth = 0.040
    pin_radius = 0.003
    pin_length = 0.018
    slat_body_length = 0.542
    slat_chord = 0.100
    slat_thickness = 0.014
    slat_pitch = 0.122
    slat_count = 9
    slat_axis_y = -0.012
    linkage_bar_x = 0.646
    linkage_bar_y = 0.0285
    linkage_bar_width = 0.016
    linkage_bar_thickness = 0.007
    linkage_bar_length = 1.08
    linkage_slot_width = 0.010
    linkage_slot_height = 0.084
    opening_x_center = leaf_width * 0.5

    slat_mesh_geom = ExtrudeGeometry(
        rounded_rect_profile(slat_chord, slat_thickness, radius=slat_thickness * 0.45),
        slat_body_length,
        center=True,
    ).rotate_y(math.pi / 2.0)
    slat_mesh = mesh_from_geometry(slat_mesh_geom, "storm_shutter_louver_slat")

    slot_profiles = [
        _translate_profile(
            rounded_rect_profile(
                linkage_slot_width,
                linkage_slot_height,
                radius=linkage_slot_width * 0.45,
            ),
            dy=(-0.488 + index * slat_pitch),
        )
        for index in range(slat_count)
    ]
    linkage_bar_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            linkage_bar_width,
            linkage_bar_length,
            radius=linkage_bar_width * 0.48,
        ),
        slot_profiles,
        linkage_bar_thickness,
        center=True,
    ).rotate_x(-math.pi / 2.0)
    linkage_bar_mesh = mesh_from_geometry(linkage_bar_geom, "storm_shutter_linkage_bar")

    opening_frame = model.part("opening_frame")
    opening_frame.visual(
        Box((0.070, 0.090, 1.46)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=trim_white,
        name="hinge_jamb",
    )
    opening_frame.visual(
        Box((0.80, 0.018, 1.42)),
        origin=Origin(xyz=(0.33, -0.054, 0.0)),
        material=reveal_shadow,
        name="opening_reveal",
    )
    opening_frame.inertial = Inertial.from_geometry(
        Box((0.87, 0.10, 1.46)),
        mass=14.0,
        origin=Origin(xyz=(0.30, -0.005, 0.0)),
    )

    leaf_frame = model.part("leaf_frame")
    leaf_frame.visual(
        Box((stile_width, front_frame_depth, leaf_height)),
        origin=Origin(xyz=(stile_width * 0.5, 0.004, 0.0)),
        material=frame_paint,
        name="left_stile",
    )
    leaf_frame.visual(
        Box((stile_width, front_frame_depth, leaf_height)),
        origin=Origin(xyz=(leaf_width - stile_width * 0.5, 0.004, 0.0)),
        material=frame_paint,
        name="right_stile",
    )
    leaf_frame.visual(
        Box((leaf_width, full_rail_depth, rail_height)),
        origin=Origin(xyz=(leaf_width * 0.5, -0.002, leaf_height * 0.5 - rail_height * 0.5)),
        material=frame_paint,
        name="top_rail",
    )
    leaf_frame.visual(
        Box((leaf_width, full_rail_depth, rail_height)),
        origin=Origin(xyz=(leaf_width * 0.5, -0.002, -leaf_height * 0.5 + rail_height * 0.5)),
        material=frame_paint,
        name="bottom_rail",
    )
    leaf_frame.visual(
        Box((0.012, 0.007, 1.18)),
        origin=Origin(xyz=(0.0825, -0.0185, 0.0)),
        material=hardware,
        name="left_pivot_clip",
    )
    leaf_frame.visual(
        Box((0.012, 0.007, 1.18)),
        origin=Origin(xyz=(0.6375, -0.0185, 0.0)),
        material=hardware,
        name="right_pivot_clip",
    )
    leaf_frame.inertial = Inertial.from_geometry(
        Box((leaf_width, 0.042, leaf_height)),
        mass=10.5,
        origin=Origin(xyz=(leaf_width * 0.5, -0.001, 0.0)),
    )

    linkage_bar = model.part("linkage_bar")
    linkage_bar.visual(
        linkage_bar_mesh,
        material=hardware,
        name="tilt_bar",
    )
    linkage_bar.visual(
        Box((0.012, 0.002, 1.04)),
        origin=Origin(xyz=(0.0, -0.0105, 0.0)),
        material=hardware,
        name="stile_runner",
    )
    linkage_bar.visual(
        Box((0.010, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, -0.0065, 0.50)),
        material=hardware,
        name="upper_runner_bridge",
    )
    linkage_bar.visual(
        Box((0.010, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, -0.0065, -0.50)),
        material=hardware,
        name="lower_runner_bridge",
    )
    linkage_bar.inertial = Inertial.from_geometry(
        Box((0.020, 0.014, linkage_bar_length)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
    )

    slat_parts: list[str] = []
    slat_joint_names: list[str] = []
    lug_local_x = 0.242
    pin_center_x = 0.274
    for index in range(slat_count):
        z = -0.488 + index * slat_pitch
        slat_name = f"louver_{index}"
        slat_parts.append(slat_name)
        slat = model.part(slat_name)
        slat.visual(
            slat_mesh,
            origin=Origin(xyz=(0.0, 0.010, 0.0)),
            material=slat_paint,
            name="blade",
        )
        slat.visual(
            Cylinder(radius=pin_radius, length=pin_length),
            origin=Origin(xyz=(-pin_center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="left_pivot_pin",
        )
        slat.visual(
            Cylinder(radius=pin_radius, length=pin_length),
            origin=Origin(xyz=(pin_center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="right_pivot_pin",
        )
        slat.visual(
            Box((0.008, 0.028, 0.014)),
            origin=Origin(xyz=(lug_local_x, -0.010, 0.0)),
            material=hardware,
            name="coupling_strap",
        )
        slat.inertial = Inertial.from_geometry(
            Box((0.56, 0.024, 0.10)),
            mass=0.65,
            origin=Origin(xyz=(0.0, 0.006, 0.0)),
        )
        joint_name = f"leaf_to_{slat_name}"
        slat_joint_names.append(joint_name)
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=leaf_frame,
            child=slat,
            origin=Origin(xyz=(opening_x_center, slat_axis_y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.4,
                lower=math.radians(-55.0),
                upper=math.radians(35.0),
            ),
        )

    model.articulation(
        "opening_to_leaf",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child=leaf_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "leaf_to_linkage_bar",
        ArticulationType.PRISMATIC,
        parent=leaf_frame,
        child=linkage_bar,
        origin=Origin(xyz=(linkage_bar_x, linkage_bar_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.20,
            lower=-0.045,
            upper=0.045,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    opening_frame = object_model.get_part("opening_frame")
    leaf_frame = object_model.get_part("leaf_frame")
    linkage_bar = object_model.get_part("linkage_bar")
    opening_to_leaf = object_model.get_articulation("opening_to_leaf")
    leaf_to_linkage_bar = object_model.get_articulation("leaf_to_linkage_bar")
    louver_parts = [object_model.get_part(f"louver_{index}") for index in range(9)]
    louver_joints = [object_model.get_articulation(f"leaf_to_louver_{index}") for index in range(9)]
    middle_louver = object_model.get_part("louver_4")
    middle_louver_joint = object_model.get_articulation("leaf_to_louver_4")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.expect_contact(leaf_frame, opening_frame, elem_a="left_stile", elem_b="hinge_jamb")
    ctx.expect_within(linkage_bar, leaf_frame, axes="xz", margin=0.0)
    for louver in louver_parts:
        ctx.expect_contact(louver, leaf_frame)

    linkage_rest = ctx.part_world_position(linkage_bar)
    assert linkage_rest is not None
    with ctx.pose({leaf_to_linkage_bar: 0.040}):
        linkage_high = ctx.part_world_position(linkage_bar)
        assert linkage_high is not None
        assert linkage_high[2] > linkage_rest[2] + 0.035
        ctx.expect_within(linkage_bar, leaf_frame, axes="xz", margin=0.0)

    with ctx.pose({opening_to_leaf: math.radians(72.0)}):
        open_aabb = ctx.part_world_aabb(leaf_frame)
        assert open_aabb is not None
        assert open_aabb[1][1] > 0.55

    middle_rest_aabb = ctx.part_world_aabb(middle_louver)
    assert middle_rest_aabb is not None
    with ctx.pose({middle_louver_joint: math.radians(30.0)}):
        middle_tilted_aabb = ctx.part_world_aabb(middle_louver)
        assert middle_tilted_aabb is not None
        assert middle_tilted_aabb[1][1] > middle_rest_aabb[1][1] + 0.018
        ctx.expect_contact(middle_louver, leaf_frame, name="middle_louver_stays_seated_when_tilted")

    coordinated_pose = {joint: math.radians(28.0) for joint in louver_joints}
    coordinated_pose[leaf_to_linkage_bar] = 0.032
    with ctx.pose(coordinated_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_operated_louver_pose")
        ctx.expect_contact(middle_louver, leaf_frame, name="operated_middle_louver_stays_seated")
        ctx.expect_within(linkage_bar, leaf_frame, axes="xz", margin=0.0, name="operated_linkage_bar_stays_in_frame")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
