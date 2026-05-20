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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dashboard_glove_box")

    dash_dark = model.material("dash_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    tub_black = model.material("tub_black", rgba=(0.09, 0.09, 0.10, 1.0))
    door_softtouch = model.material("door_softtouch", rgba=(0.20, 0.21, 0.22, 1.0))
    knob_satin = model.material("knob_satin", rgba=(0.60, 0.61, 0.62, 1.0))
    stay_metal = model.material("stay_metal", rgba=(0.56, 0.57, 0.59, 1.0))

    fascia_outer_w = 0.470
    fascia_outer_h = 0.260
    opening_w = 0.340
    opening_h = 0.178
    frame_thickness = 0.004

    tub_outer_w = 0.350
    tub_outer_h = 0.190
    tub_depth = 0.220
    tub_wall = 0.003

    door_w = 0.324
    door_h = 0.172
    door_t = 0.016
    hinge_y = -0.006
    hinge_z = door_h * 0.5

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    fascia_outer_profile = rounded_rect_profile(
        fascia_outer_w,
        fascia_outer_h,
        radius=0.026,
        corner_segments=8,
    )
    fascia_opening_profile = rounded_rect_profile(
        opening_w,
        opening_h,
        radius=0.016,
        corner_segments=8,
    )
    fascia_geom = ExtrudeWithHolesGeometry(
        fascia_outer_profile,
        [fascia_opening_profile],
        height=frame_thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)

    housing = model.part("housing")
    housing.visual(
        _mesh("glove_box_fascia_frame", fascia_geom),
        material=dash_dark,
        name="fascia_frame",
    )
    housing.visual(
        Box((tub_outer_w, tub_depth, tub_wall)),
        origin=Origin(xyz=(0.0, -tub_depth * 0.5 - frame_thickness * 0.5, tub_outer_h * 0.5 - tub_wall * 0.5)),
        material=tub_black,
        name="tub_top",
    )
    housing.visual(
        Box((tub_outer_w, tub_depth, tub_wall)),
        origin=Origin(xyz=(0.0, -tub_depth * 0.5 - frame_thickness * 0.5, -tub_outer_h * 0.5 + tub_wall * 0.5)),
        material=tub_black,
        name="tub_bottom",
    )
    housing.visual(
        Box((tub_wall, tub_depth, tub_outer_h)),
        origin=Origin(xyz=(-tub_outer_w * 0.5 + tub_wall * 0.5, -tub_depth * 0.5 - frame_thickness * 0.5, 0.0)),
        material=tub_black,
        name="tub_left_wall",
    )
    housing.visual(
        Box((tub_wall, tub_depth, tub_outer_h)),
        origin=Origin(xyz=(tub_outer_w * 0.5 - tub_wall * 0.5, -tub_depth * 0.5 - frame_thickness * 0.5, 0.0)),
        material=tub_black,
        name="tub_right_wall",
    )
    housing.visual(
        Box((tub_outer_w, tub_wall, tub_outer_h)),
        origin=Origin(xyz=(0.0, -tub_depth - frame_thickness + tub_wall * 0.5, 0.0)),
        material=tub_black,
        name="tub_back",
    )
    housing.visual(
        Box((door_w - 0.018, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.011, hinge_z + 0.003)),
        material=dash_dark,
        name="header_capture",
    )
    housing.visual(
        Box((0.006, 0.018, 0.020)),
        origin=Origin(xyz=(-0.172, -0.055, 0.018)),
        material=stay_metal,
        name="left_stay_bracket",
    )
    housing.visual(
        Box((0.006, 0.018, 0.020)),
        origin=Origin(xyz=(0.172, -0.055, 0.018)),
        material=stay_metal,
        name="right_stay_bracket",
    )
    housing.inertial = Inertial.from_geometry(
        Box((fascia_outer_w, tub_depth + 0.030, fascia_outer_h)),
        mass=2.9,
        origin=Origin(xyz=(0.0, -0.105, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.0, 0.0, -door_h * 0.5)),
        material=door_softtouch,
        name="door_panel",
    )
    door.visual(
        Box((door_w - 0.024, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.004, -0.004)),
        material=door_softtouch,
        name="upper_clip",
    )
    door.visual(
        Box((door_w - 0.040, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, -0.003, -0.128)),
        material=door_softtouch,
        name="inner_rib",
    )
    door.visual(
        Box((0.040, 0.004, 0.022)),
        origin=Origin(xyz=(-0.122, -0.010, -0.048)),
        material=door_softtouch,
        name="left_stay_pad",
    )
    door.visual(
        Box((0.040, 0.004, 0.022)),
        origin=Origin(xyz=(0.122, -0.010, -0.048)),
        material=door_softtouch,
        name="right_stay_pad",
    )
    door.visual(
        Cylinder(radius=0.004, length=door_w - 0.040),
        origin=Origin(xyz=(0.0, 0.005, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stay_metal,
        name="hinge_rod_cover",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, -door_h * 0.5)),
    )

    knob = model.part("latch_knob")
    knob.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_satin,
        name="stem",
    )
    knob.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_satin,
        name="knob_body",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.014),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    left_stay = model.part("left_stay")
    left_stay.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stay_metal,
        name="pivot_eye",
    )
    left_stay.visual(
        Box((0.007, 0.042, 0.006)),
        origin=Origin(xyz=(0.0, 0.021, 0.003)),
        material=stay_metal,
        name="stay_arm",
    )
    left_stay.visual(
        Box((0.007, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.041, 0.008)),
        material=stay_metal,
        name="shoe",
    )
    left_stay.inertial = Inertial.from_geometry(
        Box((0.010, 0.050, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.024, 0.006)),
    )

    right_stay = model.part("right_stay")
    right_stay.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stay_metal,
        name="pivot_eye",
    )
    right_stay.visual(
        Box((0.007, 0.042, 0.006)),
        origin=Origin(xyz=(0.0, 0.021, 0.003)),
        material=stay_metal,
        name="stay_arm",
    )
    right_stay.visual(
        Box((0.007, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.041, 0.008)),
        material=stay_metal,
        name="shoe",
    )
    right_stay.inertial = Inertial.from_geometry(
        Box((0.010, 0.050, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.024, 0.006)),
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.18,
        ),
    )
    knob_hinge = model.articulation(
        "knob_rotation",
        ArticulationType.REVOLUTE,
        parent=door,
        child=knob,
        origin=Origin(xyz=(0.0, 0.008, -0.148)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-1.15,
            upper=1.15,
        ),
    )
    left_stay_pivot = model.articulation(
        "left_stay_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_stay,
        origin=Origin(xyz=(-0.166, -0.055, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=1.00,
        ),
    )
    right_stay_pivot = model.articulation(
        "right_stay_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_stay,
        origin=Origin(xyz=(0.166, -0.055, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=1.00,
        ),
    )

    model.meta["primary_articulations"] = (
        door_hinge.name,
        knob_hinge.name,
        left_stay_pivot.name,
        right_stay_pivot.name,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    knob = object_model.get_part("latch_knob")
    left_stay = object_model.get_part("left_stay")
    right_stay = object_model.get_part("right_stay")

    door_hinge = object_model.get_articulation("door_hinge")
    knob_rotation = object_model.get_articulation("knob_rotation")
    left_stay_pivot = object_model.get_articulation("left_stay_pivot")
    right_stay_pivot = object_model.get_articulation("right_stay_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    for part_name in ("housing", "door", "latch_knob", "left_stay", "right_stay"):
        ctx.check(f"part_present_{part_name}", object_model.get_part(part_name) is not None)

    ctx.check(
        "door_hinge_axis_is_horizontal",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "knob_axis_points_out_of_panel",
        tuple(knob_rotation.axis) == (0.0, 1.0, 0.0),
        details=f"knob axis was {knob_rotation.axis}",
    )
    ctx.check(
        "stays_share_door_axis_orientation",
        tuple(left_stay_pivot.axis) == (1.0, 0.0, 0.0) and tuple(right_stay_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"stay axes were {left_stay_pivot.axis} and {right_stay_pivot.axis}",
    )

    ctx.expect_contact(
        door,
        housing,
        elem_a="upper_clip",
        elem_b="header_capture",
        contact_tol=5e-4,
        name="door_is_captured_on_upper_hinge_line",
    )
    ctx.expect_contact(
        knob,
        door,
        elem_a="stem",
        elem_b="door_panel",
        contact_tol=5e-4,
        name="knob_is_mounted_on_door_face",
    )
    ctx.expect_contact(
        left_stay,
        housing,
        elem_a="pivot_eye",
        elem_b="left_stay_bracket",
        contact_tol=5e-4,
        name="left_stay_is_supported_on_side_bracket",
    )
    ctx.expect_contact(
        right_stay,
        housing,
        elem_a="pivot_eye",
        elem_b="right_stay_bracket",
        contact_tol=5e-4,
        name="right_stay_is_supported_on_side_bracket",
    )
    ctx.expect_origin_gap(
        door,
        knob,
        axis="z",
        min_gap=0.130,
        max_gap=0.165,
        name="knob_sits_near_lower_edge_of_door",
    )
    ctx.expect_origin_distance(
        left_stay,
        right_stay,
        axes="x",
        min_dist=0.320,
        max_dist=0.340,
        name="stays_span_both_sides_of_tub",
    )

    with ctx.pose(
        {
            door_hinge: 1.00,
            left_stay_pivot: 0.78,
            right_stay_pivot: 0.78,
            knob_rotation: 0.35,
        }
    ):
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            positive_elem="inner_rib",
            negative_elem="fascia_frame",
            min_gap=0.050,
            name="door_lower_region_swings_outward_when_opened",
        )
        ctx.expect_gap(
            door,
            left_stay,
            axis="y",
            positive_elem="door_panel",
            negative_elem="stay_arm",
            min_gap=0.0,
            name="left_stay_remains_behind_door_when_open",
        )
        ctx.expect_gap(
            door,
            right_stay,
            axis="y",
            positive_elem="door_panel",
            negative_elem="stay_arm",
            min_gap=0.0,
            name="right_stay_remains_behind_door_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
