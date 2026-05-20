from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scratch_mixer")

    housing_black = model.material("housing_black", rgba=(0.10, 0.11, 0.12, 1.0))
    panel_black = model.material("panel_black", rgba=(0.07, 0.08, 0.09, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.60, 0.63, 0.67, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.03, 0.03, 0.035, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.05, 0.05, 0.05, 1.0))
    label_orange = model.material("label_orange", rgba=(0.89, 0.46, 0.12, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.95, 0.54, 0.15, 1.0))

    housing_w = 0.66
    housing_d = 0.38
    housing_h = 0.078
    top_t = 0.006
    wall_t = 0.022
    top_z = housing_h - (top_t * 0.5)
    wall_h = housing_h - top_t

    platter_center = (-0.17, 0.03, housing_h)
    platter_radius = 0.152
    fader_origin = (0.12, -0.13, housing_h)
    knob_origin = (0.205, 0.085, housing_h)
    arm_origin = (0.01, 0.105, housing_h)

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((housing_w, housing_d, housing_h)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, housing_h * 0.5)),
    )

    panel_profile = rounded_rect_profile(housing_w, housing_d, 0.028, corner_segments=8)
    crossfader_slot = _offset_profile(
        rounded_rect_profile(0.17, 0.018, 0.009, corner_segments=8),
        dx=fader_origin[0],
        dy=fader_origin[1],
    )
    top_panel_geom = ExtrudeWithHolesGeometry(
        panel_profile,
        [crossfader_slot],
        top_t,
        center=True,
    )
    housing.visual(
        _mesh("scratch_mixer_top_panel", top_panel_geom),
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=panel_black,
        name="top_panel",
    )
    housing.visual(
        Box((housing_w, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, (housing_d * 0.5) - (wall_t * 0.5), wall_h * 0.5)),
        material=housing_black,
        name="rear_wall",
    )
    housing.visual(
        Box((housing_w, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -(housing_d * 0.5) + (wall_t * 0.5), wall_h * 0.5)),
        material=housing_black,
        name="front_wall",
    )
    housing.visual(
        Box((wall_t, housing_d - (2.0 * wall_t), wall_h)),
        origin=Origin(xyz=((housing_w * 0.5) - (wall_t * 0.5), 0.0, wall_h * 0.5)),
        material=housing_black,
        name="right_wall",
    )
    housing.visual(
        Box((wall_t, housing_d - (2.0 * wall_t), wall_h)),
        origin=Origin(xyz=(-(housing_w * 0.5) + (wall_t * 0.5), 0.0, wall_h * 0.5)),
        material=housing_black,
        name="left_wall",
    )
    housing.visual(
        Box((housing_w - 0.040, housing_d - 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim_gray,
        name="bottom_plate",
    )
    housing.visual(
        Box((0.20, 0.090, 0.008)),
        origin=Origin(xyz=(0.185, 0.085, housing_h + 0.004)),
        material=housing_black,
        name="control_bay",
    )
    housing.visual(
        Cylinder(radius=0.166, length=0.004),
        origin=Origin(xyz=(platter_center[0], platter_center[1], housing_h + 0.002)),
        material=trim_gray,
        name="platter_seat",
    )
    housing.visual(
        Box((0.13, 0.028, 0.003)),
        origin=Origin(xyz=(fader_origin[0], fader_origin[1], housing_h + 0.0015)),
        material=trim_gray,
        name="crossfader_trim",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(arm_origin[0], arm_origin[1], housing_h + 0.001)),
        material=trim_gray,
        name="tonearm_rest_pod",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.015, -0.095, housing_h + 0.002)),
        material=accent_orange,
        name="start_button",
    )
    for index, x_sign in enumerate((-1.0, 1.0)):
        for row, y_sign in enumerate((-1.0, 1.0)):
            housing.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(
                    xyz=(
                        x_sign * ((housing_w * 0.5) - 0.050),
                        y_sign * ((housing_d * 0.5) - 0.050),
                        0.004,
                    )
                ),
                material=rubber_dark,
                name=f"foot_{index}_{row}",
            )

    platter = model.part("platter")
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=platter_radius, length=0.022),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )
    platter.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brushed_steel,
        name="bearing_collar",
    )
    platter.visual(
        Cylinder(radius=platter_radius, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=platter_radius * 0.94, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=rubber_dark,
        name="slipmat",
    )
    platter.visual(
        Cylinder(radius=platter_radius * 0.91, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=vinyl_black,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.054, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, 0.0202)),
        material=label_orange,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.0022, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=brushed_steel,
        name="center_spindle",
    )

    crossfader = model.part("crossfader")
    crossfader.inertial = Inertial.from_geometry(
        Box((0.034, 0.018, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )
    crossfader.visual(
        Box((0.034, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=trim_gray,
        name="fader_sled",
    )
    crossfader.visual(
        Box((0.010, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brushed_steel,
        name="fader_stem",
    )
    crossfader.visual(
        Box((0.024, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=accent_orange,
        name="fader_cap",
    )

    filter_knob = model.part("filter_knob")
    filter_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.028),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )
    filter_knob.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim_gray,
        name="knob_skirt",
    )
    filter_knob.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=panel_black,
        name="knob_body",
    )
    filter_knob.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=panel_black,
        name="knob_cap",
    )
    filter_knob.visual(
        Box((0.004, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0115, 0.0245)),
        material=accent_orange,
        name="indicator",
    )

    stylus_arm = model.part("stylus_arm")
    stylus_arm.inertial = Inertial.from_geometry(
        Box((0.18, 0.24, 0.035)),
        mass=0.18,
        origin=Origin(xyz=(0.05, -0.08, 0.018)),
    )
    arm_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0235),
            (0.018, -0.018, 0.0255),
            (0.050, -0.060, 0.0255),
            (0.088, -0.122, 0.0245),
            (0.116, -0.182, 0.0240),
        ],
        radius=0.0055,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    arm_geom.merge(
        CylinderGeometry(radius=0.012, height=0.032)
        .rotate_y(math.pi / 2.0)
        .translate(-0.028, 0.012, 0.023)
    )
    stylus_arm.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_gray,
        name="base_collar",
    )
    stylus_arm.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=brushed_steel,
        name="pivot_turret",
    )
    stylus_arm.visual(
        _mesh("scratch_mixer_tonearm_tube", arm_geom),
        material=brushed_steel,
        name="arm_tube",
    )
    stylus_arm.visual(
        Box((0.030, 0.014, 0.006)),
        origin=Origin(xyz=(0.118, -0.188, 0.0245)),
        material=panel_black,
        name="headshell",
    )
    stylus_arm.visual(
        Box((0.004, 0.006, 0.004)),
        origin=Origin(xyz=(0.129, -0.198, 0.0240)),
        material=accent_orange,
        name="stylus_tip",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=platter,
        origin=Origin(xyz=platter_center),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )
    model.articulation(
        "crossfader_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=fader_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.35,
            lower=-0.055,
            upper=0.055,
        ),
    )
    model.articulation(
        "filter_sweep",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=filter_knob,
        origin=Origin(xyz=knob_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "stylus_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=stylus_arm,
        origin=Origin(xyz=arm_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=1.2,
            lower=-1.75,
            upper=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    platter = object_model.get_part("platter")
    crossfader = object_model.get_part("crossfader")
    filter_knob = object_model.get_part("filter_knob")
    stylus_arm = object_model.get_part("stylus_arm")

    platter_spin = object_model.get_articulation("platter_spin")
    crossfader_slide = object_model.get_articulation("crossfader_slide")
    filter_sweep = object_model.get_articulation("filter_sweep")
    stylus_pivot = object_model.get_articulation("stylus_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(platter, housing, elem_a="bearing_collar", elem_b="platter_seat")
    ctx.expect_contact(crossfader, housing, elem_a="fader_sled", elem_b="crossfader_trim")
    ctx.expect_contact(filter_knob, housing, elem_a="knob_skirt", elem_b="control_bay")
    ctx.expect_contact(stylus_arm, housing, elem_a="base_collar", elem_b="tonearm_rest_pod")

    ctx.expect_overlap(platter, housing, axes="xy", min_overlap=0.26, elem_a="record_disc", elem_b="top_panel")
    ctx.expect_within(filter_knob, housing, axes="xy", margin=0.0)
    ctx.expect_within(crossfader, housing, axes="xy", margin=0.0)

    ctx.check(
        "platter spin axis",
        tuple(platter_spin.axis) == (0.0, 0.0, 1.0),
        f"expected vertical platter axle, got {platter_spin.axis}",
    )
    ctx.check(
        "crossfader slide axis",
        tuple(crossfader_slide.axis) == (1.0, 0.0, 0.0),
        f"expected lateral crossfader travel, got {crossfader_slide.axis}",
    )
    ctx.check(
        "filter knob axis",
        tuple(filter_sweep.axis) == (0.0, 0.0, 1.0),
        f"expected vertical filter knob axis, got {filter_sweep.axis}",
    )
    ctx.check(
        "stylus arm hinge axis",
        tuple(stylus_pivot.axis) == (0.0, 0.0, 1.0),
        f"expected vertical stylus pivot, got {stylus_pivot.axis}",
    )

    fader_center = ctx.part_world_position(crossfader)
    assert fader_center is not None
    with ctx.pose({crossfader_slide: 0.055}):
        fader_right = ctx.part_world_position(crossfader)
        assert fader_right is not None
        ctx.check(
            "crossfader reaches right",
            fader_right[0] > fader_center[0] + 0.050 and abs(fader_right[1] - fader_center[1]) < 1e-6,
            f"rest={fader_center}, right={fader_right}",
        )
        ctx.expect_contact(crossfader, housing, elem_a="fader_sled", elem_b="crossfader_trim")

    with ctx.pose({crossfader_slide: -0.055}):
        fader_left = ctx.part_world_position(crossfader)
        assert fader_left is not None
        ctx.check(
            "crossfader reaches left",
            fader_left[0] < fader_center[0] - 0.050 and abs(fader_left[1] - fader_center[1]) < 1e-6,
            f"rest={fader_center}, left={fader_left}",
        )
        ctx.expect_contact(crossfader, housing, elem_a="fader_sled", elem_b="crossfader_trim")

    knob_pos = ctx.part_world_position(filter_knob)
    knob_indicator_rest = _aabb_center(ctx.part_element_world_aabb(filter_knob, elem="indicator"))
    assert knob_pos is not None and knob_indicator_rest is not None
    with ctx.pose({filter_sweep: 1.5}):
        knob_indicator_turn = _aabb_center(ctx.part_element_world_aabb(filter_knob, elem="indicator"))
        assert knob_indicator_turn is not None
        radial_rest = math.hypot(knob_indicator_rest[0] - knob_pos[0], knob_indicator_rest[1] - knob_pos[1])
        radial_turn = math.hypot(knob_indicator_turn[0] - knob_pos[0], knob_indicator_turn[1] - knob_pos[1])
        ctx.check(
            "filter knob indicator rotates",
            abs(radial_rest - radial_turn) < 0.002
            and abs(knob_indicator_turn[0] - knob_indicator_rest[0]) > 0.008
            and abs(knob_indicator_turn[1] - knob_indicator_rest[1]) > 0.004,
            f"rest={knob_indicator_rest}, turned={knob_indicator_turn}",
        )
        ctx.expect_contact(filter_knob, housing, elem_a="knob_skirt", elem_b="control_bay")

    with ctx.pose({stylus_pivot: -1.55}):
        ctx.expect_overlap(
            stylus_arm,
            platter,
            axes="xy",
            min_overlap=0.004,
            elem_a="headshell",
            elem_b="record_disc",
        )
        ctx.expect_gap(
            stylus_arm,
            platter,
            axis="z",
            min_gap=0.0005,
            max_gap=0.006,
            positive_elem="stylus_tip",
            negative_elem="record_disc",
        )
        ctx.expect_contact(stylus_arm, housing, elem_a="base_collar", elem_b="tonearm_rest_pod")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
