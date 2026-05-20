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
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _circle_profile(radius: float, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _fork_part_geometry():
    crown_width = 0.240

    steerer = CylinderGeometry(radius=0.028, height=0.300).translate(0.0, 0.0, 0.150)
    lower_bearing = CylinderGeometry(radius=0.036, height=0.030).translate(0.0, 0.0, 0.012)
    crown_yoke = BoxGeometry((0.105, crown_width, 0.042)).translate(0.010, 0.0, -0.010)
    lower_yoke = BoxGeometry((0.085, 0.160, 0.040)).translate(0.028, 0.0, -0.045)

    blade_profile = rounded_rect_profile(0.030, 0.056, radius=0.010, corner_segments=8)
    left_blade = sweep_profile_along_spline(
        [
            (0.012, 0.068, -0.020),
            (0.026, 0.084, -0.110),
            (0.043, 0.102, -0.250),
            (0.058, 0.118, -0.420),
        ],
        profile=blade_profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    right_blade = sweep_profile_along_spline(
        [
            (0.012, -0.068, -0.020),
            (0.026, -0.084, -0.110),
            (0.043, -0.102, -0.250),
            (0.058, -0.118, -0.420),
        ],
        profile=blade_profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )

    return {
        "steerer": steerer,
        "lower_bearing": lower_bearing,
        "crown_yoke": crown_yoke,
        "lower_yoke": lower_yoke,
        "left_blade": left_blade,
        "right_blade": right_blade,
        "left_dropout": BoxGeometry((0.042, 0.020, 0.048)).translate(0.060, 0.118, -0.430),
        "right_dropout": BoxGeometry((0.042, 0.020, 0.048)).translate(0.060, -0.118, -0.430),
        "left_axle_boss": CylinderGeometry(radius=0.013, height=0.022)
        .rotate_x(math.pi / 2.0)
        .translate(0.072, 0.118, -0.430),
        "right_axle_boss": CylinderGeometry(radius=0.013, height=0.022)
        .rotate_x(math.pi / 2.0)
        .translate(0.072, -0.118, -0.430),
    }


def _build_stem_geometry():
    steer_hole = _circle_profile(0.0284, segments=56)
    steer_outer = rounded_rect_profile(0.094, 0.084, radius=0.017, corner_segments=10)
    steer_clamp = ExtrudeWithHolesGeometry(steer_outer, [steer_hole], 0.060, center=True)

    upper_web = BoxGeometry((0.056, 0.028, 0.010)).translate(0.073, 0.0, 0.031)
    lower_web = BoxGeometry((0.056, 0.028, 0.010)).translate(0.073, 0.0, -0.031)

    # Give the handlebar clamp a small assembly clearance around the 31.8 mm
    # center section so the bar reads as seated in the clamp without
    # interpenetrating the stem body.
    bar_hole = _circle_profile(0.0167, segments=56)
    bar_outer = rounded_rect_profile(0.054, 0.060, radius=0.013, corner_segments=10)
    bar_clamp = (
        ExtrudeWithHolesGeometry(bar_outer, [bar_hole], 0.036, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.088, 0.0, 0.0)
    )

    # Add split clamp lands that actually seat the round center section of the
    # handlebar. They bridge into the upper and lower webs and provide a real
    # support path without closing the clamp hole into a solid overlap.
    upper_cradle = BoxGeometry((0.014, 0.022, 0.020)).translate(0.088, 0.0, 0.0258)
    lower_cradle = BoxGeometry((0.014, 0.022, 0.020)).translate(0.088, 0.0, -0.0258)
    upper_faceplate = BoxGeometry((0.014, 0.032, 0.010)).translate(0.117, 0.0, 0.026)
    lower_faceplate = BoxGeometry((0.014, 0.032, 0.010)).translate(0.117, 0.0, -0.026)

    stem_geom = steer_clamp
    stem_geom.merge(upper_web)
    stem_geom.merge(lower_web)
    stem_geom.merge(bar_clamp)
    stem_geom.merge(upper_cradle)
    stem_geom.merge(lower_cradle)
    stem_geom.merge(upper_faceplate)
    stem_geom.merge(lower_faceplate)
    return stem_geom


def _build_handlebar_mesh():
    left_wing = tube_from_spline_points(
        [
            (0.000, 0.040, 0.000),
            (0.000, 0.060, 0.000),
            (-0.010, 0.120, 0.006),
            (-0.022, 0.195, 0.014),
            (-0.034, 0.285, 0.018),
            (-0.040, 0.360, 0.020),
        ],
        radius=0.0158,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    right_wing = tube_from_spline_points(
        [
            (0.000, -0.040, 0.000),
            (0.000, -0.060, 0.000),
            (-0.010, -0.120, 0.006),
            (-0.022, -0.195, 0.014),
            (-0.034, -0.285, 0.018),
            (-0.040, -0.360, 0.020),
        ],
        radius=0.0158,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    left_wing.merge(right_wing)
    return left_wing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cargo_bike_wide_fork")

    powder_black = model.material("powder_black", rgba=(0.15, 0.15, 0.16, 1.0))
    satin_black = model.material("satin_black", rgba=(0.09, 0.09, 0.10, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    fork = model.part("fork")
    fork_geom = _fork_part_geometry()
    fork.visual(
        Cylinder(radius=0.028, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=powder_black,
        name="steerer",
    )
    fork.visual(
        Cylinder(radius=0.036, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_alloy,
        name="lower_bearing",
    )
    fork.visual(
        Cylinder(radius=0.038, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=dark_alloy,
        name="upper_headset_cap",
    )
    fork.visual(
        Box((0.105, 0.240, 0.042)),
        origin=Origin(xyz=(0.010, 0.0, -0.010)),
        material=powder_black,
        name="crown_yoke",
    )
    fork.visual(
        Box((0.085, 0.160, 0.040)),
        origin=Origin(xyz=(0.028, 0.0, -0.045)),
        material=powder_black,
        name="lower_yoke",
    )
    fork.visual(_save_mesh("cargo_fork_left_blade", fork_geom["left_blade"]), material=powder_black, name="left_blade")
    fork.visual(_save_mesh("cargo_fork_right_blade", fork_geom["right_blade"]), material=powder_black, name="right_blade")
    fork.visual(
        Box((0.042, 0.020, 0.048)),
        origin=Origin(xyz=(0.060, 0.118, -0.430)),
        material=powder_black,
        name="left_dropout",
    )
    fork.visual(
        Box((0.042, 0.020, 0.048)),
        origin=Origin(xyz=(0.060, -0.118, -0.430)),
        material=powder_black,
        name="right_dropout",
    )
    fork.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.072, 0.118, -0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="left_axle_boss",
    )
    fork.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.072, -0.118, -0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="right_axle_boss",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.160, 0.280, 0.760)),
        mass=2.8,
        origin=Origin(xyz=(0.020, 0.0, -0.080)),
    )

    stem = model.part("stem")
    stem.visual(
        _save_mesh("cargo_stem_body", _build_stem_geometry()),
        material=dark_alloy,
        name="stem_body",
    )
    stem.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(xyz=(0.122, 0.0, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="faceplate_upper_bolt",
    )
    stem.visual(
        Cylinder(radius=0.004, length=0.060),
        origin=Origin(xyz=(0.122, 0.0, -0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="faceplate_lower_bolt",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.132, 0.094, 0.086)),
        mass=0.7,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.0158, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="bar_center_section",
    )
    handlebar.visual(
        _save_mesh("cargo_handlebar_main", _build_handlebar_mesh()),
        material=dark_alloy,
        name="bar_main",
    )
    handlebar.visual(
        Cylinder(radius=0.0178, length=0.125),
        origin=Origin(xyz=(-0.040, 0.335, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.0178, length=0.125),
        origin=Origin(xyz=(-0.040, -0.335, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.080, 0.780, 0.120)),
        mass=0.9,
        origin=Origin(xyz=(-0.018, 0.0, 0.012)),
    )

    model.articulation(
        "steerer_to_stem",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.2,
            lower=-0.75,
            upper=0.75,
        ),
    )

    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.088, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
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
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    steer_joint = object_model.get_articulation("steerer_to_stem")
    bar_joint = object_model.get_articulation("stem_to_handlebar")

    ctx.check(
        "steerer joint uses vertical steering axis",
        tuple(round(value, 6) for value in steer_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={steer_joint.axis}",
    )
    ctx.check(
        "handlebar joint rolls around the bar axis",
        tuple(round(value, 6) for value in bar_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={bar_joint.axis}",
    )

    def _center_of_elem(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    left_dropout_aabb = ctx.part_element_world_aabb(fork, elem="left_dropout")
    right_dropout_aabb = ctx.part_element_world_aabb(fork, elem="right_dropout")
    wide_clearance_ok = False
    clearance_details = f"left={left_dropout_aabb}, right={right_dropout_aabb}"
    if left_dropout_aabb is not None and right_dropout_aabb is not None:
        left_min_y = left_dropout_aabb[0][1]
        right_max_y = right_dropout_aabb[1][1]
        wide_clearance_ok = (left_min_y - right_max_y) >= 0.205
        clearance_details = f"inner_gap={left_min_y - right_max_y:.4f}, left={left_dropout_aabb}, right={right_dropout_aabb}"
    ctx.check(
        "fork dropout spacing clears a wide cargo tire",
        wide_clearance_ok,
        details=clearance_details,
    )

    bar_origin_rest = ctx.part_world_position(handlebar)
    with ctx.pose({steer_joint: 0.45}):
        bar_origin_steered = ctx.part_world_position(handlebar)
    ctx.check(
        "positive steering swings the stem and bar to the rider's left",
        bar_origin_rest is not None
        and bar_origin_steered is not None
        and bar_origin_steered[1] > bar_origin_rest[1] + 0.025,
        details=f"rest={bar_origin_rest}, steered={bar_origin_steered}",
    )

    left_grip_rest = _center_of_elem(handlebar, "left_grip")
    right_grip_rest = _center_of_elem(handlebar, "right_grip")
    with ctx.pose({bar_joint: 0.35}):
        left_grip_rolled = _center_of_elem(handlebar, "left_grip")
        right_grip_rolled = _center_of_elem(handlebar, "right_grip")
    ctx.check(
        "bar roll raises the grips and changes their fore-aft attitude",
        left_grip_rest is not None
        and right_grip_rest is not None
        and left_grip_rolled is not None
        and right_grip_rolled is not None
        and left_grip_rolled[2] > left_grip_rest[2] + 0.010
        and right_grip_rolled[2] > right_grip_rest[2] + 0.010
        and left_grip_rolled[0] > left_grip_rest[0] + 0.008
        and right_grip_rolled[0] > right_grip_rest[0] + 0.008,
        details=(
            f"left_rest={left_grip_rest}, left_rolled={left_grip_rolled}, "
            f"right_rest={right_grip_rest}, right_rolled={right_grip_rolled}"
        ),
    )

    ctx.expect_overlap(
        stem,
        fork,
        axes="xy",
        min_overlap=0.040,
        name="stem stays centered above the wide steerer",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="yz",
        min_overlap=0.030,
        name="handlebar clamp section stays seated in the stem clamp zone",
    )
    ctx.expect_contact(
        handlebar,
        stem,
        elem_a="bar_center_section",
        elem_b="stem_body",
        contact_tol=1e-6,
        name="stem clamp lands touch the handlebar center section",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
