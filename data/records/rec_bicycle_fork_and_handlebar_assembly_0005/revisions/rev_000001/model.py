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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(part, start, end, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_rpy_for_cylinder(start, end)),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _ring_shell(
    inner_radius: float,
    outer_radius: float,
    length: float,
    *,
    flare: float = 0.0,
    segments: int = 56,
):
    half = length * 0.5
    chamfer = min(0.0025, length * 0.25)
    outer = [
        (outer_radius + flare, -half),
        (outer_radius, -half + chamfer),
        (outer_radius, half - chamfer),
        (outer_radius + flare, half),
    ]
    inner = [
        (inner_radius, -half),
        (inner_radius, half),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _xy_section(
    center_x: float,
    center_y: float,
    z: float,
    width_x: float,
    width_y: float,
    corner: float,
):
    profile = rounded_rect_profile(width_x, width_y, corner, corner_segments=6)
    return [(center_x + x, center_y + y, z) for x, y in profile]


def _yz_section(
    x: float,
    center_y: float,
    center_z: float,
    width_y: float,
    height_z: float,
    corner: float,
):
    profile = rounded_rect_profile(width_y, height_z, corner, corner_segments=6)
    return [(x, center_y + y, center_z + z) for y, z in profile]


def _repaired_loft(sections):
    return repair_loft(section_loft(sections), repair="mesh")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_bike_fork_and_cockpit", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.08, 0.22, 0.42, 1.0))
    carbon = model.material("carbon", rgba=(0.12, 0.12, 0.14, 1.0))
    alloy = model.material("alloy", rgba=(0.73, 0.74, 0.77, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.29, 0.31, 0.34, 1.0))
    hood_rubber = model.material("hood_rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    bar_tape = model.material("bar_tape", rgba=(0.07, 0.07, 0.08, 1.0))

    steerer_radius = 0.0143
    bar_center_radius = 0.0159

    head_tube = model.part("head_tube")
    head_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.180),
        mass=1.2,
    )
    head_shell = LatheGeometry.from_shell_profiles(
        [
            (0.030, -0.086),
            (0.0285, -0.066),
            (0.0272, 0.000),
            (0.0285, 0.066),
            (0.030, 0.086),
        ],
        [
            (0.0225, -0.080),
            (0.0225, 0.080),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    head_tube.visual(
        _save_mesh("head_tube_shell.obj", head_shell),
        material=frame_paint,
        name="head_shell",
    )
    head_tube.visual(
        _save_mesh("upper_headset.obj", _ring_shell(0.0152, 0.0315, 0.010, flare=0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_alloy,
        name="upper_headset",
    )
    head_tube.visual(
        _save_mesh("lower_headset.obj", _ring_shell(0.0152, 0.0325, 0.010, flare=0.0015)),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=dark_alloy,
        name="lower_headset",
    )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.60)),
        mass=0.95,
        origin=Origin(xyz=(0.02, 0.0, -0.12)),
    )
    fork.visual(
        Cylinder(radius=steerer_radius, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_alloy,
        name="steerer",
    )
    fork.visual(
        _save_mesh("upper_spacer.obj", _ring_shell(steerer_radius, 0.0185, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_alloy,
        name="upper_spacer",
    )
    fork.visual(
        _save_mesh("crown_race.obj", _ring_shell(steerer_radius, 0.0195, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.096)),
        material=dark_alloy,
        name="crown_race",
    )
    crown_sections = [
        _xy_section(0.002, 0.000, -0.106, 0.036, 0.034, 0.006),
        _xy_section(0.012, 0.000, -0.124, 0.062, 0.108, 0.012),
        _xy_section(0.018, 0.000, -0.142, 0.050, 0.098, 0.010),
    ]
    fork.visual(
        _save_mesh("fork_crown.obj", _repaired_loft(crown_sections)),
        material=carbon,
        name="crown",
    )
    left_blade_sections = [
        _xy_section(0.016, -0.049, -0.124, 0.027, 0.014, 0.004),
        _xy_section(0.024, -0.050, -0.190, 0.025, 0.013, 0.004),
        _xy_section(0.036, -0.051, -0.282, 0.021, 0.011, 0.003),
        _xy_section(0.048, -0.052, -0.382, 0.017, 0.0095, 0.003),
        _xy_section(0.055, -0.051, -0.432, 0.015, 0.009, 0.0025),
    ]
    right_blade_sections = [
        _xy_section(0.016, 0.049, -0.124, 0.027, 0.014, 0.004),
        _xy_section(0.024, 0.050, -0.190, 0.025, 0.013, 0.004),
        _xy_section(0.036, 0.051, -0.282, 0.021, 0.011, 0.003),
        _xy_section(0.048, 0.052, -0.382, 0.017, 0.0095, 0.003),
        _xy_section(0.055, 0.051, -0.432, 0.015, 0.009, 0.0025),
    ]
    fork.visual(
        _save_mesh("left_blade.obj", _repaired_loft(left_blade_sections)),
        material=carbon,
        name="left_blade",
    )
    fork.visual(
        _save_mesh("right_blade.obj", _repaired_loft(right_blade_sections)),
        material=carbon,
        name="right_blade",
    )
    fork.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(0.055, -0.051, -0.4365)),
        material=dark_alloy,
        name="left_dropout",
    )
    fork.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(0.055, 0.051, -0.4365)),
        material=dark_alloy,
        name="right_dropout",
    )

    stem = model.part("stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.145, 0.070, 0.060)),
        mass=0.34,
        origin=Origin(xyz=(0.062, 0.0, -0.003)),
    )
    stem.visual(
        Box((0.010, 0.048, 0.044)),
        origin=Origin(xyz=(-0.0193, 0.0, 0.0)),
        material=alloy,
        name="steerer_clamp",
    )
    stem.visual(
        Box((0.052, 0.018, 0.044)),
        origin=Origin(xyz=(0.0017, -0.02335, 0.0)),
        material=alloy,
        name="left_clamp_cheek",
    )
    stem.visual(
        Box((0.052, 0.018, 0.044)),
        origin=Origin(xyz=(0.0017, 0.02335, 0.0)),
        material=alloy,
        name="right_clamp_cheek",
    )
    stem.visual(
        Box((0.072, 0.034, 0.012)),
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
        material=alloy,
        name="stem_body",
    )
    stem.visual(
        Box((0.0062, 0.046, 0.034)),
        origin=Origin(xyz=(0.0910, 0.0, -0.006)),
        material=alloy,
        name="bar_clamp",
    )

    faceplate_part = model.part("faceplate")
    faceplate_part.inertial = Inertial.from_geometry(
        Box((0.020, 0.050, 0.040)),
        mass=0.06,
    )
    faceplate_part.visual(
        Box((0.008, 0.048, 0.036)),
        material=alloy,
        name="faceplate",
    )
    for y_sign in (-1.0, 1.0):
        for z_sign in (-1.0, 1.0):
            faceplate_part.visual(
                Cylinder(radius=0.003, length=0.0318),
                origin=Origin(
                    xyz=(-0.0199, y_sign * 0.018, z_sign * 0.018),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=dark_alloy,
            )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.24, 0.44, 0.24)),
        mass=0.30,
        origin=Origin(xyz=(0.045, 0.0, -0.080)),
    )
    bar_points = [
        (0.086, -0.190, -0.215),
        (0.090, -0.188, -0.178),
        (0.083, -0.182, -0.122),
        (0.060, -0.170, -0.055),
        (0.024, -0.112, -0.010),
        (0.000, -0.050, 0.000),
        (0.000, 0.000, 0.000),
        (0.000, 0.050, 0.000),
        (0.024, 0.112, -0.010),
        (0.060, 0.170, -0.055),
        (0.083, 0.182, -0.122),
        (0.090, 0.188, -0.178),
        (0.086, 0.190, -0.215),
    ]
    handlebar.visual(
        _save_mesh(
            "drop_handlebar.obj",
            tube_from_spline_points(
                bar_points,
                radius=0.0118,
                samples_per_segment=18,
                radial_segments=20,
            ),
        ),
        material=alloy,
        name="bar_shell",
    )
    handlebar.visual(
        Cylinder(radius=bar_center_radius, length=0.044),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="center_bulge",
    )
    _add_tube(
        handlebar,
        (0.069, -0.186, -0.102),
        (0.088, -0.189, -0.188),
        0.0140,
        bar_tape,
        name="left_tape",
    )
    _add_tube(
        handlebar,
        (0.069, 0.186, -0.102),
        (0.088, 0.189, -0.188),
        0.0140,
        bar_tape,
        name="right_tape",
    )
    handlebar.visual(
        Box((0.034, 0.018, 0.074)),
        origin=Origin(xyz=(0.062, -0.148, -0.012), rpy=(0.35, 0.10, 0.10)),
        material=hood_rubber,
        name="left_hood",
    )
    handlebar.visual(
        Box((0.034, 0.018, 0.074)),
        origin=Origin(xyz=(0.062, 0.148, -0.012), rpy=(-0.35, 0.10, -0.10)),
        material=hood_rubber,
        name="right_hood",
    )

    model.articulation(
        "steerer_rotation",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-1.0, upper=1.0),
    )
    model.articulation(
        "fork_to_stem",
        ArticulationType.FIXED,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.FIXED,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.110, 0.0, -0.006)),
    )
    model.articulation(
        "stem_to_faceplate",
        ArticulationType.FIXED,
        parent=stem,
        child=faceplate_part,
        origin=Origin(xyz=(0.1299, 0.0, -0.006)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    head_tube = object_model.get_part("head_tube")
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    faceplate_part = object_model.get_part("faceplate")
    steer = object_model.get_articulation("steerer_rotation")

    head_shell = head_tube.get_visual("head_shell")
    upper_headset = head_tube.get_visual("upper_headset")
    lower_headset = head_tube.get_visual("lower_headset")
    steerer = fork.get_visual("steerer")
    upper_spacer = fork.get_visual("upper_spacer")
    crown_race = fork.get_visual("crown_race")
    crown = fork.get_visual("crown")
    left_blade = fork.get_visual("left_blade")
    right_blade = fork.get_visual("right_blade")
    left_dropout = fork.get_visual("left_dropout")
    right_dropout = fork.get_visual("right_dropout")
    steerer_clamp = stem.get_visual("steerer_clamp")
    left_clamp_cheek = stem.get_visual("left_clamp_cheek")
    right_clamp_cheek = stem.get_visual("right_clamp_cheek")
    bar_clamp = stem.get_visual("bar_clamp")
    faceplate = faceplate_part.get_visual("faceplate")
    center_bulge = handlebar.get_visual("center_bulge")
    left_tape = handlebar.get_visual("left_tape")
    right_tape = handlebar.get_visual("right_tape")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    limits = steer.motion_limits
    ctx.check(
        "steering joint axis is vertical",
        tuple(steer.axis) == (0.0, 0.0, 1.0),
        f"axis was {steer.axis}",
    )
    ctx.check(
        "steering limits are realistic road-bike values",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -0.9
        and limits.upper >= 0.9,
        f"limits were {limits}",
    )

    ctx.expect_within(
        fork,
        head_tube,
        axes="xy",
        inner_elem=steerer,
        outer_elem=head_shell,
        name="steerer stays centered inside the head tube",
    )
    ctx.expect_contact(
        head_tube,
        fork,
        elem_a=upper_headset,
        elem_b=upper_spacer,
        name="upper headset contacts the fork spacer",
    )
    ctx.expect_contact(
        head_tube,
        fork,
        elem_a=lower_headset,
        elem_b=crown_race,
        name="lower headset contacts the crown race",
    )
    ctx.expect_gap(
        head_tube,
        fork,
        axis="y",
        min_gap=0.010,
        positive_elem=head_shell,
        negative_elem=left_blade,
        name="left blade stays outboard of the head tube",
    )
    ctx.expect_gap(
        fork,
        head_tube,
        axis="y",
        min_gap=0.010,
        positive_elem=right_blade,
        negative_elem=head_shell,
        name="right blade stays outboard of the head tube",
    )
    ctx.expect_gap(
        fork,
        stem,
        axis="y",
        max_gap=0.0002,
        max_penetration=0.0,
        positive_elem=steerer,
        negative_elem=left_clamp_cheek,
        name="left clamp cheek sits just outboard of the steerer",
    )
    ctx.expect_gap(
        stem,
        fork,
        axis="y",
        max_gap=0.0002,
        max_penetration=0.0,
        positive_elem=right_clamp_cheek,
        negative_elem=steerer,
        name="right clamp cheek sits just outboard of the steerer",
    )
    ctx.expect_overlap(
        stem,
        fork,
        axes="yz",
        elem_a=steerer_clamp,
        elem_b=steerer,
        min_overlap=0.025,
        name="stem clamp wraps around the steerer in projection",
    )
    ctx.expect_gap(
        faceplate_part,
        head_tube,
        axis="x",
        min_gap=0.095,
        positive_elem=faceplate,
        negative_elem=head_shell,
        name="stem projects ahead of the head tube",
    )
    ctx.expect_gap(
        handlebar,
        stem,
        axis="x",
        max_gap=0.002,
        max_penetration=1e-5,
        positive_elem=center_bulge,
        negative_elem=bar_clamp,
        name="bar center bears against the rear clamp cradle",
    )
    ctx.expect_gap(
        faceplate_part,
        handlebar,
        axis="x",
        max_gap=0.002,
        max_penetration=1e-5,
        positive_elem=faceplate,
        negative_elem=center_bulge,
        name="faceplate closes against the front of the bar center",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="yz",
        elem_a=center_bulge,
        elem_b=bar_clamp,
        min_overlap=0.030,
        name="handlebar center section aligns with the rear clamp cradle",
    )
    ctx.expect_overlap(
        handlebar,
        faceplate_part,
        axes="yz",
        elem_a=center_bulge,
        elem_b=faceplate,
        min_overlap=0.030,
        name="faceplate covers the bar center section",
    )
    ctx.expect_gap(
        handlebar,
        head_tube,
        axis="z",
        min_gap=0.015,
        positive_elem=center_bulge,
        negative_elem=head_shell,
        name="handlebar center sits above the head tube",
    )
    ctx.expect_gap(
        stem,
        handlebar,
        axis="z",
        min_gap=0.065,
        positive_elem=bar_clamp,
        negative_elem=left_tape,
        name="left drop hangs well below the stem clamp",
    )
    ctx.expect_gap(
        stem,
        handlebar,
        axis="z",
        min_gap=0.065,
        positive_elem=bar_clamp,
        negative_elem=right_tape,
        name="right drop hangs well below the stem clamp",
    )

    head_aabb = ctx.part_element_world_aabb(head_tube, elem=head_shell)
    handlebar_aabb = ctx.part_world_aabb(handlebar)
    crown_aabb = ctx.part_element_world_aabb(fork, elem=crown)
    left_dropout_aabb = ctx.part_element_world_aabb(fork, elem=left_dropout)
    right_dropout_aabb = ctx.part_element_world_aabb(fork, elem=right_dropout)
    faceplate_rest_aabb = ctx.part_element_world_aabb(faceplate_part, elem=faceplate)

    def _extent(aabb, axis: int) -> float:
        return aabb[1][axis] - aabb[0][axis]

    def _center(aabb, axis: int) -> float:
        return (aabb[0][axis] + aabb[1][axis]) * 0.5

    ctx.check(
        "head tube height is road-bike sized",
        head_aabb is not None and 0.160 <= _extent(head_aabb, 2) <= 0.180,
        f"head tube aabb was {head_aabb}",
    )
    ctx.check(
        "drop handlebar width is realistic",
        handlebar_aabb is not None and 0.380 <= _extent(handlebar_aabb, 1) <= 0.430,
        f"handlebar aabb was {handlebar_aabb}",
    )
    ctx.check(
        "fork offset gives the blades forward rake",
        crown_aabb is not None
        and left_dropout_aabb is not None
        and _center(left_dropout_aabb, 0) > _center(crown_aabb, 0) + 0.020,
        f"crown={crown_aabb}, dropout={left_dropout_aabb}",
    )
    ctx.check(
        "dropout spacing matches a 100 mm road front hub",
        left_dropout_aabb is not None
        and right_dropout_aabb is not None
        and 0.095 <= (_center(right_dropout_aabb, 1) - _center(left_dropout_aabb, 1)) <= 0.105,
        f"left={left_dropout_aabb}, right={right_dropout_aabb}",
    )

    test_angle = 0.75
    with ctx.pose({steer: test_angle}):
        ctx.expect_within(
            fork,
            head_tube,
            axes="xy",
            inner_elem=steerer,
            outer_elem=head_shell,
            name="steerer remains in the head tube at positive steering",
        )
        ctx.expect_contact(
            head_tube,
            fork,
            elem_a=lower_headset,
            elem_b=crown_race,
            name="lower headset stays seated at positive steering",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="positive_steering_no_overlap")
        ctx.fail_if_isolated_parts(name="positive_steering_no_floating")
        faceplate_pos_aabb = ctx.part_element_world_aabb(faceplate_part, elem=faceplate)
        ctx.check(
            "positive steering moves the stem toward +y",
            faceplate_rest_aabb is not None
            and faceplate_pos_aabb is not None
            and _center(faceplate_pos_aabb, 1) > _center(faceplate_rest_aabb, 1) + 0.060,
            f"rest={faceplate_rest_aabb}, positive={faceplate_pos_aabb}",
        )
    with ctx.pose({steer: -test_angle}):
        ctx.expect_within(
            fork,
            head_tube,
            axes="xy",
            inner_elem=steerer,
            outer_elem=head_shell,
            name="steerer remains in the head tube at negative steering",
        )
        ctx.expect_contact(
            head_tube,
            fork,
            elem_a=lower_headset,
            elem_b=crown_race,
            name="lower headset stays seated at negative steering",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="negative_steering_no_overlap")
        ctx.fail_if_isolated_parts(name="negative_steering_no_floating")
        faceplate_neg_aabb = ctx.part_element_world_aabb(faceplate_part, elem=faceplate)
        ctx.check(
            "negative steering moves the stem toward -y",
            faceplate_rest_aabb is not None
            and faceplate_neg_aabb is not None
            and _center(faceplate_neg_aabb, 1) < _center(faceplate_rest_aabb, 1) - 0.060,
            f"rest={faceplate_rest_aabb}, negative={faceplate_neg_aabb}",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({steer: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_lower_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_lower_limit_no_floating")
        with ctx.pose({steer: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_upper_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_upper_limit_no_floating")
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
