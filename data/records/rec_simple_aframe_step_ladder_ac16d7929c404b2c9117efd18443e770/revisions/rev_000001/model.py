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


FRONT_OPEN_ANGLE = 0.33
REAR_OPEN_ANGLE = 0.33
SPREADER_OPEN_ANGLE = 0.95


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    size_x: float,
    size_y: float,
    material,
    name: str | None = None,
):
    part.visual(
        Box((size_x, size_y, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def _translate_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_tread_mesh(width: float, depth: float, thickness: float):
    outer = rounded_rect_profile(width, depth, radius=0.013, corner_segments=8)
    slot = rounded_rect_profile(width * 0.20, depth * 0.22, radius=0.006, corner_segments=6)
    holes = [
        _translate_profile(slot, -width * 0.22, 0.0),
        _translate_profile(slot, 0.0, 0.0),
        _translate_profile(slot, width * 0.22, 0.0),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            holes,
            height=thickness,
            center=True,
        ),
        "service_ladder_tread",
    )


def _add_front_frame(
    model: ArticulatedObject,
    material_rail,
    material_step,
    material_hardware,
):
    front = model.part("front_frame")
    front.inertial = Inertial.from_geometry(
        Box((0.48, 0.16, 1.46)),
        mass=9.5,
        origin=Origin(xyz=(0.0, -0.02, -0.72)),
    )

    rail_top_half = 0.148
    rail_bottom_half = 0.214
    top_z = -0.060
    bottom_z = -1.44
    left_top = (-rail_top_half, 0.0, top_z)
    left_bottom = (-rail_bottom_half, 0.0, bottom_z)
    right_top = (rail_top_half, 0.0, top_z)
    right_bottom = (rail_bottom_half, 0.0, bottom_z)

    _add_box_member(
        front,
        left_top,
        left_bottom,
        size_x=0.046,
        size_y=0.024,
        material=material_rail,
        name="left_rail",
    )
    _add_box_member(
        front,
        right_top,
        right_bottom,
        size_x=0.046,
        size_y=0.024,
        material=material_rail,
        name="right_rail",
    )

    front.visual(
        Box((0.318, 0.050, 0.032)),
        origin=Origin(xyz=(0.0, -0.002, -0.050)),
        material=material_hardware,
        name="top_bridge",
    )
    front.visual(
        Box((0.270, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, 0.010, -0.100)),
        material=material_hardware,
        name="top_backbone",
    )

    tread_mesh = _build_tread_mesh(width=0.300, depth=0.086, thickness=0.020)
    tread_zs = (-0.285, -0.575, -0.865, -1.155)
    tread_widths = (0.308, 0.336, 0.364, 0.392)
    for index, (z, width) in enumerate(zip(tread_zs, tread_widths), start=1):
        front.visual(
            tread_mesh,
            origin=Origin(xyz=(0.0, -0.038, z), rpy=(FRONT_OPEN_ANGLE, 0.0, 0.0)),
            material=material_step,
            name=f"tread_{index}",
        )
        front.visual(
            Box((width, 0.012, 0.040)),
            origin=Origin(xyz=(0.0, -0.006, z - 0.012), rpy=(FRONT_OPEN_ANGLE, 0.0, 0.0)),
            material=material_hardware,
            name=f"tread_support_{index}",
        )

    front.visual(
        Box((0.024, 0.040, 0.028)),
        origin=Origin(xyz=(-0.152, 0.015, -0.586)),
        material=material_hardware,
        name="left_spreader_bracket",
    )
    front.visual(
        Box((0.024, 0.040, 0.028)),
        origin=Origin(xyz=(0.152, 0.015, -0.586)),
        material=material_hardware,
        name="right_spreader_bracket",
    )
    front.visual(
        Box((0.022, 0.020, 0.052)),
        origin=Origin(xyz=(-0.165, 0.000, -0.600)),
        material=material_hardware,
        name="left_spreader_standoff",
    )
    front.visual(
        Box((0.022, 0.020, 0.052)),
        origin=Origin(xyz=(0.165, 0.000, -0.600)),
        material=material_hardware,
        name="right_spreader_standoff",
    )

    front.visual(
        Box((0.060, 0.038, 0.022)),
        origin=Origin(xyz=(-0.150, 0.000, -0.041)),
        material=material_hardware,
        name="left_hinge_block",
    )
    front.visual(
        Box((0.060, 0.038, 0.022)),
        origin=Origin(xyz=(0.150, 0.000, -0.041)),
        material=material_hardware,
        name="right_hinge_block",
    )
    return front


def _add_rear_frame(
    model: ArticulatedObject,
    material_rail,
    material_hardware,
):
    rear = model.part("rear_frame")
    rear.inertial = Inertial.from_geometry(
        Box((0.44, 0.14, 1.46)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.01, -0.72)),
    )

    rail_top_half = 0.146
    rail_bottom_half = 0.192
    top_z = -0.060
    bottom_z = -1.43
    left_top = (-rail_top_half, 0.0, top_z)
    left_bottom = (-rail_bottom_half, 0.0, bottom_z)
    right_top = (rail_top_half, 0.0, top_z)
    right_bottom = (rail_bottom_half, 0.0, bottom_z)

    _add_box_member(
        rear,
        left_top,
        left_bottom,
        size_x=0.040,
        size_y=0.022,
        material=material_rail,
        name="left_rail",
    )
    _add_box_member(
        rear,
        right_top,
        right_bottom,
        size_x=0.040,
        size_y=0.022,
        material=material_rail,
        name="right_rail",
    )

    rear.visual(
        Box((0.306, 0.048, 0.030)),
        origin=Origin(xyz=(0.0, 0.000, -0.045)),
        material=material_hardware,
        name="top_bridge",
    )

    brace_zs = (-0.420, -0.805, -1.180)
    brace_widths = (0.296, 0.326, 0.354)
    for index, (z, width) in enumerate(zip(brace_zs, brace_widths), start=1):
        rear.visual(
            Box((width, 0.020, 0.042)),
            origin=Origin(xyz=(0.0, 0.004, z), rpy=(-REAR_OPEN_ANGLE, 0.0, 0.0)),
            material=material_hardware,
            name=f"rear_cross_brace_{index}",
        )

    rear.visual(
        Box((0.024, 0.040, 0.028)),
        origin=Origin(xyz=(-0.152, -0.015, -0.586)),
        material=material_hardware,
        name="left_spreader_bracket",
    )
    rear.visual(
        Box((0.024, 0.040, 0.028)),
        origin=Origin(xyz=(0.152, -0.015, -0.586)),
        material=material_hardware,
        name="right_spreader_bracket",
    )
    rear.visual(
        Box((0.020, 0.020, 0.052)),
        origin=Origin(xyz=(-0.162, 0.000, -0.600)),
        material=material_hardware,
        name="left_spreader_standoff",
    )
    rear.visual(
        Box((0.020, 0.020, 0.052)),
        origin=Origin(xyz=(0.162, 0.000, -0.600)),
        material=material_hardware,
        name="right_spreader_standoff",
    )

    rear.visual(
        Box((0.056, 0.038, 0.022)),
        origin=Origin(xyz=(-0.148, 0.000, -0.041)),
        material=material_hardware,
        name="left_hinge_block",
    )
    rear.visual(
        Box((0.056, 0.038, 0.022)),
        origin=Origin(xyz=(0.148, 0.000, -0.041)),
        material=material_hardware,
        name="right_hinge_block",
    )
    return rear


def _add_spreader_part(
    model: ArticulatedObject,
    name: str,
    *,
    material_bar,
    material_hardware,
    clevis: bool,
):
    spreader = model.part(name)
    spreader.inertial = Inertial.from_geometry(
        Box((0.022, 0.024, 0.28)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.14)),
    )
    spreader.visual(
        Box((0.006, 0.016, 0.240)),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=material_bar,
        name="main_bar",
    )
    spreader.visual(
        Box((0.010, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=material_hardware,
        name="top_lug",
    )
    spreader.visual(
        Box((0.010, 0.018, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.240)),
        material=material_hardware,
        name="end_block",
    )
    if clevis:
        spreader.visual(
            Box((0.003, 0.016, 0.036)),
            origin=Origin(xyz=(-0.0045, 0.0, -0.258)),
            material=material_hardware,
            name="clevis_left",
        )
        spreader.visual(
            Box((0.003, 0.016, 0.036)),
            origin=Origin(xyz=(0.0045, 0.0, -0.258)),
            material=material_hardware,
            name="clevis_right",
        )
    else:
        spreader.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.258), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material_hardware,
            name="knuckle_barrel",
        )
    return spreader


def _add_foot_part(
    model: ArticulatedObject,
    name: str,
    *,
    material_rubber,
    material_hardware,
):
    foot = model.part(name)
    foot.inertial = Inertial.from_geometry(
        Box((0.078, 0.052, 0.052)),
        mass=0.2,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )
    foot.visual(
        Box((0.078, 0.052, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=material_rubber,
        name="rubber_shoe",
    )
    foot.visual(
        Box((0.050, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=material_hardware,
        name="retainer_sleeve",
    )
    foot.visual(
        Box((0.054, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.018, -0.004)),
        material=material_hardware,
        name="service_plate",
    )
    return foot


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_aframe_step_ladder")

    fiberglass_orange = model.material("fiberglass_orange", rgba=(0.88, 0.46, 0.13, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    zinc = model.material("zinc", rgba=(0.58, 0.60, 0.63, 1.0))
    charcoal = model.material("charcoal", rgba=(0.23, 0.24, 0.26, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    head = model.part("head_assembly")
    head.inertial = Inertial.from_geometry(
        Box((0.40, 0.22, 0.16)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 1.535)),
    )
    head.visual(
        Box((0.360, 0.250, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 1.535)),
        material=charcoal,
        name="top_cap",
    )
    head.visual(
        Box((0.290, 0.140, 0.020)),
        origin=Origin(xyz=(0.0, 0.000, 1.574)),
        material=charcoal,
        name="service_tray",
    )
    head.visual(
        Box((0.018, 0.090, 0.100)),
        origin=Origin(xyz=(-0.182, 0.000, 1.504)),
        material=zinc,
        name="left_hinge_cheek",
    )
    head.visual(
        Box((0.018, 0.090, 0.100)),
        origin=Origin(xyz=(0.182, 0.000, 1.504)),
        material=zinc,
        name="right_hinge_cheek",
    )
    head.visual(
        Box((0.320, 0.050, 0.032)),
        origin=Origin(xyz=(0.0, -0.117, 1.491)),
        material=zinc,
        name="front_pivot_housing",
    )
    head.visual(
        Box((0.320, 0.050, 0.032)),
        origin=Origin(xyz=(0.0, 0.117, 1.491)),
        material=zinc,
        name="rear_pivot_housing",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.332),
        origin=Origin(xyz=(0.0, -0.117, 1.502), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="front_pivot_shaft",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.332),
        origin=Origin(xyz=(0.0, 0.117, 1.502), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="rear_pivot_shaft",
    )

    front = _add_front_frame(model, fiberglass_orange, aluminum, zinc)
    rear = _add_rear_frame(model, fiberglass_orange, zinc)
    left_front_spreader = _add_spreader_part(
        model,
        "left_front_spreader",
        material_bar=aluminum,
        material_hardware=zinc,
        clevis=True,
    )
    right_front_spreader = _add_spreader_part(
        model,
        "right_front_spreader",
        material_bar=aluminum,
        material_hardware=zinc,
        clevis=True,
    )
    left_rear_spreader = _add_spreader_part(
        model,
        "left_rear_spreader",
        material_bar=aluminum,
        material_hardware=zinc,
        clevis=False,
    )
    right_rear_spreader = _add_spreader_part(
        model,
        "right_rear_spreader",
        material_bar=aluminum,
        material_hardware=zinc,
        clevis=False,
    )

    front_left_foot = _add_foot_part(
        model, "front_left_foot", material_rubber=rubber, material_hardware=zinc
    )
    front_right_foot = _add_foot_part(
        model, "front_right_foot", material_rubber=rubber, material_hardware=zinc
    )
    rear_left_foot = _add_foot_part(
        model, "rear_left_foot", material_rubber=rubber, material_hardware=zinc
    )
    rear_right_foot = _add_foot_part(
        model, "rear_right_foot", material_rubber=rubber, material_hardware=zinc
    )

    model.articulation(
        "head_to_front_frame",
        ArticulationType.REVOLUTE,
        parent=head,
        child=front,
        origin=Origin(xyz=(0.0, -0.117, 1.505)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=0.40),
    )
    model.articulation(
        "head_to_rear_frame",
        ArticulationType.REVOLUTE,
        parent=head,
        child=rear,
        origin=Origin(xyz=(0.0, 0.117, 1.505)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=0.40),
    )

    model.articulation(
        "front_left_spreader_fold",
        ArticulationType.REVOLUTE,
        parent=front,
        child=left_front_spreader,
        origin=Origin(xyz=(-0.145, 0.030, -0.600)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "front_right_spreader_fold",
        ArticulationType.REVOLUTE,
        parent=front,
        child=right_front_spreader,
        origin=Origin(xyz=(0.145, 0.030, -0.600)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "rear_left_spreader_fold",
        ArticulationType.REVOLUTE,
        parent=rear,
        child=left_rear_spreader,
        origin=Origin(xyz=(-0.145, -0.030, -0.600)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "rear_right_spreader_fold",
        ArticulationType.REVOLUTE,
        parent=rear,
        child=right_rear_spreader,
        origin=Origin(xyz=(0.145, -0.030, -0.600)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    model.articulation(
        "front_left_foot_mount",
        ArticulationType.FIXED,
        parent=front,
        child=front_left_foot,
        origin=Origin(xyz=(-0.214, 0.000, -1.457)),
    )
    model.articulation(
        "front_right_foot_mount",
        ArticulationType.FIXED,
        parent=front,
        child=front_right_foot,
        origin=Origin(xyz=(0.214, 0.000, -1.457)),
    )
    model.articulation(
        "rear_left_foot_mount",
        ArticulationType.FIXED,
        parent=rear,
        child=rear_left_foot,
        origin=Origin(xyz=(-0.192, 0.000, -1.446)),
    )
    model.articulation(
        "rear_right_foot_mount",
        ArticulationType.FIXED,
        parent=rear,
        child=rear_right_foot,
        origin=Origin(xyz=(0.192, 0.000, -1.446)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    head = object_model.get_part("head_assembly")
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    left_front_spreader = object_model.get_part("left_front_spreader")
    right_front_spreader = object_model.get_part("right_front_spreader")
    left_rear_spreader = object_model.get_part("left_rear_spreader")
    right_rear_spreader = object_model.get_part("right_rear_spreader")
    front_left_foot = object_model.get_part("front_left_foot")
    front_right_foot = object_model.get_part("front_right_foot")
    rear_left_foot = object_model.get_part("rear_left_foot")
    rear_right_foot = object_model.get_part("rear_right_foot")

    head_to_front = object_model.get_articulation("head_to_front_frame")
    head_to_rear = object_model.get_articulation("head_to_rear_frame")
    front_left_spreader_fold = object_model.get_articulation("front_left_spreader_fold")
    front_right_spreader_fold = object_model.get_articulation("front_right_spreader_fold")
    rear_left_spreader_fold = object_model.get_articulation("rear_left_spreader_fold")
    rear_right_spreader_fold = object_model.get_articulation("rear_right_spreader_fold")

    ctx.expect_contact(head, front, contact_tol=0.018, name="front_frame_supported_by_head")
    ctx.expect_contact(head, rear, contact_tol=0.018, name="rear_frame_supported_by_head")
    ctx.expect_contact(front_left_foot, front, contact_tol=0.012, name="front_left_foot_mounted")
    ctx.expect_contact(front_right_foot, front, contact_tol=0.012, name="front_right_foot_mounted")
    ctx.expect_contact(rear_left_foot, rear, contact_tol=0.012, name="rear_left_foot_mounted")
    ctx.expect_contact(rear_right_foot, rear, contact_tol=0.012, name="rear_right_foot_mounted")
    ctx.expect_contact(
        left_front_spreader,
        front,
        contact_tol=0.010,
        name="left_front_spreader_bracketed",
    )
    ctx.expect_contact(
        right_front_spreader,
        front,
        contact_tol=0.010,
        name="right_front_spreader_bracketed",
    )
    ctx.expect_contact(
        left_rear_spreader,
        rear,
        contact_tol=0.010,
        name="left_rear_spreader_bracketed",
    )
    ctx.expect_contact(
        right_rear_spreader,
        rear,
        contact_tol=0.010,
        name="right_rear_spreader_bracketed",
    )

    ctx.expect_gap(
        rear,
        front,
        axis="y",
        min_gap=0.10,
        max_gap=0.18,
        name="closed_frames_nest_without_touching",
    )

    with ctx.pose(
        {
            head_to_front: FRONT_OPEN_ANGLE,
            head_to_rear: REAR_OPEN_ANGLE,
            front_left_spreader_fold: SPREADER_OPEN_ANGLE,
            front_right_spreader_fold: SPREADER_OPEN_ANGLE,
            rear_left_spreader_fold: SPREADER_OPEN_ANGLE,
            rear_right_spreader_fold: SPREADER_OPEN_ANGLE,
        }
    ):
        ctx.expect_origin_gap(
            left_rear_spreader,
            left_front_spreader,
            axis="y",
            min_gap=0.50,
            max_gap=0.62,
            name="left_spreader_pair_deployed",
        )
        ctx.expect_origin_gap(
            right_rear_spreader,
            right_front_spreader,
            axis="y",
            min_gap=0.50,
            max_gap=0.62,
            name="right_spreader_pair_deployed",
        )
        ctx.expect_origin_gap(
            rear_left_foot,
            front_left_foot,
            axis="y",
            min_gap=0.80,
            name="left_side_open_stance",
        )
        ctx.expect_origin_gap(
            rear_right_foot,
            front_right_foot,
            axis="y",
            min_gap=0.80,
            name="right_side_open_stance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
