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


def _circle_profile(
    radius: float,
    *,
    segments: int = 48,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments):
        angle = (2.0 * math.pi * index) / segments
        if clockwise:
            angle = -angle
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _rounded_plate_geometry(
    *,
    width: float,
    height: float,
    radius: float,
    thickness: float,
    z_center: float,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.translate(0.0, 0.0, z_center)
    return geom


def _dial_geometry():
    body = ExtrudeWithHolesGeometry(
        _circle_profile(0.038, segments=64),
        [_circle_profile(0.0165, segments=64, clockwise=True)],
        0.016,
        cap=True,
        center=True,
        closed=True,
    )
    body.translate(0.0, 0.0, 0.009)

    face = ExtrudeWithHolesGeometry(
        _circle_profile(0.033, segments=64),
        [_circle_profile(0.0185, segments=64, clockwise=True)],
        0.006,
        cap=True,
        center=True,
        closed=True,
    )
    face.translate(0.0, 0.0, 0.020)

    body.merge(face)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_wall_thermostat")

    plate_ivory = model.material("plate_ivory", rgba=(0.89, 0.86, 0.78, 1.0))
    housing_enamel = model.material("housing_enamel", rgba=(0.63, 0.65, 0.60, 1.0))
    service_panel = model.material("service_panel", rgba=(0.45, 0.48, 0.46, 1.0))
    adapter_metal = model.material("adapter_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    bolt_dark = model.material("bolt_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    dial_black = model.material("dial_black", rgba=(0.13, 0.12, 0.11, 1.0))
    pointer_cream = model.material("pointer_cream", rgba=(0.87, 0.83, 0.73, 1.0))
    shaft_metal = model.material("shaft_metal", rgba=(0.69, 0.69, 0.67, 1.0))
    retainer_brass = model.material("retainer_brass", rgba=(0.74, 0.63, 0.35, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_geometry(
            _rounded_plate_geometry(
                width=0.176,
                height=0.145,
                radius=0.018,
                thickness=0.004,
                z_center=0.002,
            ),
            "thermostat_wall_plate",
        ),
        material=plate_ivory,
        name="backplate",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.176, 0.145, 0.004)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(
            _rounded_plate_geometry(
                width=0.148,
                height=0.120,
                radius=0.013,
                thickness=0.004,
                z_center=0.036,
            ),
            "thermostat_front_fascia",
        ),
        material=housing_enamel,
        name="front_fascia",
    )
    housing.visual(
        Box((0.008, 0.104, 0.034)),
        origin=Origin(xyz=(-0.070, 0.0, 0.017)),
        material=housing_enamel,
        name="left_wall",
    )
    housing.visual(
        Box((0.008, 0.104, 0.034)),
        origin=Origin(xyz=(0.070, 0.0, 0.017)),
        material=housing_enamel,
        name="right_wall",
    )
    housing.visual(
        Box((0.132, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.056, 0.017)),
        material=housing_enamel,
        name="top_wall",
    )
    housing.visual(
        Box((0.132, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, -0.056, 0.017)),
        material=housing_enamel,
        name="bottom_wall",
    )
    housing.visual(
        Box((0.012, 0.028, 0.014)),
        origin=Origin(xyz=(-0.068, 0.0, 0.015)),
        material=housing_enamel,
        name="left_adapter_pad",
    )
    housing.visual(
        Box((0.012, 0.028, 0.014)),
        origin=Origin(xyz=(0.068, 0.0, 0.015)),
        material=housing_enamel,
        name="right_adapter_pad",
    )
    housing.visual(
        Box((0.086, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.048, 0.015)),
        material=housing_enamel,
        name="lower_hatch_reinforcement",
    )
    housing.visual(
        Box((0.052, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=housing_enamel,
        name="shaft_saddle",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.148, 0.120, 0.038)),
        mass=0.92,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    model.articulation(
        "wall_plate_to_housing",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    left_adapter = model.part("left_adapter")
    left_adapter.visual(
        Box((0.008, 0.108, 0.004)),
        origin=Origin(xyz=(-0.084, 0.0, 0.006)),
        material=adapter_metal,
        name="strap",
    )
    left_adapter.visual(
        Box((0.014, 0.036, 0.012)),
        origin=Origin(xyz=(-0.081, 0.0, 0.014)),
        material=adapter_metal,
        name="gusset_block",
    )
    left_adapter.visual(
        Box((0.012, 0.024, 0.014)),
        origin=Origin(xyz=(-0.080, 0.0, 0.015)),
        material=adapter_metal,
        name="housing_lug",
    )
    left_adapter.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(-0.084, 0.034, 0.0095)),
        material=bolt_dark,
        name="upper_bolt",
    )
    left_adapter.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(-0.084, -0.034, 0.0095)),
        material=bolt_dark,
        name="lower_bolt",
    )
    left_adapter.inertial = Inertial.from_geometry(
        Box((0.014, 0.108, 0.020)),
        mass=0.11,
        origin=Origin(xyz=(-0.081, 0.0, 0.010)),
    )
    model.articulation(
        "wall_plate_to_left_adapter",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=left_adapter,
        origin=Origin(),
    )

    right_adapter = model.part("right_adapter")
    right_adapter.visual(
        Box((0.008, 0.108, 0.004)),
        origin=Origin(xyz=(0.084, 0.0, 0.006)),
        material=adapter_metal,
        name="strap",
    )
    right_adapter.visual(
        Box((0.014, 0.036, 0.012)),
        origin=Origin(xyz=(0.081, 0.0, 0.014)),
        material=adapter_metal,
        name="gusset_block",
    )
    right_adapter.visual(
        Box((0.012, 0.024, 0.014)),
        origin=Origin(xyz=(0.080, 0.0, 0.015)),
        material=adapter_metal,
        name="housing_lug",
    )
    right_adapter.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(0.084, 0.034, 0.0095)),
        material=bolt_dark,
        name="upper_bolt",
    )
    right_adapter.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(0.084, -0.034, 0.0095)),
        material=bolt_dark,
        name="lower_bolt",
    )
    right_adapter.inertial = Inertial.from_geometry(
        Box((0.014, 0.108, 0.020)),
        mass=0.11,
        origin=Origin(xyz=(0.081, 0.0, 0.010)),
    )
    model.articulation(
        "wall_plate_to_right_adapter",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=right_adapter,
        origin=Origin(),
    )

    lower_service_hatch = model.part("lower_service_hatch")
    lower_service_hatch.visual(
        Box((0.070, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=service_panel,
        name="hatch_plate",
    )
    lower_service_hatch.visual(
        Box((0.040, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=adapter_metal,
        name="latch_bar",
    )
    for bolt_name, bolt_xyz in (
        ("bolt_ul", (-0.028, 0.004, 0.005)),
        ("bolt_ur", (0.028, 0.004, 0.005)),
        ("bolt_ll", (-0.028, -0.004, 0.005)),
        ("bolt_lr", (0.028, -0.004, 0.005)),
    ):
        lower_service_hatch.visual(
            Cylinder(radius=0.003, length=0.002),
            origin=Origin(xyz=bolt_xyz),
            material=bolt_dark,
            name=bolt_name,
        )
    lower_service_hatch.inertial = Inertial.from_geometry(
        Box((0.070, 0.016, 0.006)),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "housing_to_lower_service_hatch",
        ArticulationType.FIXED,
        parent=housing,
        child=lower_service_hatch,
        origin=Origin(xyz=(0.0, -0.048, 0.038)),
    )

    side_service_hatch = model.part("side_service_hatch")
    side_service_hatch.visual(
        Box((0.004, 0.030, 0.022)),
        origin=Origin(xyz=(0.002, 0.0, 0.012)),
        material=service_panel,
        name="side_plate",
    )
    side_service_hatch.visual(
        Box((0.003, 0.016, 0.004)),
        origin=Origin(xyz=(0.0055, 0.0, 0.022)),
        material=adapter_metal,
        name="side_handle",
    )
    for bolt_name, bolt_y in (("forward_bolt", 0.010), ("aft_bolt", -0.010)):
        side_service_hatch.visual(
            Cylinder(radius=0.0025, length=0.002),
            origin=Origin(
                xyz=(0.005, bolt_y, 0.012),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_dark,
            name=bolt_name,
        )
    side_service_hatch.inertial = Inertial.from_geometry(
        Box((0.006, 0.030, 0.022)),
        mass=0.035,
        origin=Origin(xyz=(0.003, 0.0, 0.012)),
    )
    model.articulation(
        "housing_to_side_service_hatch",
        ArticulationType.FIXED,
        parent=housing,
        child=side_service_hatch,
        origin=Origin(xyz=(0.074, 0.032, 0.008)),
    )

    center_shaft = model.part("center_shaft")
    center_shaft.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shaft_metal,
        name="base_collar",
    )
    center_shaft.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=shaft_metal,
        name="shaft_pin",
    )
    center_shaft.visual(
        Box((0.024, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shaft_metal,
        name="horizontal_web",
    )
    center_shaft.visual(
        Box((0.003, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shaft_metal,
        name="vertical_web",
    )
    center_shaft.inertial = Inertial.from_geometry(
        Box((0.024, 0.024, 0.018)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )
    model.articulation(
        "housing_to_center_shaft",
        ArticulationType.FIXED,
        parent=housing,
        child=center_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
    )

    retainer = model.part("retainer")
    retainer.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0195)),
        material=retainer_brass,
        name="retainer_washer",
    )
    retainer.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=retainer_brass,
        name="retainer_cap",
    )
    retainer.inertial = Inertial.from_geometry(
        Box((0.024, 0.024, 0.007)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
    )
    model.articulation(
        "center_shaft_to_retainer",
        ArticulationType.FIXED,
        parent=center_shaft,
        child=retainer,
        origin=Origin(),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(_dial_geometry(), "thermostat_dial"),
        material=dial_black,
        name="dial_body",
    )
    for rib_name, rib_xyz, rib_size in (
        ("rib_pos_x", (0.0135, 0.0, 0.013), (0.008, 0.004, 0.010)),
        ("rib_neg_x", (-0.0135, 0.0, 0.013), (0.008, 0.004, 0.010)),
        ("rib_pos_y", (0.0, 0.0135, 0.013), (0.004, 0.008, 0.010)),
        ("rib_neg_y", (0.0, -0.0135, 0.013), (0.004, 0.008, 0.010)),
    ):
        dial.visual(
            Box(rib_size),
            origin=Origin(xyz=rib_xyz),
            material=shaft_metal,
            name=rib_name,
        )
    dial.visual(
        Box((0.018, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.033, 0.021)),
        material=pointer_cream,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.076, 0.076, 0.023)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
    )
    model.articulation(
        "center_shaft_to_dial",
        ArticulationType.REVOLUTE,
        parent=center_shaft,
        child=dial,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=1.8,
            lower=-2.35,
            upper=2.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    housing = object_model.get_part("housing")
    left_adapter = object_model.get_part("left_adapter")
    right_adapter = object_model.get_part("right_adapter")
    lower_service_hatch = object_model.get_part("lower_service_hatch")
    side_service_hatch = object_model.get_part("side_service_hatch")
    center_shaft = object_model.get_part("center_shaft")
    retainer = object_model.get_part("retainer")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("center_shaft_to_dial")

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

    ctx.expect_contact(housing, wall_plate, name="housing_is_backed_by_wall_plate")
    ctx.expect_contact(left_adapter, wall_plate, name="left_adapter_is_bolted_to_plate")
    ctx.expect_contact(right_adapter, wall_plate, name="right_adapter_is_bolted_to_plate")
    ctx.expect_contact(left_adapter, housing, name="left_adapter_braces_housing")
    ctx.expect_contact(right_adapter, housing, name="right_adapter_braces_housing")
    ctx.expect_contact(lower_service_hatch, housing, name="lower_service_hatch_is_seated")
    ctx.expect_contact(side_service_hatch, housing, name="side_service_hatch_is_seated")
    ctx.expect_contact(center_shaft, housing, name="center_shaft_is_supported_by_housing")
    ctx.expect_contact(dial, retainer, name="dial_is_captured_by_retainer")
    ctx.expect_contact(retainer, center_shaft, name="retainer_is_captured_on_shaft")

    ctx.expect_origin_distance(
        dial,
        center_shaft,
        axes="xy",
        max_dist=0.0005,
        name="dial_is_concentric_with_shaft",
    )
    ctx.expect_origin_distance(
        retainer,
        center_shaft,
        axes="xy",
        max_dist=0.0005,
        name="retainer_stays_on_axis",
    )
    ctx.expect_gap(
        dial,
        housing,
        axis="z",
        min_gap=0.0008,
        max_gap=0.0015,
        name="dial_keeps_front_clearance",
    )
    ctx.expect_overlap(
        dial,
        housing,
        axes="xy",
        min_overlap=0.070,
        name="dial_remains_centered_over_body",
    )
    ctx.check(
        "dial_joint_axis_and_limits",
        dial_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in dial_joint.axis) == (0.0, 0.0, 1.0)
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is not None
        and dial_joint.motion_limits.upper is not None
        and dial_joint.motion_limits.lower < 0.0 < dial_joint.motion_limits.upper,
        (
            f"type={dial_joint.articulation_type}, "
            f"axis={dial_joint.axis}, "
            f"limits={dial_joint.motion_limits}"
        ),
    )
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="dial_clearance_in_motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
