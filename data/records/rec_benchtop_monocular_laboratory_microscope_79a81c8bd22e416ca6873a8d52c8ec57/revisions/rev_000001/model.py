from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TILT_DEG = 30.0
TILT_RAD = math.radians(TILT_DEG)
OPTICAL_AXIS = (0.0, -math.sin(TILT_RAD), math.cos(TILT_RAD))
NOSE_JOINT = (0.0, 0.135, 0.054)
STAGE_Y = 0.095
PIVOT_Y = 0.113


def _v_add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _box(size: tuple[float, float, float], center: tuple[float, float, float], fillet: float | None = None):
    shape = cq.Workplane("XY").box(*size).translate(center)
    if fillet is not None and fillet > 0.0:
        shape = shape.edges("|Z").fillet(fillet)
    return shape


def _cyl_z(radius: float, length: float, center: tuple[float, float, float] = (0.0, 0.0, 0.0)):
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]):
    return _cyl_z(radius, length).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0).translate(center)


def _ring_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _tilt_about_x(shape, angle_deg: float = TILT_DEG):
    return shape.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)


def _add_mesh_visual(part, shape, name: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, name), material=material, name=name)


def _base_cast_shape():
    rear_pad = _box((0.115, 0.120, 0.020), (0.0, -0.032, 0.010), fillet=0.010)
    left_foot = _box((0.052, 0.092, 0.016), (-0.046, 0.064, 0.008), fillet=0.009)
    right_foot = _box((0.052, 0.092, 0.016), (0.046, 0.064, 0.008), fillet=0.009)
    bridge = _box((0.040, 0.040, 0.014), (0.0, 0.020, 0.007), fillet=0.005)
    column_boss = _cyl_z(0.030, 0.010, (0.0, -0.055, 0.025))
    return rear_pad.union(left_foot).union(right_foot).union(bridge).union(column_boss)


def _carriage_body_shape():
    sleeve = _ring_z(0.027, 0.016, 0.082)
    rear_post = _box((0.030, 0.034, 0.082), (0.0, 0.044, 0.062), fillet=0.004)
    upper_body = _box((0.032, 0.046, 0.036), (0.0, 0.036, 0.092), fillet=0.004)
    front_pad = _box((0.024, 0.020, 0.022), (0.0, 0.064, 0.102), fillet=0.003)
    nose_neck = _box((0.018, 0.058, 0.040), (0.0, 0.085, 0.062), fillet=0.003)

    main_tube_center = _v_add(NOSE_JOINT, _v_scale(OPTICAL_AXIS, 0.078))
    eyepiece_center = _v_add(NOSE_JOINT, _v_scale(OPTICAL_AXIS, 0.160))
    ocular_cap_center = _v_add(NOSE_JOINT, _v_scale(OPTICAL_AXIS, 0.204))

    main_tube = _tilt_about_x(_cyl_z(0.018, 0.102)).translate(main_tube_center)
    eyepiece_tube = _tilt_about_x(_cyl_z(0.013, 0.078)).translate(eyepiece_center)
    ocular_cap = _tilt_about_x(_cyl_z(0.0145, 0.016)).translate(ocular_cap_center)

    left_coarse = _cyl_x(0.013, 0.008, (-0.031, 0.034, 0.012))
    right_coarse = _cyl_x(0.013, 0.008, (0.031, 0.034, 0.012))
    left_fine = _cyl_x(0.0065, 0.010, (-0.040, 0.034, 0.012))
    right_fine = _cyl_x(0.0065, 0.010, (0.040, 0.034, 0.012))

    return (
        sleeve.union(rear_post)
        .union(upper_body)
        .union(front_pad)
        .union(nose_neck)
        .union(main_tube)
        .union(eyepiece_tube)
        .union(ocular_cap)
        .union(left_coarse)
        .union(right_coarse)
        .union(left_fine)
        .union(right_fine)
    )


def _nosepiece_collar_shape():
    return _tilt_about_x(_ring_z(0.020, 0.0105, 0.014)).translate(NOSE_JOINT)


def _stage_bridge_shape():
    return _box((0.042, 0.090, 0.014), (0.0, 0.062, -0.005), fillet=0.003)


def _stage_guide_shape():
    base_plate = _box((0.040, 0.110, 0.004), (0.0, STAGE_Y, 0.002))
    left_rail = _box((0.008, 0.110, 0.008), (-0.016, STAGE_Y, 0.006))
    right_rail = _box((0.008, 0.110, 0.008), (0.016, STAGE_Y, 0.006))
    return base_plate.union(left_rail).union(right_rail)


def _condenser_housing_shape():
    stem = _box((0.012, 0.018, 0.028), (0.0, STAGE_Y, -0.014), fillet=0.002)
    housing = _cyl_z(0.018, 0.030, (0.0, STAGE_Y, -0.043))
    lower_ring = _cyl_z(0.011, 0.012, (0.0, STAGE_Y, -0.064))
    support_tab = _box((0.024, 0.012, 0.004), (0.012, PIVOT_Y, -0.047))
    return stem.union(housing).union(lower_ring).union(support_tab)


def _stage_plate_shape():
    plate = cq.Workplane("XY").box(0.095, 0.090, 0.004)
    opening = cq.Workplane("XY").box(0.040, 0.040, 0.012)
    return plate.cut(opening).translate((0.0, 0.0, 0.014))


def _stage_saddle_shape():
    return _box((0.018, 0.105, 0.012), (0.0, 0.0, 0.006), fillet=0.0015)


def _turret_hub_shape():
    disk = _cyl_z(0.021, 0.006, (0.0, 0.0, -0.008))
    spindle = _cyl_z(0.0105, 0.016, (0.0, 0.0, 0.001))
    return disk.union(spindle)


def _objective_shape(length: float, angle_deg: float):
    angle_rad = math.radians(angle_deg)
    radius = 0.012
    center = (radius * math.cos(angle_rad), radius * math.sin(angle_rad), -0.011 - length / 2.0)
    return _cyl_z(0.0055, length, center)


def _lever_shape():
    hub = _ring_z(0.0055, 0.0032, 0.004)
    arm = _box((0.004, 0.026, 0.004), (0.0, 0.016, 0.0))
    tip = _cyl_z(0.0035, 0.004, (0.0, 0.030, 0.0))
    return hub.union(arm).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clinical_monocular_microscope")

    model.material("cast_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("enamel_gray", rgba=(0.78, 0.79, 0.80, 1.0))
    model.material("stage_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("optic_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("steel_dark", rgba=(0.32, 0.34, 0.37, 1.0))

    stand = model.part("stand")
    _add_mesh_visual(stand, _base_cast_shape(), "base_cast", "cast_gray")
    stand.visual(
        Cylinder(radius=0.014, length=0.220),
        origin=Origin(xyz=(0.0, -0.055, 0.130)),
        material="steel_dark",
        name="column",
    )
    _add_mesh_visual(
        stand,
        _box((0.038, 0.032, 0.060), (0.0, -0.074, 0.050), fillet=0.003),
        "rear_brace",
        "cast_gray",
    )

    carriage = model.part("carriage")
    _add_mesh_visual(carriage, _ring_z(0.027, 0.016, 0.082), "column_sleeve", "enamel_gray")
    carriage.visual(
        Box((0.002, 0.010, 0.060)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material="steel_dark",
        name="guide_pad",
    )
    carriage.visual(
        Box((0.030, 0.034, 0.082)),
        origin=Origin(xyz=(0.0, 0.0425, 0.062)),
        material="enamel_gray",
        name="rear_body",
    )
    carriage.visual(
        Box((0.034, 0.054, 0.040)),
        origin=Origin(xyz=(0.0, 0.050, 0.094)),
        material="enamel_gray",
        name="upper_body",
    )
    carriage.visual(
        Box((0.022, 0.032, 0.072)),
        origin=Origin(xyz=(0.0, 0.085, 0.068)),
        material="enamel_gray",
        name="head_support",
    )
    carriage.visual(
        Box((0.014, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, 0.106, 0.050)),
        material="enamel_gray",
        name="collar_support",
    )
    carriage.visual(
        Box((0.018, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.030, 0.012)),
        material="enamel_gray",
        name="stage_pillar",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=_v_add(NOSE_JOINT, _v_scale(OPTICAL_AXIS, 0.090)), rpy=(TILT_RAD, 0.0, 0.0)),
        material="enamel_gray",
        name="main_tube",
    )
    carriage.visual(
        Cylinder(radius=0.013, length=0.090),
        origin=Origin(xyz=_v_add(NOSE_JOINT, _v_scale(OPTICAL_AXIS, 0.165)), rpy=(TILT_RAD, 0.0, 0.0)),
        material="optic_black",
        name="eyepiece_tube",
    )
    carriage.visual(
        Cylinder(radius=0.0145, length=0.020),
        origin=Origin(xyz=_v_add(NOSE_JOINT, _v_scale(OPTICAL_AXIS, 0.210)), rpy=(TILT_RAD, 0.0, 0.0)),
        material="optic_black",
        name="ocular_cap",
    )
    carriage.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(-0.025, 0.044, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="optic_black",
        name="coarse_knob_0",
    )
    carriage.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.025, 0.044, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="optic_black",
        name="coarse_knob_1",
    )
    carriage.visual(
        Cylinder(radius=0.0065, length=0.012),
        origin=Origin(xyz=(-0.034, 0.044, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel_dark",
        name="fine_knob_0",
    )
    carriage.visual(
        Cylinder(radius=0.0065, length=0.012),
        origin=Origin(xyz=(0.034, 0.044, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel_dark",
        name="fine_knob_1",
    )
    _add_mesh_visual(carriage, _nosepiece_collar_shape(), "nosepiece_collar", "optic_black")
    _add_mesh_visual(carriage, _stage_bridge_shape(), "stage_bridge", "enamel_gray")
    _add_mesh_visual(carriage, _stage_guide_shape(), "stage_guide", "steel_dark")
    _add_mesh_visual(carriage, _condenser_housing_shape(), "condenser_housing", "optic_black")
    carriage.visual(
        Cylinder(radius=0.0032, length=0.008),
        origin=Origin(xyz=(0.020, PIVOT_Y, -0.043)),
        material="steel_dark",
        name="pivot_pin",
    )

    stage = model.part("stage")
    _add_mesh_visual(stage, _stage_plate_shape(), "stage_plate", "stage_black")
    _add_mesh_visual(stage, _stage_saddle_shape(), "stage_saddle", "steel_dark")

    turret = model.part("turret")
    _add_mesh_visual(turret, _turret_hub_shape(), "turret_hub", "optic_black")
    _add_mesh_visual(turret, _objective_shape(0.024, 0.0), "objective_0", "optic_black")
    _add_mesh_visual(turret, _objective_shape(0.030, 120.0), "objective_1", "optic_black")
    _add_mesh_visual(turret, _objective_shape(0.020, 240.0), "objective_2", "optic_black")

    diaphragm_lever = model.part("diaphragm_lever")
    _add_mesh_visual(diaphragm_lever, _lever_shape(), "lever", "steel_dark")

    focus_slide = model.articulation(
        "stand_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.055, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.020, upper=0.040, effort=140.0, velocity=0.060),
    )
    stage_slide = model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=(0.0, STAGE_Y, 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.022, upper=0.022, effort=40.0, velocity=0.040),
    )
    turret_spin = model.articulation(
        "carriage_to_turret",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=turret,
        origin=Origin(xyz=NOSE_JOINT, rpy=(TILT_RAD, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0),
    )
    iris_lever = model.articulation(
        "carriage_to_diaphragm_lever",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=diaphragm_lever,
        origin=Origin(xyz=(0.020, PIVOT_Y, -0.043)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=2.0, velocity=2.0),
    )

    focus_slide.meta["qc_samples"] = [-0.020, 0.0, 0.040]
    stage_slide.meta["qc_samples"] = [-0.022, 0.0, 0.022]
    turret_spin.meta["qc_samples"] = [0.0, 1.7, 3.4]
    iris_lever.meta["qc_samples"] = [-0.55, 0.0, 0.55]

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    carriage = object_model.get_part("carriage")
    stage = object_model.get_part("stage")
    turret = object_model.get_part("turret")
    diaphragm_lever = object_model.get_part("diaphragm_lever")

    focus_slide = object_model.get_articulation("stand_to_carriage")
    stage_slide = object_model.get_articulation("carriage_to_stage")
    turret_spin = object_model.get_articulation("carriage_to_turret")
    iris_lever = object_model.get_articulation("carriage_to_diaphragm_lever")

    focus_limits = focus_slide.motion_limits
    stage_limits = stage_slide.motion_limits
    lever_limits = iris_lever.motion_limits

    ctx.allow_overlap(
        turret,
        carriage,
        elem_a="turret_hub",
        elem_b="nosepiece_collar",
        reason="The revolving turret hub is intentionally simplified as a captured bearing fit nested inside the nosepiece collar.",
    )
    ctx.expect_overlap(
        turret,
        carriage,
        axes="xy",
        elem_a="turret_hub",
        elem_b="nosepiece_collar",
        min_overlap=0.020,
        name="turret hub stays centered in the nosepiece collar",
    )
    ctx.expect_contact(
        diaphragm_lever,
        carriage,
        elem_a="lever",
        elem_b="pivot_pin",
        name="diaphragm lever rides on the condenser pivot",
    )
    ctx.expect_contact(
        stage,
        carriage,
        elem_a="stage_saddle",
        elem_b="stage_guide",
        name="stage saddle stays supported by the guide channel",
    )

    if focus_limits is not None and focus_limits.lower is not None and focus_limits.upper is not None:
        with ctx.pose({focus_slide: focus_limits.lower}):
            ctx.expect_overlap(
                carriage,
                stand,
                axes="z",
                elem_a="column_sleeve",
                elem_b="column",
                min_overlap=0.080,
                name="focus sleeve remains engaged at the low end",
            )
            low_pos = ctx.part_world_position(carriage)
        with ctx.pose({focus_slide: focus_limits.upper}):
            ctx.expect_overlap(
                carriage,
                stand,
                axes="z",
                elem_a="column_sleeve",
                elem_b="column",
                min_overlap=0.080,
                name="focus sleeve remains engaged at the high end",
            )
            high_pos = ctx.part_world_position(carriage)
        ctx.check(
            "focus carriage rises on the column",
            low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.050,
            details=f"low={low_pos}, high={high_pos}",
        )

    if stage_limits is not None and stage_limits.lower is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.lower}):
            ctx.expect_within(
                stage,
                carriage,
                axes="x",
                inner_elem="stage_saddle",
                outer_elem="stage_guide",
                margin=0.0,
                name="stage saddle stays laterally inside the guide at the rear limit",
            )
            ctx.expect_overlap(
                stage,
                carriage,
                axes="y",
                elem_a="stage_saddle",
                elem_b="stage_guide",
                min_overlap=0.080,
                name="stage saddle remains inserted at the rear limit",
            )
            rear_pos = ctx.part_world_position(stage)
        with ctx.pose({stage_slide: stage_limits.upper}):
            ctx.expect_within(
                stage,
                carriage,
                axes="x",
                inner_elem="stage_saddle",
                outer_elem="stage_guide",
                margin=0.0,
                name="stage saddle stays laterally inside the guide at the front limit",
            )
            ctx.expect_overlap(
                stage,
                carriage,
                axes="y",
                elem_a="stage_saddle",
                elem_b="stage_guide",
                min_overlap=0.080,
                name="stage saddle remains inserted at the front limit",
            )
            front_pos = ctx.part_world_position(stage)
        ctx.check(
            "stage travels front to back",
            rear_pos is not None and front_pos is not None and front_pos[1] > rear_pos[1] + 0.040,
            details=f"rear={rear_pos}, front={front_pos}",
        )

    objective_rest_aabb = ctx.part_element_world_aabb(turret, elem="objective_0")
    with ctx.pose({turret_spin: 2.09439510239}):
        objective_rotated_aabb = ctx.part_element_world_aabb(turret, elem="objective_0")
    objective_rest_center = _aabb_center(objective_rest_aabb)
    objective_rotated_center = _aabb_center(objective_rotated_aabb)
    ctx.check(
        "turret rotates objectives around the nosepiece axis",
        objective_rest_center is not None
        and objective_rotated_center is not None
        and (
            abs(objective_rest_center[0] - objective_rotated_center[0]) > 0.008
            or abs(objective_rest_center[1] - objective_rotated_center[1]) > 0.008
        ),
        details=f"rest={objective_rest_center}, rotated={objective_rotated_center}",
    )

    if lever_limits is not None and lever_limits.lower is not None and lever_limits.upper is not None:
        with ctx.pose({iris_lever: lever_limits.lower}):
            lever_low_aabb = ctx.part_element_world_aabb(diaphragm_lever, elem="lever")
        with ctx.pose({iris_lever: lever_limits.upper}):
            lever_high_aabb = ctx.part_element_world_aabb(diaphragm_lever, elem="lever")
        lever_low_center = _aabb_center(lever_low_aabb)
        lever_high_center = _aabb_center(lever_high_aabb)
        ctx.check(
            "diaphragm lever sweeps beneath the stage",
            lever_low_center is not None
            and lever_high_center is not None
            and (
                abs(lever_low_center[0] - lever_high_center[0]) > 0.008
                or abs(lever_low_center[1] - lever_high_center[1]) > 0.008
            ),
            details=f"low={lever_low_center}, high={lever_high_center}",
        )

    return ctx.report()


object_model = build_object_model()
