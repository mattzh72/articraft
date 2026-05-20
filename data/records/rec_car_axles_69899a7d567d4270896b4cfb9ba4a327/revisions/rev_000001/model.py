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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x_pos: float,
    *,
    width_y: float,
    height_z: float,
    exponent: float = 2.2,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos)
        for y_pos, z_pos in superellipse_profile(
            width_y,
            height_z,
            exponent=exponent,
            segments=segments,
        )
    ]


def _annulus_profile(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    flange_radius: float | None = None,
    flange_length: float = 0.0,
    outer_step_radius: float | None = None,
    outer_step_pos: float | None = None,
) -> list[tuple[float, float]]:
    flange_radius = outer_radius if flange_radius is None else flange_radius
    outer_step_radius = outer_radius if outer_step_radius is None else outer_step_radius
    outer_step_pos = length * 0.55 if outer_step_pos is None else outer_step_pos
    return [
        (inner_radius, 0.0),
        (outer_radius, 0.0),
        (outer_radius, min(length * 0.35, outer_step_pos)),
        (outer_step_radius, outer_step_pos),
        (flange_radius, length),
        (inner_radius * 1.08, length),
        (inner_radius, max(length * 0.62, outer_step_pos)),
        (inner_radius, 0.0),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="live_rear_axle")

    cast_steel = model.material("cast_steel", rgba=(0.25, 0.26, 0.28, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.10, 0.11, 0.12, 1.0))

    axle_housing = model.part("axle_housing")

    housing_sections = [
        _yz_section(-0.680, width_y=0.102, height_z=0.102, exponent=2.0),
        _yz_section(-0.500, width_y=0.102, height_z=0.102, exponent=2.0),
        _yz_section(-0.300, width_y=0.120, height_z=0.116, exponent=2.1),
        _yz_section(-0.170, width_y=0.190, height_z=0.175, exponent=2.15),
        _yz_section(0.000, width_y=0.360, height_z=0.310, exponent=2.35),
        _yz_section(0.170, width_y=0.190, height_z=0.175, exponent=2.15),
        _yz_section(0.300, width_y=0.120, height_z=0.116, exponent=2.1),
        _yz_section(0.500, width_y=0.102, height_z=0.102, exponent=2.0),
        _yz_section(0.680, width_y=0.102, height_z=0.102, exponent=2.0),
    ]
    housing_mesh = _save_mesh("axle_center_housing", section_loft(housing_sections))
    axle_housing.visual(
        housing_mesh,
        material=cast_steel,
        name="center_housing",
    )

    rear_cover_profile = [
        (0.0, 0.0),
        (0.110, 0.0),
        (0.145, 0.016),
        (0.170, 0.055),
        (0.160, 0.100),
        (0.110, 0.142),
        (0.040, 0.162),
        (0.0, 0.168),
        (0.0, 0.0),
    ]
    axle_housing.visual(
        _save_mesh("differential_rear_cover", LatheGeometry(rear_cover_profile, segments=48)),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_steel,
        name="rear_cover",
    )

    pinion_nose_profile = [
        (0.028, 0.0),
        (0.066, 0.0),
        (0.074, 0.060),
        (0.086, 0.120),
        (0.070, 0.180),
        (0.034, 0.180),
        (0.030, 0.110),
        (0.028, 0.0),
    ]
    axle_housing.visual(
        _save_mesh("pinion_nose", LatheGeometry(pinion_nose_profile, segments=48)),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cast_steel,
        name="pinion_nose",
    )

    axle_housing.visual(
        Cylinder(radius=0.075, length=0.016),
        origin=Origin(xyz=(-0.688, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="left_bearing_seat",
    )
    axle_housing.visual(
        Cylinder(radius=0.075, length=0.016),
        origin=Origin(xyz=(0.688, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="right_bearing_seat",
    )
    axle_housing.visual(
        Cylinder(radius=0.030, length=0.085),
        origin=Origin(xyz=(-0.7385, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="left_spindle",
    )
    axle_housing.visual(
        Cylinder(radius=0.030, length=0.085),
        origin=Origin(xyz=(0.7385, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="right_spindle",
    )
    axle_housing.visual(
        Cylinder(radius=0.052, length=0.180),
        origin=Origin(xyz=(-0.510, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_steel,
        name="left_spring_pad",
    )
    axle_housing.visual(
        Cylinder(radius=0.052, length=0.180),
        origin=Origin(xyz=(0.510, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_steel,
        name="right_spring_pad",
    )
    axle_housing.inertial = Inertial.from_geometry(
        Box((1.64, 0.40, 0.42)),
        mass=95.0,
        origin=Origin(),
    )

    pinion_input_shaft = model.part("pinion_input_shaft")
    pinion_input_shaft.visual(
        Cylinder(radius=0.022, length=0.220),
        origin=Origin(xyz=(0.0, 0.050, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pinion_shaft",
    )
    pinion_input_shaft.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pinion_shaft_shoulder",
    )
    pinion_input_shaft.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.155, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="pinion_yoke_flange",
    )
    for bolt_index in range(4):
        angle = bolt_index * (math.tau / 4.0)
        pinion_input_shaft.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(
                xyz=(0.038 * math.cos(angle), 0.164, 0.038 * math.sin(angle)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_oxide,
            name=f"pinion_yoke_ear_{bolt_index}",
        )
    pinion_input_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.250),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    hub_profile = _annulus_profile(
        inner_radius=0.036,
        outer_radius=0.072,
        length=0.128,
        flange_radius=0.102,
        outer_step_radius=0.090,
        outer_step_pos=0.095,
    )
    hub_mesh = _save_mesh("wheel_hub_shell", LatheGeometry(hub_profile, segments=56))

    left_hub = model.part("left_hub")
    left_hub.visual(
        hub_mesh,
        origin=Origin(xyz=(-0.128, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="left_hub_shell",
    )
    left_hub.visual(
        Cylinder(radius=0.048, length=0.040),
        origin=Origin(xyz=(-0.114, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="left_dust_cap",
    )
    for stud_index in range(5):
        angle = stud_index * (math.tau / 5.0)
        left_hub.visual(
            Cylinder(radius=0.007, length=0.026),
            origin=Origin(
                xyz=(-0.124, 0.040 * math.cos(angle), 0.040 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
            name=f"left_wheel_stud_{stud_index}",
        )
    left_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.128),
        mass=8.0,
        origin=Origin(xyz=(-0.064, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_hub = model.part("right_hub")
    right_hub.visual(
        hub_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="right_hub_shell",
    )
    right_hub.visual(
        Cylinder(radius=0.048, length=0.040),
        origin=Origin(xyz=(0.114, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="right_dust_cap",
    )
    for stud_index in range(5):
        angle = stud_index * (math.tau / 5.0)
        right_hub.visual(
            Cylinder(radius=0.007, length=0.026),
            origin=Origin(
                xyz=(0.124, 0.040 * math.cos(angle), 0.040 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
            name=f"right_wheel_stud_{stud_index}",
        )
    right_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.128),
        mass=8.0,
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "pinion_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_housing,
        child=pinion_input_shaft,
        origin=Origin(xyz=(0.0, 0.240, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=120.0),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_housing,
        child=left_hub,
        origin=Origin(xyz=(-0.696, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=90.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_housing,
        child=right_hub,
        origin=Origin(xyz=(0.696, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=90.0),
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

    axle_housing = object_model.get_part("axle_housing")
    pinion_input_shaft = object_model.get_part("pinion_input_shaft")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    pinion_spin = object_model.get_articulation("pinion_spin")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

    ctx.check(
        "pinion articulation axis is longitudinal",
        pinion_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={pinion_spin.axis}",
    )
    ctx.check(
        "wheel hub articulations spin on axle axis",
        left_hub_spin.axis == (1.0, 0.0, 0.0) and right_hub_spin.axis == (1.0, 0.0, 0.0),
        details=f"left={left_hub_spin.axis}, right={right_hub_spin.axis}",
    )

    ctx.expect_contact(
        pinion_input_shaft,
        axle_housing,
        elem_a="pinion_shaft_shoulder",
        elem_b="pinion_nose",
        contact_tol=0.0005,
        name="pinion shaft is seated against the nose bearing support",
    )
    ctx.expect_contact(
        left_hub,
        axle_housing,
        elem_a="left_hub_shell",
        elem_b="left_bearing_seat",
        contact_tol=0.0005,
        name="left hub is supported at the axle bearing seat",
    )
    ctx.expect_contact(
        right_hub,
        axle_housing,
        elem_a="right_hub_shell",
        elem_b="right_bearing_seat",
        contact_tol=0.0005,
        name="right hub is supported at the axle bearing seat",
    )

    ctx.expect_origin_gap(
        right_hub,
        left_hub,
        axis="x",
        min_gap=1.30,
        max_gap=1.50,
        name="wheel hubs sit at opposite ends of the axle",
    )

    with ctx.pose({pinion_spin: 1.2, left_hub_spin: 2.0, right_hub_spin: -1.7}):
        ctx.expect_contact(
            pinion_input_shaft,
            axle_housing,
            elem_a="pinion_shaft_shoulder",
            elem_b="pinion_nose",
            contact_tol=0.0005,
            name="pinion remains supported while spinning",
        )
        ctx.expect_contact(
            left_hub,
            axle_housing,
            elem_a="left_hub_shell",
            elem_b="left_bearing_seat",
            contact_tol=0.0005,
            name="left hub remains supported while spinning",
        )
        ctx.expect_contact(
            right_hub,
            axle_housing,
            elem_a="right_hub_shell",
            elem_b="right_bearing_seat",
            contact_tol=0.0005,
            name="right hub remains supported while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
