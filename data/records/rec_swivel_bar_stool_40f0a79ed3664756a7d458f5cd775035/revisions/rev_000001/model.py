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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_section(
    width: float,
    depth: float,
    corner_radius: float,
    z: float,
    *,
    y_shift: float = 0.0,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_shift, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            corner_radius,
            corner_segments=corner_segments,
        )
    ]


def _polar(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_swivel_bar_stool")

    painted_metal = model.material("painted_metal", rgba=(0.26, 0.27, 0.29, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    polymer_shell = model.material("polymer_shell", rgba=(0.83, 0.83, 0.80, 1.0))
    elastomer_dark = model.material("elastomer_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.43, 0.43, 0.78)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    base_disc = LatheGeometry(
        [
            (0.0, 0.000),
            (0.060, 0.000),
            (0.150, 0.004),
            (0.190, 0.010),
            (0.205, 0.018),
            (0.198, 0.026),
            (0.160, 0.031),
            (0.050, 0.034),
            (0.0, 0.034),
        ],
        segments=72,
    )
    base.visual(
        _mesh("stool_base_disc", base_disc),
        material=painted_metal,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.190, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=elastomer_dark,
        name="base_pad",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=painted_metal,
        name="plinth",
    )

    pedestal_cover = LatheGeometry(
        [
            (0.0, 0.000),
            (0.048, 0.000),
            (0.054, 0.090),
            (0.056, 0.280),
            (0.054, 0.500),
            (0.050, 0.620),
            (0.0, 0.620),
        ],
        segments=64,
    )
    base.visual(
        _mesh("stool_pedestal_cover", pedestal_cover),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=painted_metal,
        name="pedestal_cover",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material=painted_metal,
        name="upper_collar",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.683)),
        material=satin_metal,
        name="spindle_housing",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.704)),
        material=satin_metal,
        name="lower_bearing_race",
    )

    footrest_ring = TorusGeometry(
        radius=0.155,
        tube=0.011,
        radial_segments=18,
        tubular_segments=48,
    ).translate(0.0, 0.0, 0.305)
    base.visual(
        _mesh("stool_footrest_ring", footrest_ring),
        material=satin_metal,
        name="footrest_ring",
    )

    for arm_index in range(3):
        angle = arm_index * (2.0 * math.pi / 3.0) + math.pi / 6.0
        arm = tube_from_spline_points(
            [
                _polar(0.056, angle, 0.286),
                _polar(0.095, angle, 0.294),
                _polar(0.146, angle, 0.305),
            ],
            radius=0.008,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        )
        base.visual(
            _mesh(f"stool_footrest_arm_{arm_index}", arm),
            material=satin_metal,
            name=f"footrest_arm_{arm_index}",
        )

    seat_stage = model.part("seat_stage")
    seat_stage.inertial = Inertial.from_geometry(
        Box((0.43, 0.40, 0.08)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )
    seat_stage.visual(
        Cylinder(radius=0.072, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_metal,
        name="upper_bearing_race",
    )
    seat_stage.visual(
        Cylinder(radius=0.044, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=satin_metal,
        name="swivel_spindle",
    )
    seat_stage.visual(
        Cylinder(radius=0.094, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=painted_metal,
        name="seat_plate",
    )

    seat_shell_geom = section_loft(
        [
            _rounded_section(0.320, 0.290, 0.050, 0.026, y_shift=0.004),
            _rounded_section(0.400, 0.360, 0.066, 0.040, y_shift=0.001),
            _rounded_section(0.418, 0.374, 0.070, 0.048, y_shift=0.000),
            _rounded_section(0.394, 0.350, 0.060, 0.054, y_shift=-0.002),
        ]
    )
    seat_stage.visual(
        _mesh("stool_seat_shell", seat_shell_geom),
        material=polymer_shell,
        name="seat_shell",
    )

    cushion_geom = section_loft(
        [
            _rounded_section(0.336, 0.302, 0.046, 0.050, y_shift=0.000),
            _rounded_section(0.356, 0.322, 0.052, 0.061, y_shift=-0.001),
            _rounded_section(0.348, 0.312, 0.048, 0.070, y_shift=-0.003),
        ]
    )
    seat_stage.visual(
        _mesh("stool_seat_cushion", cushion_geom),
        material=elastomer_dark,
        name="seat_cushion",
    )

    model.articulation(
        "base_to_seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.710)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
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

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    ctx.check(
        "required_parts_present",
        {"base", "seat_stage"}.issubset(part_names),
        f"present parts: {sorted(part_names)}",
    )
    ctx.check(
        "required_joint_present",
        "base_to_seat_swivel" in joint_names,
        f"present articulations: {sorted(joint_names)}",
    )

    if {"base", "seat_stage"}.issubset(part_names) and "base_to_seat_swivel" in joint_names:
        base = object_model.get_part("base")
        seat_stage = object_model.get_part("seat_stage")
        swivel = object_model.get_articulation("base_to_seat_swivel")

        ctx.check(
            "swivel_is_vertical_continuous_joint",
            swivel.joint_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
            f"type={swivel.joint_type}, axis={swivel.axis}",
        )
        ctx.expect_origin_distance(
            seat_stage,
            base,
            axes="xy",
            min_dist=0.0,
            max_dist=0.001,
            name="seat_centered_over_pedestal",
        )
        ctx.expect_origin_gap(
            seat_stage,
            base,
            axis="z",
            min_gap=0.705,
            max_gap=0.715,
            name="seat_swivel_height",
        )
        ctx.expect_contact(
            base,
            seat_stage,
            elem_a="lower_bearing_race",
            elem_b="upper_bearing_race",
            name="bearing_stack_supported_contact",
        )
        ctx.expect_overlap(
            base,
            seat_stage,
            axes="xy",
            min_overlap=0.12,
            elem_a="lower_bearing_race",
            elem_b="upper_bearing_race",
            name="bearing_stack_coaxial_overlap",
        )
        ctx.expect_within(
            seat_stage,
            seat_stage,
            axes="xy",
            inner_elem="seat_cushion",
            outer_elem="seat_shell",
            margin=0.0,
            name="cushion_inset_within_shell",
        )

        with ctx.pose({swivel: math.pi / 2.0}):
            ctx.expect_contact(
                base,
                seat_stage,
                elem_a="lower_bearing_race",
                elem_b="upper_bearing_race",
                name="bearing_contact_persists_when_swiveled",
            )
            ctx.expect_origin_distance(
                seat_stage,
                base,
                axes="xy",
                min_dist=0.0,
                max_dist=0.001,
                name="seat_remains_centered_when_swiveled",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
