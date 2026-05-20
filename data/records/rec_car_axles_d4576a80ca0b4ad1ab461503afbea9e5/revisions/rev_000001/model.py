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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="de_dion_rear_axle")

    frame_black = model.material("frame_black", rgba=(0.12, 0.12, 0.13, 1.0))
    beam_black = model.material("beam_black", rgba=(0.15, 0.15, 0.16, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.30, 0.31, 0.33, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.69, 0.72, 1.0))
    rotor_steel = model.material("rotor_steel", rgba=(0.58, 0.56, 0.53, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    carrier_frame = model.part("carrier_frame")
    carrier_frame.visual(
        Cylinder(radius=0.04, length=0.86),
        origin=Origin(xyz=(0.0, -0.02, 0.57), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_black,
        name="crossmember",
    )
    carrier_frame.visual(
        save_mesh(
            "left_hanger_brace",
            tube_from_spline_points(
                [
                    (-0.26, -0.02, 0.57),
                    (-0.33, -0.09, 0.50),
                    (-0.40, -0.16, 0.52),
                ],
                radius=0.022,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="left_hanger_brace",
    )
    carrier_frame.visual(
        save_mesh(
            "right_hanger_brace",
            tube_from_spline_points(
                [
                    (0.26, -0.02, 0.57),
                    (0.33, -0.09, 0.50),
                    (0.40, -0.16, 0.52),
                ],
                radius=0.022,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="right_hanger_brace",
    )
    carrier_frame.visual(
        save_mesh(
            "left_diff_brace",
            tube_from_spline_points(
                [
                    (-0.12, -0.02, 0.57),
                    (-0.08, -0.01, 0.54),
                    (-0.04, 0.00, 0.505),
                ],
                radius=0.018,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="left_diff_brace",
    )
    carrier_frame.visual(
        save_mesh(
            "right_diff_brace",
            tube_from_spline_points(
                [
                    (0.12, -0.02, 0.57),
                    (0.08, -0.01, 0.54),
                    (0.04, 0.00, 0.505),
                ],
                radius=0.018,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="right_diff_brace",
    )
    carrier_frame.visual(
        Box((0.10, 0.06, 0.02)),
        origin=Origin(xyz=(-0.40, -0.17, 0.41)),
        material=frame_black,
        name="left_support_pad",
    )
    carrier_frame.visual(
        Box((0.10, 0.06, 0.02)),
        origin=Origin(xyz=(0.40, -0.17, 0.41)),
        material=frame_black,
        name="right_support_pad",
    )
    carrier_frame.visual(
        Box((0.05, 0.05, 0.16)),
        origin=Origin(xyz=(-0.40, -0.15, 0.50)),
        material=frame_black,
        name="left_hanger_post",
    )
    carrier_frame.visual(
        Box((0.05, 0.05, 0.16)),
        origin=Origin(xyz=(0.40, -0.15, 0.50)),
        material=frame_black,
        name="right_hanger_post",
    )
    carrier_frame.visual(
        Box((0.22, 0.12, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=frame_black,
        name="diff_hanger_pad",
    )
    carrier_frame.inertial = Inertial.from_geometry(
        Box((0.92, 0.26, 0.22)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.09, 0.51)),
    )

    de_dion_beam = model.part("de_dion_beam")
    de_dion_beam.visual(
        save_mesh(
            "beam_left_leg",
            tube_from_spline_points(
                [
                    (-0.74, 0.04, 0.00),
                    (-0.56, 0.00, 0.00),
                    (-0.30, -0.06, 0.03),
                ],
                radius=0.036,
                samples_per_segment=14,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=beam_black,
        name="left_beam_leg",
    )
    de_dion_beam.visual(
        save_mesh(
            "beam_center_bow",
            tube_from_spline_points(
                [
                    (-0.32, -0.06, 0.03),
                    (0.0, -0.10, 0.05),
                    (0.32, -0.06, 0.03),
                ],
                radius=0.036,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=beam_black,
        name="center_bow",
    )
    de_dion_beam.visual(
        save_mesh(
            "beam_right_leg",
            tube_from_spline_points(
                [
                    (0.30, -0.06, 0.03),
                    (0.56, 0.00, 0.00),
                    (0.74, 0.04, 0.00),
                ],
                radius=0.036,
                samples_per_segment=14,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=beam_black,
        name="right_beam_leg",
    )
    de_dion_beam.visual(
        Box((0.10, 0.08, 0.08)),
        origin=Origin(xyz=(-0.40, 0.00, 0.03)),
        material=beam_black,
        name="left_pedestal",
    )
    de_dion_beam.visual(
        Box((0.10, 0.08, 0.08)),
        origin=Origin(xyz=(0.40, 0.00, 0.03)),
        material=beam_black,
        name="right_pedestal",
    )
    de_dion_beam.visual(
        Box((0.12, 0.08, 0.02)),
        origin=Origin(xyz=(-0.40, 0.03, 0.05)),
        material=beam_black,
        name="left_pad",
    )
    de_dion_beam.visual(
        Box((0.12, 0.08, 0.02)),
        origin=Origin(xyz=(0.40, 0.03, 0.05)),
        material=beam_black,
        name="right_pad",
    )
    de_dion_beam.visual(
        Box((0.14, 0.10, 0.14)),
        origin=Origin(xyz=(-0.74, 0.05, 0.00)),
        material=cast_iron,
        name="left_knuckle_block",
    )
    de_dion_beam.visual(
        Box((0.14, 0.10, 0.14)),
        origin=Origin(xyz=(0.74, 0.05, 0.00)),
        material=cast_iron,
        name="right_knuckle_block",
    )
    de_dion_beam.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(-0.80, 0.04, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="left_carrier",
    )
    de_dion_beam.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.80, 0.04, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="right_carrier",
    )
    de_dion_beam.inertial = Inertial.from_geometry(
        Box((1.70, 0.32, 0.20)),
        mass=32.0,
        origin=Origin(xyz=(0.0, -0.03, 0.02)),
    )

    differential_housing = model.part("differential_housing")
    pumpkin_profile = [
        (0.0, -0.17),
        (0.05, -0.16),
        (0.09, -0.12),
        (0.115, -0.06),
        (0.13, 0.0),
        (0.115, 0.06),
        (0.09, 0.12),
        (0.05, 0.16),
        (0.0, 0.17),
    ]
    differential_housing.visual(
        save_mesh(
            "pumpkin_shell",
            LatheGeometry(pumpkin_profile, segments=56).rotate_y(math.pi / 2.0),
        ),
        material=cast_iron,
        name="pumpkin_shell",
    )
    differential_housing.visual(
        Cylinder(radius=0.11, length=0.04),
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="rear_cover",
    )
    differential_housing.visual(
        Cylinder(radius=0.055, length=0.10),
        origin=Origin(xyz=(-0.19, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="left_side_flange",
    )
    differential_housing.visual(
        Cylinder(radius=0.055, length=0.10),
        origin=Origin(xyz=(0.19, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="right_side_flange",
    )
    differential_housing.visual(
        Box((0.18, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=cast_iron,
        name="top_mount_pad",
    )
    differential_housing.visual(
        Box((0.16, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=cast_iron,
        name="lower_rib",
    )
    differential_housing.inertial = Inertial.from_geometry(
        Box((0.52, 0.30, 0.32)),
        mass=26.0,
        origin=Origin(),
    )

    def add_hub(part_name: str, side_sign: float) -> None:
        hub = model.part(part_name)
        hub.visual(
            Cylinder(radius=0.050, length=0.06),
            origin=Origin(xyz=(side_sign * 0.03, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="inner_bearing",
        )
        hub.visual(
            Cylinder(radius=0.060, length=0.12),
            origin=Origin(xyz=(side_sign * 0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="hub_barrel",
        )
        hub.visual(
            Cylinder(radius=0.145, length=0.010),
            origin=Origin(xyz=(side_sign * 0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rotor_steel,
            name="brake_disc",
        )
        hub.visual(
            Cylinder(radius=0.090, length=0.028),
            origin=Origin(xyz=(side_sign * 0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=0.040, length=0.040),
            origin=Origin(xyz=(side_sign * 0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="dust_cap",
        )
        hub.inertial = Inertial.from_geometry(
            Cylinder(radius=0.15, length=0.19),
            mass=9.0,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    def add_stub(part_name: str, side_sign: float) -> None:
        stub = model.part(part_name)
        stub.visual(
            Cylinder(radius=0.055, length=0.05),
            origin=Origin(xyz=(side_sign * 0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="inner_cv",
        )
        stub.visual(
            Cylinder(radius=0.026, length=0.12),
            origin=Origin(xyz=(side_sign * 0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="stub_shaft",
        )
        stub.visual(
            Cylinder(radius=0.018, length=0.04),
            origin=Origin(xyz=(side_sign * 0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="splined_tip",
        )
        stub.inertial = Inertial.from_geometry(
            Cylinder(radius=0.06, length=0.18),
            mass=2.0,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    add_hub("left_hub", side_sign=-1.0)
    add_hub("right_hub", side_sign=1.0)
    add_stub("left_stub", side_sign=-1.0)
    add_stub("right_stub", side_sign=1.0)

    model.articulation(
        "carrier_to_beam",
        ArticulationType.FIXED,
        parent=carrier_frame,
        child=de_dion_beam,
        origin=Origin(xyz=(0.0, -0.24, 0.34)),
    )
    model.articulation(
        "carrier_to_differential",
        ArticulationType.FIXED,
        parent=carrier_frame,
        child=differential_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=de_dion_beam,
        child="left_hub",
        origin=Origin(xyz=(-0.86, 0.04, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=40.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=de_dion_beam,
        child="right_hub",
        origin=Origin(xyz=(0.86, 0.04, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=40.0),
    )
    model.articulation(
        "left_stub_spin",
        ArticulationType.CONTINUOUS,
        parent=differential_housing,
        child="left_stub",
        origin=Origin(xyz=(-0.24, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=70.0),
    )
    model.articulation(
        "right_stub_spin",
        ArticulationType.CONTINUOUS,
        parent=differential_housing,
        child="right_stub",
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=70.0),
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

    carrier_frame = object_model.get_part("carrier_frame")
    de_dion_beam = object_model.get_part("de_dion_beam")
    differential_housing = object_model.get_part("differential_housing")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    left_stub = object_model.get_part("left_stub")
    right_stub = object_model.get_part("right_stub")

    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")
    left_stub_spin = object_model.get_articulation("left_stub_spin")
    right_stub_spin = object_model.get_articulation("right_stub_spin")

    ctx.expect_contact(
        de_dion_beam,
        carrier_frame,
        elem_a="left_pad",
        elem_b="left_support_pad",
        name="left beam pad is supported by the carrier frame",
    )
    ctx.expect_contact(
        de_dion_beam,
        carrier_frame,
        elem_a="right_pad",
        elem_b="right_support_pad",
        name="right beam pad is supported by the carrier frame",
    )
    ctx.expect_contact(
        differential_housing,
        carrier_frame,
        elem_a="top_mount_pad",
        elem_b="diff_hanger_pad",
        name="differential housing is mounted from the carrier frame",
    )
    ctx.expect_contact(
        left_hub,
        de_dion_beam,
        elem_a="inner_bearing",
        elem_b="left_carrier",
        name="left hub spins from the left beam carrier",
    )
    ctx.expect_contact(
        right_hub,
        de_dion_beam,
        elem_a="inner_bearing",
        elem_b="right_carrier",
        name="right hub spins from the right beam carrier",
    )
    ctx.expect_contact(
        left_stub,
        differential_housing,
        elem_a="inner_cv",
        elem_b="left_side_flange",
        name="left driveshaft stub meets the left differential flange",
    )
    ctx.expect_contact(
        right_stub,
        differential_housing,
        elem_a="inner_cv",
        elem_b="right_side_flange",
        name="right driveshaft stub meets the right differential flange",
    )
    ctx.expect_gap(
        differential_housing,
        de_dion_beam,
        axis="y",
        min_gap=0.02,
        positive_elem="pumpkin_shell",
        negative_elem="center_bow",
        name="De Dion bow clears the separate differential housing",
    )
    ctx.check(
        "all rotating members use the lateral axle axis",
        left_hub_spin.axis == (1.0, 0.0, 0.0)
        and right_hub_spin.axis == (1.0, 0.0, 0.0)
        and left_stub_spin.axis == (1.0, 0.0, 0.0)
        and right_stub_spin.axis == (1.0, 0.0, 0.0),
        details=(
            f"left_hub={left_hub_spin.axis}, right_hub={right_hub_spin.axis}, "
            f"left_stub={left_stub_spin.axis}, right_stub={right_stub_spin.axis}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
