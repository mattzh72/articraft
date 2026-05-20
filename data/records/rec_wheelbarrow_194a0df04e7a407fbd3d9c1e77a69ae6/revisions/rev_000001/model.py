from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def annulus_mesh(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    *,
    segments: int = 40,
) -> MeshGeometry:
    return ExtrudeWithHolesGeometry(
        circle_profile(outer_radius, segments=segments),
        [circle_profile(inner_radius, segments=segments)],
        thickness,
        center=True,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_wheelbarrow")

    model.material("powder_coat_blue", rgba=(0.18, 0.30, 0.56, 1.0))
    model.material("industrial_gray", rgba=(0.40, 0.42, 0.45, 1.0))
    model.material("zinc_plate", rgba=(0.75, 0.76, 0.78, 1.0))
    model.material("tire_black", rgba=(0.10, 0.10, 0.10, 1.0))
    model.material("datum_orange", rgba=(0.92, 0.48, 0.16, 1.0))

    handle_frame = model.part("handle_frame")
    tray = model.part("tray")
    rear_legs = model.part("rear_legs")
    fork_assembly = model.part("fork_assembly")
    front_wheel = model.part("front_wheel")

    # Root handle frame: tubular handles over a flat, calibration-friendly support rail stack.
    left_handle_geom = tube_from_spline_points(
        [
            (-0.50, 0.280, 0.565),
            (-0.30, 0.274, 0.525),
            (-0.06, 0.258, 0.470),
            (0.18, 0.246, 0.455),
            (0.52, 0.195, 0.410),
            (0.78, 0.140, 0.405),
            (0.90, 0.100, 0.430),
        ],
        radius=0.016,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )
    right_handle_geom = tube_from_spline_points(
        [
            (-0.50, -0.280, 0.565),
            (-0.30, -0.274, 0.525),
            (-0.06, -0.258, 0.470),
            (0.18, -0.246, 0.455),
            (0.52, -0.195, 0.410),
            (0.78, -0.140, 0.405),
            (0.90, -0.100, 0.430),
        ],
        radius=0.016,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )
    handle_frame.visual(
        mesh_from_geometry(left_handle_geom, "handle_frame_left_handle"),
        material="industrial_gray",
        name="left_handle",
    )
    handle_frame.visual(
        mesh_from_geometry(right_handle_geom, "handle_frame_right_handle"),
        material="industrial_gray",
        name="right_handle",
    )
    handle_frame.visual(
        Box((0.10, 0.56, 0.04)),
        origin=Origin(xyz=(-0.07, 0.0, 0.486)),
        material="industrial_gray",
        name="rear_brace",
    )
    handle_frame.visual(
        Box((0.12, 0.34, 0.035)),
        origin=Origin(xyz=(0.56, 0.0, 0.402)),
        material="industrial_gray",
        name="front_brace",
    )
    handle_frame.visual(
        Box((0.54, 0.22, 0.072)),
        origin=Origin(xyz=(0.28, 0.190, 0.419)),
        material="industrial_gray",
        name="left_support_rail",
    )
    handle_frame.visual(
        Box((0.54, 0.22, 0.072)),
        origin=Origin(xyz=(0.28, -0.190, 0.419)),
        material="industrial_gray",
        name="right_support_rail",
    )
    for name, xyz in (
        ("rear_support_left", (0.14, 0.190, 0.450)),
        ("rear_support_right", (0.14, -0.190, 0.450)),
        ("mid_support_left", (0.54, 0.110, 0.450)),
        ("mid_support_right", (0.54, -0.110, 0.450)),
    ):
        handle_frame.visual(
            Box((0.08, 0.05, 0.010)),
            origin=Origin(xyz=xyz),
            material="zinc_plate",
            name=name,
        )
    handle_frame.visual(
        Box((0.18, 0.042, 0.010)),
        origin=Origin(xyz=(-0.34, 0.275, 0.573)),
        material="datum_orange",
        name="left_datum_pad",
    )
    handle_frame.visual(
        Box((0.18, 0.042, 0.010)),
        origin=Origin(xyz=(-0.34, -0.275, 0.573)),
        material="datum_orange",
        name="right_datum_pad",
    )
    handle_frame.visual(
        Box((0.18, 0.014, 0.020)),
        origin=Origin(xyz=(-0.34, 0.275, 0.563)),
        material="industrial_gray",
        name="left_datum_bridge",
    )
    handle_frame.visual(
        Box((0.18, 0.014, 0.020)),
        origin=Origin(xyz=(-0.34, -0.275, 0.563)),
        material="industrial_gray",
        name="right_datum_bridge",
    )
    for i, x in enumerate((-0.38, -0.34, -0.30), start=1):
        handle_frame.visual(
            Box((0.006, 0.018, 0.003)),
            origin=Origin(xyz=(x, 0.275, 0.5795)),
            material="zinc_plate",
            name=f"left_handle_index_{i}",
        )
    handle_frame.visual(
        Box((0.10, 0.010, 0.010)),
        origin=Origin(xyz=(-0.34, 0.275, 0.574)),
        material="industrial_gray",
        name="left_index_strip",
    )
    handle_frame.visual(
        Box((0.05, 0.16, 0.04)),
        origin=Origin(xyz=(0.865, 0.0, 0.430)),
        material="zinc_plate",
        name="fork_interface_plate",
    )
    handle_frame.visual(
        Box((0.22, 0.08, 0.04)),
        origin=Origin(xyz=(0.73, 0.0, 0.430)),
        material="zinc_plate",
        name="fork_interface_spine",
    )

    # Tray: open pan with mounting blocks, rim datum pads, and rear index marks.
    tray.visual(
        Box((0.62, 0.46, 0.006)),
        origin=Origin(xyz=(0.34, 0.0, 0.490)),
        material="powder_coat_blue",
        name="tray_floor",
    )
    tray.visual(
        Box((0.26, 0.28, 0.006)),
        origin=Origin(xyz=(0.74, 0.0, 0.490)),
        material="powder_coat_blue",
        name="tray_front_floor",
    )
    tray.visual(
        Box((0.006, 0.46, 0.12)),
        origin=Origin(xyz=(0.03, 0.0, 0.550)),
        material="powder_coat_blue",
        name="tray_rear_wall",
    )
    tray.visual(
        Box((0.52, 0.008, 0.14)),
        origin=Origin(xyz=(0.32, 0.228, 0.563)),
        material="powder_coat_blue",
        name="left_side_rear",
    )
    tray.visual(
        Box((0.52, 0.008, 0.14)),
        origin=Origin(xyz=(0.32, -0.228, 0.563)),
        material="powder_coat_blue",
        name="right_side_rear",
    )
    tray.visual(
        Box((0.32, 0.008, 0.16)),
        origin=Origin(xyz=(0.72, 0.228, 0.563)),
        material="powder_coat_blue",
        name="left_side_front",
    )
    tray.visual(
        Box((0.32, 0.008, 0.16)),
        origin=Origin(xyz=(0.72, -0.228, 0.563)),
        material="powder_coat_blue",
        name="right_side_front",
    )
    tray.visual(
        Box((0.006, 0.46, 0.16)),
        origin=Origin(xyz=(0.86, 0.0, 0.590)),
        material="powder_coat_blue",
        name="nose_wall",
    )
    for name, xyz in (
        ("rear_mount_left", (0.14, 0.190, 0.471)),
        ("rear_mount_right", (0.14, -0.190, 0.471)),
        ("mid_mount_left", (0.54, 0.110, 0.471)),
        ("mid_mount_right", (0.54, -0.110, 0.471)),
    ):
        tray.visual(
            Box((0.05, 0.05, 0.032) if "rear" in name else (0.08, 0.05, 0.032)),
            origin=Origin(xyz=xyz),
            material="zinc_plate",
            name=name,
        )
    for name, xyz, size in (
        ("rear_mount_left_riser", (0.14, 0.190, 0.475), (0.040, 0.040, 0.024)),
        ("rear_mount_right_riser", (0.14, -0.190, 0.475), (0.040, 0.040, 0.024)),
        ("mid_mount_left_riser", (0.54, 0.110, 0.475), (0.050, 0.040, 0.024)),
        ("mid_mount_right_riser", (0.54, -0.110, 0.475), (0.050, 0.040, 0.024)),
    ):
        tray.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material="zinc_plate",
            name=name,
        )
    tray.visual(
        Box((0.10, 0.06, 0.010)),
        origin=Origin(xyz=(0.00, 0.0, 0.615)),
        material="datum_orange",
        name="rear_datum_pad",
    )
    tray.visual(
        Box((0.14, 0.04, 0.010)),
        origin=Origin(xyz=(0.35, 0.230, 0.6375)),
        material="datum_orange",
        name="left_rim_datum",
    )
    tray.visual(
        Box((0.14, 0.04, 0.010)),
        origin=Origin(xyz=(0.35, -0.230, 0.6375)),
        material="datum_orange",
        name="right_rim_datum",
    )
    for i, x in enumerate((-0.03, 0.00, 0.03), start=1):
        tray.visual(
            Box((0.006, 0.020, 0.002)),
            origin=Origin(xyz=(x, 0.0, 0.621)),
            material="zinc_plate",
            name=f"rear_index_mark_{i}",
        )

    # Rear stance: two legs tied together with a cross spreader and foot pads.
    left_leg_geom = tube_from_spline_points(
        [(0.14, 0.190, 0.348), (0.08, 0.215, 0.215), (0.01, 0.240, 0.025)],
        radius=0.015,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    right_leg_geom = tube_from_spline_points(
        [(0.14, -0.190, 0.348), (0.08, -0.215, 0.215), (0.01, -0.240, 0.025)],
        radius=0.015,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    rear_spreader_geom = tube_from_spline_points(
        [(0.03, -0.220, 0.040), (0.03, 0.220, 0.040)],
        radius=0.010,
        radial_segments=16,
        cap_ends=True,
    )
    rear_legs.visual(
        mesh_from_geometry(left_leg_geom, "rear_legs_left_leg"),
        material="industrial_gray",
        name="left_leg",
    )
    rear_legs.visual(
        mesh_from_geometry(right_leg_geom, "rear_legs_right_leg"),
        material="industrial_gray",
        name="right_leg",
    )
    rear_legs.visual(
        mesh_from_geometry(rear_spreader_geom, "rear_legs_spreader"),
        material="industrial_gray",
        name="rear_spreader",
    )
    rear_legs.visual(
        Box((0.10, 0.05, 0.02)),
        origin=Origin(xyz=(0.14, 0.190, 0.373)),
        material="zinc_plate",
        name="left_mount",
    )
    rear_legs.visual(
        Box((0.10, 0.05, 0.02)),
        origin=Origin(xyz=(0.14, -0.190, 0.373)),
        material="zinc_plate",
        name="right_mount",
    )
    rear_legs.visual(
        Box((0.040, 0.040, 0.030)),
        origin=Origin(xyz=(0.13, 0.190, 0.368)),
        material="industrial_gray",
        name="left_mount_riser",
    )
    rear_legs.visual(
        Box((0.040, 0.040, 0.030)),
        origin=Origin(xyz=(0.13, -0.190, 0.368)),
        material="industrial_gray",
        name="right_mount_riser",
    )
    rear_legs.visual(
        Box((0.10, 0.05, 0.010)),
        origin=Origin(xyz=(0.01, 0.220, 0.005)),
        material="datum_orange",
        name="left_foot_pad",
    )
    rear_legs.visual(
        Box((0.10, 0.05, 0.010)),
        origin=Origin(xyz=(0.01, -0.220, 0.005)),
        material="datum_orange",
        name="right_foot_pad",
    )
    rear_legs.visual(
        Box((0.03, 0.03, 0.035)),
        origin=Origin(xyz=(0.01, 0.220, 0.0225)),
        material="industrial_gray",
        name="left_foot_stem",
    )
    rear_legs.visual(
        Box((0.03, 0.03, 0.035)),
        origin=Origin(xyz=(0.01, -0.220, 0.0225)),
        material="industrial_gray",
        name="right_foot_stem",
    )

    # Front fork assembly: crown block, tubular arms, slotted dropouts, axle and index marks.
    left_arm_geom = tube_from_spline_points(
        [(0.920, 0.080, 0.455), (0.965, 0.074, 0.330), (1.000, 0.066, 0.215)],
        radius=0.012,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    right_arm_geom = tube_from_spline_points(
        [(0.920, -0.080, 0.455), (0.965, -0.074, 0.330), (1.000, -0.066, 0.215)],
        radius=0.012,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    dropout_outer = [(-0.035, -0.040), (0.035, -0.040), (0.035, 0.040), (-0.035, 0.040)]
    dropout_slot = [(-0.018, -0.008), (0.018, -0.008), (0.018, 0.008), (-0.018, 0.008)]
    left_dropout_geom = ExtrudeWithHolesGeometry(
        dropout_outer, [dropout_slot], 0.008, center=True, closed=True
    )
    left_dropout_geom.rotate_x(math.pi / 2.0).translate(1.025, 0.072, 0.175)
    right_dropout_geom = ExtrudeWithHolesGeometry(
        dropout_outer, [dropout_slot], 0.008, center=True, closed=True
    )
    right_dropout_geom.rotate_x(math.pi / 2.0).translate(1.025, -0.072, 0.175)
    # The axle is modeled as a solid-looking shaft by using a very small internal bore
    # rather than a mathematically solid mesh, which keeps the mesh authoring path simple.
    axle_geom = annulus_mesh(0.0085, 0.0008, 0.136, segments=40)
    axle_geom.rotate_x(math.pi / 2.0).translate(1.025, 0.0, 0.175)
    left_washer_geom = annulus_mesh(0.024, 0.0075, 0.004, segments=32)
    left_washer_geom.rotate_x(math.pi / 2.0).translate(1.025, 0.038, 0.175)
    right_washer_geom = annulus_mesh(0.024, 0.0075, 0.004, segments=32)
    right_washer_geom.rotate_x(math.pi / 2.0).translate(1.025, -0.038, 0.175)
    fork_assembly.visual(
        Box((0.05, 0.14, 0.05)),
        origin=Origin(xyz=(0.915, 0.0, 0.430)),
        material="zinc_plate",
        name="crown_block",
    )
    fork_assembly.visual(
        Box((0.04, 0.08, 0.006)),
        origin=Origin(xyz=(0.915, 0.0, 0.454)),
        material="datum_orange",
        name="crown_datum",
    )
    fork_assembly.visual(
        mesh_from_geometry(left_arm_geom, "fork_left_arm"),
        material="industrial_gray",
        name="left_arm",
    )
    fork_assembly.visual(
        mesh_from_geometry(right_arm_geom, "fork_right_arm"),
        material="industrial_gray",
        name="right_arm",
    )
    fork_assembly.visual(
        mesh_from_geometry(left_dropout_geom, "fork_left_dropout"),
        material="zinc_plate",
        name="left_dropout",
    )
    fork_assembly.visual(
        mesh_from_geometry(right_dropout_geom, "fork_right_dropout"),
        material="zinc_plate",
        name="right_dropout",
    )
    fork_assembly.visual(
        mesh_from_geometry(axle_geom, "fork_axle"),
        material="zinc_plate",
        name="axle",
    )
    fork_assembly.visual(
        mesh_from_geometry(left_washer_geom, "fork_left_thrust_washer"),
        material="zinc_plate",
        name="left_thrust_washer",
    )
    fork_assembly.visual(
        mesh_from_geometry(right_washer_geom, "fork_right_thrust_washer"),
        material="zinc_plate",
        name="right_thrust_washer",
    )
    fork_assembly.visual(
        Box((0.018, 0.016, 0.012)),
        origin=Origin(xyz=(1.000, 0.084, 0.177)),
        material="datum_orange",
        name="left_adjuster_block",
    )
    fork_assembly.visual(
        Box((0.018, 0.016, 0.012)),
        origin=Origin(xyz=(1.000, -0.084, 0.177)),
        material="datum_orange",
        name="right_adjuster_block",
    )
    for side, y in (("left", 0.077), ("right", -0.077)):
        for i, x in enumerate((0.995, 1.010, 1.025, 1.040), start=1):
            fork_assembly.visual(
                Box((0.004, 0.002, 0.016)),
                origin=Origin(xyz=(x, y, 0.195)),
                material="industrial_gray",
                name=f"{side}_dropout_mark_{i}",
            )

    # Front wheel centered on the axle frame with explicit bore clearance.
    tire_geom = TorusGeometry(0.145, 0.030, radial_segments=24, tubular_segments=48)
    tire_geom.rotate_x(math.pi / 2.0)
    rim_geom = annulus_mesh(0.118, 0.088, 0.065, segments=48)
    rim_geom.rotate_x(math.pi / 2.0)
    disc_left_geom = annulus_mesh(0.090, 0.042, 0.006, segments=48)
    disc_left_geom.rotate_x(math.pi / 2.0).translate(0.0, 0.018, 0.0)
    disc_right_geom = annulus_mesh(0.090, 0.042, 0.006, segments=48)
    disc_right_geom.rotate_x(math.pi / 2.0).translate(0.0, -0.018, 0.0)
    hub_geom = annulus_mesh(0.045, 0.0115, 0.072, segments=40)
    hub_geom.rotate_x(math.pi / 2.0)
    front_wheel.visual(
        mesh_from_geometry(tire_geom, "front_wheel_tire"),
        material="tire_black",
        name="tire",
    )
    front_wheel.visual(
        mesh_from_geometry(rim_geom, "front_wheel_rim"),
        material="powder_coat_blue",
        name="rim",
    )
    front_wheel.visual(
        mesh_from_geometry(disc_left_geom, "front_wheel_disc_left"),
        material="zinc_plate",
        name="disc_left",
    )
    front_wheel.visual(
        mesh_from_geometry(disc_right_geom, "front_wheel_disc_right"),
        material="zinc_plate",
        name="disc_right",
    )
    front_wheel.visual(
        mesh_from_geometry(hub_geom, "front_wheel_hub"),
        material="industrial_gray",
        name="hub",
    )

    model.articulation(
        "handle_frame_to_tray",
        ArticulationType.FIXED,
        parent=handle_frame,
        child=tray,
        origin=Origin(),
    )
    model.articulation(
        "handle_frame_to_rear_legs",
        ArticulationType.FIXED,
        parent=handle_frame,
        child=rear_legs,
        origin=Origin(),
    )
    model.articulation(
        "handle_frame_to_fork",
        ArticulationType.FIXED,
        parent=handle_frame,
        child=fork_assembly,
        origin=Origin(),
    )
    model.articulation(
        "fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork_assembly,
        child=front_wheel,
        origin=Origin(xyz=(1.025, 0.0, 0.175)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_frame = object_model.get_part("handle_frame")
    tray = object_model.get_part("tray")
    rear_legs = object_model.get_part("rear_legs")
    fork_assembly = object_model.get_part("fork_assembly")
    front_wheel = object_model.get_part("front_wheel")
    wheel_joint = object_model.get_articulation("fork_to_front_wheel")

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

    ctx.check(
        "wheel_joint_is_continuous_lateral_spin",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0)
        and wheel_joint.motion_limits is not None
        and wheel_joint.motion_limits.lower is None
        and wheel_joint.motion_limits.upper is None,
        details=(
            f"type={wheel_joint.articulation_type}, "
            f"axis={wheel_joint.axis}, limits={wheel_joint.motion_limits}"
        ),
    )

    ctx.expect_contact(
        fork_assembly,
        handle_frame,
        elem_a="crown_block",
        elem_b="fork_interface_plate",
        name="fork_crown_bolts_to_handle_frame",
    )
    ctx.expect_gap(
        tray,
        handle_frame,
        axis="z",
        positive_elem="rear_mount_left",
        negative_elem="rear_support_left",
        max_penetration=1e-6,
        max_gap=0.001,
        name="tray_left_rear_mount_seat",
    )
    ctx.expect_gap(
        tray,
        handle_frame,
        axis="z",
        positive_elem="mid_mount_right",
        negative_elem="mid_support_right",
        max_penetration=1e-6,
        max_gap=0.001,
        name="tray_right_mid_mount_seat",
    )
    ctx.expect_contact(
        handle_frame,
        rear_legs,
        elem_a="left_support_rail",
        elem_b="left_mount",
        name="left_leg_mount_seat",
    )
    ctx.expect_contact(
        handle_frame,
        rear_legs,
        elem_a="right_support_rail",
        elem_b="right_mount",
        name="right_leg_mount_seat",
    )
    ctx.expect_gap(
        fork_assembly,
        front_wheel,
        axis="y",
        positive_elem="left_dropout",
        negative_elem="tire",
        min_gap=0.020,
        max_gap=0.040,
        name="left_dropout_to_tire_gap",
    )
    ctx.expect_gap(
        front_wheel,
        fork_assembly,
        axis="y",
        positive_elem="tire",
        negative_elem="right_dropout",
        min_gap=0.020,
        max_gap=0.040,
        name="right_dropout_to_tire_gap",
    )
    ctx.expect_contact(
        fork_assembly,
        front_wheel,
        elem_a="left_thrust_washer",
        elem_b="hub",
        name="left_hub_face_bears_on_thrust_washer",
    )
    ctx.expect_contact(
        fork_assembly,
        front_wheel,
        elem_a="right_thrust_washer",
        elem_b="hub",
        name="right_hub_face_bears_on_thrust_washer",
    )
    ctx.expect_gap(
        tray,
        front_wheel,
        axis="z",
        positive_elem="tray_floor",
        negative_elem="tire",
        min_gap=0.100,
        max_gap=0.140,
        name="tray_clears_front_wheel",
    )
    ctx.expect_overlap(
        tray,
        handle_frame,
        axes="xy",
        min_overlap=0.20,
        name="tray_is_centered_over_frame_span",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
