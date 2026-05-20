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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_shell(
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    segments: int = 64,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _build_index_ring_mesh() -> MeshGeometry:
    return _ring_shell(
        outer_profile=[(0.090, 0.948), (0.090, 0.958)],
        inner_profile=[(0.058, 0.948), (0.058, 0.958)],
        segments=72,
    )


def _build_rotor_hub_mesh() -> MeshGeometry:
    hub_shell = _ring_shell(
        outer_profile=[
            (0.080, 0.000),
            (0.080, 0.014),
            (0.064, 0.024),
            (0.064, 0.086),
            (0.076, 0.096),
            (0.076, 0.110),
        ],
        inner_profile=[(0.032, 0.000), (0.032, 0.110)],
        segments=84,
    )

    datum_flat = (
        BoxGeometry((0.028, 0.034, 0.008))
        .translate(0.058, 0.0, 0.106)
    )
    return _merge_geometries([hub_shell, datum_flat])


def _center_of_aabb(aabb):
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_turnstile_gate")

    powder_gray = model.material("powder_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.18, 0.19, 0.20, 1.0))
    brass = model.material("brass", rgba=(0.63, 0.54, 0.31, 1.0))
    calibration_red = model.material("calibration_red", rgba=(0.74, 0.15, 0.13, 1.0))

    stator = model.part("stator")
    stator.visual(
        Box((0.420, 0.420, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_oxide,
        name="base_plate",
    )
    stator.visual(
        Cylinder(radius=0.070, length=0.920),
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        material=powder_gray,
        name="central_post",
    )
    stator.visual(
        Cylinder(radius=0.105, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.890)),
        material=powder_gray,
        name="bearing_housing",
    )
    stator.visual(
        mesh_from_geometry(_build_index_ring_mesh(), "index_ring"),
        material=dark_oxide,
        name="index_ring",
    )
    tick_geometry = Box((0.028, 0.010, 0.006))
    for tick_index in range(3):
        tick_angle = tick_index * (math.tau / 3.0)
        stator.visual(
            tick_geometry,
            origin=Origin(
                xyz=(0.076 * math.cos(tick_angle), 0.076 * math.sin(tick_angle), 0.957),
                rpy=(0.0, 0.0, tick_angle),
            ),
            material=calibration_red,
            name=f"index_tick_{tick_index}",
        )
    stator.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.956)),
        material=brass,
        name="thrust_shoulder",
    )
    stator.visual(
        Cylinder(radius=0.028, length=0.186),
        origin=Origin(xyz=(0.0, 0.0, 0.991)),
        material=stainless,
        name="supported_spindle",
    )
    stator.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 1.079)),
        material=dark_oxide,
        name="retainer_cap",
    )
    datum_x_geometry = Box((0.028, 0.080, 0.050))
    datum_y_geometry = Box((0.080, 0.028, 0.050))
    stator.visual(
        datum_x_geometry,
        origin=Origin(xyz=(0.102, 0.0, 0.898)),
        material=stainless,
        name="datum_flat_pos_x",
    )
    stator.visual(
        datum_x_geometry,
        origin=Origin(xyz=(-0.102, 0.0, 0.898)),
        material=stainless,
        name="datum_flat_neg_x",
    )
    stator.visual(
        datum_y_geometry,
        origin=Origin(xyz=(0.0, 0.102, 0.898)),
        material=stainless,
        name="datum_flat_pos_y",
    )
    stator.visual(
        datum_y_geometry,
        origin=Origin(xyz=(0.0, -0.102, 0.898)),
        material=stainless,
        name="datum_flat_neg_y",
    )
    stator.inertial = Inertial.from_geometry(
        Box((0.420, 0.420, 1.084)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.542)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(_build_rotor_hub_mesh(), "rotor_hub"),
        material=stainless,
        name="hub_cartridge",
    )
    clamp_geometry = Cylinder(radius=0.031, length=0.150)
    arm_geometry = Cylinder(radius=0.018, length=0.620)
    pad_geometry = Box((0.048, 0.024, 0.010))
    screw_geometry = Cylinder(radius=0.006, length=0.024)
    for arm_index in range(3):
        yaw = arm_index * (math.tau / 3.0)
        rotor.visual(
            clamp_geometry,
            origin=Origin(
                xyz=(0.132, 0.0, 0.056),
                rpy=(0.0, math.pi / 2.0, yaw),
            ),
            material=powder_gray,
            name=f"arm_{arm_index}_clamp",
        )
        rotor.visual(
            arm_geometry,
            origin=Origin(
                xyz=(0.390, 0.0, 0.056),
                rpy=(0.0, math.pi / 2.0, yaw),
            ),
            material=stainless,
            name=f"arm_{arm_index}_tube",
        )
        rotor.visual(
            pad_geometry,
            origin=Origin(
                xyz=(0.126, 0.0, 0.083),
                rpy=(0.0, 0.0, yaw),
            ),
            material=stainless,
            name=f"adjustment_pad_{arm_index}",
        )
        rotor.visual(
            screw_geometry,
            origin=Origin(
                xyz=(0.126, 0.0, 0.090),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_oxide,
            name=f"adjustment_screw_{arm_index}",
        )
    rotor.visual(
        Box((0.080, 0.010, 0.004)),
        origin=Origin(xyz=(0.110, 0.0, 0.004)),
        material=calibration_red,
        name="witness_pointer",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.700, length=0.110),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    model.articulation(
        "stator_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=stator,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.962)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stator = object_model.get_part("stator")
    rotor = object_model.get_part("rotor")
    rotation = object_model.get_articulation("stator_to_rotor")

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

    ctx.expect_contact(
        rotor,
        stator,
        elem_a="hub_cartridge",
        elem_b="thrust_shoulder",
        name="rotor_supported_by_thrust_shoulder",
    )
    ctx.expect_gap(
        stator,
        rotor,
        axis="z",
        positive_elem="retainer_cap",
        negative_elem="hub_cartridge",
        min_gap=0.0015,
        max_gap=0.0035,
        name="retainer_cap_controlled_gap",
    )
    ctx.expect_gap(
        rotor,
        stator,
        axis="z",
        positive_elem="witness_pointer",
        negative_elem="index_tick_0",
        min_gap=0.003,
        max_gap=0.007,
        name="pointer_clears_primary_index_mark",
    )
    ctx.expect_overlap(
        rotor,
        stator,
        axes="xy",
        elem_a="witness_pointer",
        elem_b="index_tick_0",
        min_overlap=0.008,
        name="pointer_aligns_to_home_index",
    )

    with ctx.pose({rotation: math.tau / 3.0}):
        ctx.expect_contact(
            rotor,
            stator,
            elem_a="hub_cartridge",
            elem_b="thrust_shoulder",
            name="rotor_support_persists_after_index_rotation",
        )
        ctx.expect_gap(
            rotor,
            stator,
            axis="z",
            positive_elem="witness_pointer",
            negative_elem="index_tick_1",
            min_gap=0.003,
            max_gap=0.007,
            name="pointer_clears_second_index_mark",
        )
        ctx.expect_overlap(
            rotor,
            stator,
            axes="xy",
            elem_a="witness_pointer",
            elem_b="index_tick_1",
            min_overlap=0.008,
            name="pointer_aligns_to_second_index",
        )

    with ctx.pose({rotation: 0.0}):
        arm_aabb_0 = ctx.part_element_world_aabb(rotor, elem="arm_0_tube")
    with ctx.pose({rotation: math.pi / 2.0}):
        arm_aabb_90 = ctx.part_element_world_aabb(rotor, elem="arm_0_tube")

    if arm_aabb_0 is None or arm_aabb_90 is None:
        ctx.fail("arm_rotation_measurement_available", "Could not resolve arm world AABBs")
    else:
        x0, y0, _ = _center_of_aabb(arm_aabb_0)
        x90, y90, _ = _center_of_aabb(arm_aabb_90)
        radius_0 = math.hypot(x0, y0)
        radius_90 = math.hypot(x90, y90)
        ctx.check(
            "arm_rotates_about_supported_spindle_axis",
            x0 > 0.32
            and abs(y0) < 0.04
            and y90 > 0.32
            and abs(x90) < 0.04
            and abs(radius_0 - radius_90) < 0.01,
            details=(
                f"arm centers were rest=({x0:.3f}, {y0:.3f}) and "
                f"quarter_turn=({x90:.3f}, {y90:.3f}); "
                f"radii=({radius_0:.3f}, {radius_90:.3f})"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
