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
)


def _radial_panel_origin(radius: float, angle_deg: float, z_center: float) -> Origin:
    angle = math.radians(angle_deg)
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z_center),
        rpy=(0.0, 0.0, angle + math.pi / 2.0),
    )


def _wing_origin(radius: float, yaw: float, z_center: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(yaw), radius * math.sin(yaw), z_center),
        rpy=(0.0, 0.0, yaw),
    )


def _annular_disc_mesh(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    name: str,
):
    ring = LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, thickness)],
        [(inner_radius, 0.0), (inner_radius, thickness)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(ring, name)


def _add_framed_wing(
    part,
    *,
    yaw: float,
    prefix: str,
    frame_material,
    panel_material,
) -> None:
    stile_z = 1.21

    part.visual(
        Box((0.07, 0.055, 2.14)),
        origin=_wing_origin(0.16, yaw, stile_z),
        material=frame_material,
        name=f"{prefix}_inner_stile",
    )
    part.visual(
        Box((0.09, 0.065, 2.14)),
        origin=_wing_origin(1.23, yaw, stile_z),
        material=frame_material,
        name=f"{prefix}_outer_stile",
    )
    part.visual(
        Box((1.05, 0.055, 0.07)),
        origin=_wing_origin(0.695, yaw, 0.175),
        material=frame_material,
        name=f"{prefix}_bottom_rail",
    )
    part.visual(
        Box((1.05, 0.055, 0.07)),
        origin=_wing_origin(0.695, yaw, 2.245),
        material=frame_material,
        name=f"{prefix}_top_rail",
    )
    part.visual(
        Box((1.00, 0.040, 2.00)),
        origin=_wing_origin(0.695, yaw, 1.23),
        material=panel_material,
        name=f"{prefix}_panel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cold_climate_revolving_door")

    aluminum = model.material("aluminum", rgba=(0.64, 0.66, 0.69, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.23, 0.25, 0.28, 1.0))
    canopy_metal = model.material("canopy_metal", rgba=(0.44, 0.46, 0.50, 1.0))
    threshold_metal = model.material("threshold_metal", rgba=(0.53, 0.54, 0.56, 1.0))
    insulated_panel = model.material("insulated_panel", rgba=(0.86, 0.88, 0.90, 1.0))
    vestibule_glass = model.material("vestibule_glass", rgba=(0.69, 0.80, 0.86, 0.28))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))

    drum_vestibule = model.part("drum_vestibule")
    drum_vestibule.visual(
        _annular_disc_mesh(1.62, 0.12, 0.05, "threshold_plate_ring"),
        origin=Origin(),
        material=threshold_metal,
        name="threshold_plate",
    )
    drum_vestibule.visual(
        _annular_disc_mesh(1.62, 0.12, 0.16, "canopy_shell_ring"),
        origin=Origin(xyz=(0.0, 0.0, 2.44)),
        material=canopy_metal,
        name="canopy_shell",
    )
    drum_vestibule.visual(
        _annular_disc_mesh(0.12, 0.04, 0.05, "floor_pivot_bearing"),
        origin=Origin(),
        material=dark_frame,
        name="floor_pivot_bearing",
    )
    drum_vestibule.visual(
        _annular_disc_mesh(1.50, 0.24, 0.06, "inner_header_ring"),
        origin=Origin(xyz=(0.0, 0.0, 2.38)),
        material=dark_frame,
        name="inner_header_ring",
    )
    drum_vestibule.visual(
        _annular_disc_mesh(0.12, 0.04, 0.16, "upper_pivot_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 2.44)),
        material=dark_frame,
        name="upper_pivot_bearing",
    )

    wall_angles = (45.0, 90.0, 135.0, 225.0, 270.0, 315.0)
    for angle in wall_angles:
        drum_vestibule.visual(
            Box((1.14, 0.045, 2.42)),
            origin=_radial_panel_origin(1.53, angle, 1.235),
            material=vestibule_glass,
            name=f"wall_panel_{int(angle):03d}",
        )
        drum_vestibule.visual(
            Box((1.18, 0.070, 0.080)),
            origin=_radial_panel_origin(1.53, angle, 0.095),
            material=dark_frame,
            name=f"base_frame_{int(angle):03d}",
        )
        drum_vestibule.visual(
            Box((1.18, 0.070, 0.090)),
            origin=_radial_panel_origin(1.53, angle, 2.355),
            material=dark_frame,
            name=f"top_frame_{int(angle):03d}",
        )

    for side_sign, side_name in ((1.0, "east"), (-1.0, "west")):
        for y_sign, edge_name in ((1.0, "north"), (-1.0, "south")):
            drum_vestibule.visual(
                Box((0.12, 0.16, 2.42)),
                origin=Origin(
                    xyz=(1.44 * side_sign, 0.60 * y_sign, 1.235),
                    rpy=(0.0, 0.0, 0.0),
                ),
                material=aluminum,
                name=f"{side_name}_{edge_name}_jamb",
            )
        drum_vestibule.visual(
            Box((0.16, 1.24, 0.14)),
            origin=Origin(xyz=(1.46 * side_sign, 0.0, 2.37)),
            material=aluminum,
            name=f"{side_name}_header",
        )
        drum_vestibule.visual(
            Box((0.18, 1.18, 0.03)),
            origin=Origin(xyz=(1.46 * side_sign, 0.0, 0.055)),
            material=gasket_black,
            name=f"{side_name}_threshold_sweep",
        )

    drum_vestibule.inertial = Inertial.from_geometry(
        Cylinder(radius=1.62, length=2.68),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
    )

    rotor_assembly = model.part("rotor_assembly")
    rotor_assembly.visual(
        Cylinder(radius=0.15, length=2.28),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=insulated_panel,
        name="central_post",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_frame,
        name="lower_rotor_hub",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.35)),
        material=dark_frame,
        name="upper_rotor_hub",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.03, length=2.34),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=aluminum,
        name="spindle_core",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=aluminum,
        name="lower_pivot_journal",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.41)),
        material=aluminum,
        name="upper_pivot_journal",
    )

    rotor_assembly.visual(
        Box((0.07, 0.055, 2.14)),
        origin=_wing_origin(0.16, 0.0, 1.21),
        material=dark_frame,
        name="east_wing_inner_stile",
    )
    rotor_assembly.visual(
        Box((0.09, 0.065, 2.14)),
        origin=_wing_origin(1.23, 0.0, 1.21),
        material=dark_frame,
        name="east_wing_outer_stile",
    )
    rotor_assembly.visual(
        Box((1.05, 0.055, 0.07)),
        origin=_wing_origin(0.695, 0.0, 0.175),
        material=dark_frame,
        name="east_wing_bottom_rail",
    )
    rotor_assembly.visual(
        Box((1.05, 0.055, 0.07)),
        origin=_wing_origin(0.695, 0.0, 2.245),
        material=dark_frame,
        name="east_wing_top_rail",
    )
    rotor_assembly.visual(
        Box((1.00, 0.040, 2.00)),
        origin=_wing_origin(0.695, 0.0, 1.23),
        material=insulated_panel,
        name="east_wing_panel",
    )
    _add_framed_wing(
        rotor_assembly,
        yaw=math.pi,
        prefix="west_wing",
        frame_material=dark_frame,
        panel_material=insulated_panel,
    )
    _add_framed_wing(
        rotor_assembly,
        yaw=math.pi / 2.0,
        prefix="north_wing",
        frame_material=dark_frame,
        panel_material=insulated_panel,
    )
    _add_framed_wing(
        rotor_assembly,
        yaw=-math.pi / 2.0,
        prefix="south_wing",
        frame_material=dark_frame,
        panel_material=insulated_panel,
    )

    rotor_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=1.40, length=2.34),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.17)),
    )

    model.articulation(
        "vestibule_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=drum_vestibule,
        child=rotor_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.2),
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

    drum_vestibule = object_model.get_part("drum_vestibule")
    rotor_assembly = object_model.get_part("rotor_assembly")
    spin = object_model.get_articulation("vestibule_to_rotor")
    east_wing_panel = rotor_assembly.get_visual("east_wing_panel")
    threshold_plate = drum_vestibule.get_visual("threshold_plate")
    canopy_shell = drum_vestibule.get_visual("canopy_shell")
    floor_pivot_bearing = drum_vestibule.get_visual("floor_pivot_bearing")
    upper_pivot_bearing = drum_vestibule.get_visual("upper_pivot_bearing")
    lower_pivot_journal = rotor_assembly.get_visual("lower_pivot_journal")
    upper_pivot_journal = rotor_assembly.get_visual("upper_pivot_journal")

    limits = spin.motion_limits
    ctx.check(
        "rotor articulation is continuous about vertical axis",
        spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={spin.joint_type}, axis={spin.axis}, "
            f"limits=({None if limits is None else limits.lower}, "
            f"{None if limits is None else limits.upper})"
        ),
    )
    ctx.expect_origin_distance(
        rotor_assembly,
        drum_vestibule,
        axes="xy",
        max_dist=0.001,
        name="rotor stays centered in drum",
    )
    ctx.expect_gap(
        rotor_assembly,
        drum_vestibule,
        axis="z",
        positive_elem=east_wing_panel,
        negative_elem=threshold_plate,
        min_gap=0.14,
        max_gap=0.18,
        name="wing panels clear threshold",
    )
    ctx.expect_gap(
        drum_vestibule,
        rotor_assembly,
        axis="z",
        positive_elem=canopy_shell,
        negative_elem=east_wing_panel,
        min_gap=0.16,
        max_gap=0.22,
        name="wing panels clear canopy",
    )
    ctx.expect_contact(
        rotor_assembly,
        drum_vestibule,
        elem_a=lower_pivot_journal,
        elem_b=floor_pivot_bearing,
        name="lower pivot journal seats on floor bearing",
    )
    ctx.expect_contact(
        rotor_assembly,
        drum_vestibule,
        elem_a=upper_pivot_journal,
        elem_b=upper_pivot_bearing,
        name="upper pivot journal seats on top bearing",
    )

    rest_pos = ctx.part_world_position(rotor_assembly)
    with ctx.pose({spin: math.pi / 4.0}):
        turned_pos = ctx.part_world_position(rotor_assembly)
        ctx.expect_gap(
            rotor_assembly,
            drum_vestibule,
            axis="z",
            positive_elem=east_wing_panel,
            negative_elem=threshold_plate,
            min_gap=0.14,
            max_gap=0.18,
            name="turned wing panels still clear threshold",
        )
        ctx.expect_gap(
            drum_vestibule,
            rotor_assembly,
            axis="z",
            positive_elem=canopy_shell,
            negative_elem=east_wing_panel,
            min_gap=0.16,
            max_gap=0.22,
            name="turned wing panels still clear canopy",
        )

    ctx.check(
        "rotor rotates without center drift",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
