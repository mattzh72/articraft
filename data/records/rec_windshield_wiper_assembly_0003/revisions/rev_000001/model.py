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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _aero_member_mesh(
    name: str,
    path_points: list[tuple[float, float, float]],
    *,
    width: float,
    thickness: float,
    samples_per_segment: int = 14,
):
    return _save_mesh(
        name,
        sweep_profile_along_spline(
            path_points,
            profile=rounded_rect_profile(
                width,
                thickness,
                radius=min(width, thickness) * 0.30,
                corner_segments=6,
            ),
            samples_per_segment=samples_per_segment,
            cap_profile=True,
        ),
    )


def _tube_member_mesh(
    name: str,
    path_points: list[tuple[float, float, float]],
    *,
    radius: float,
    corner_radius: float = 0.0,
):
    return _save_mesh(
        name,
        wire_from_points(
            path_points,
            radius=radius,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=corner_radius,
            corner_segments=8,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_windshield_wiper_assembly", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.58, 0.61, 0.65, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    cowl_module = model.part("cowl_module")
    cowl_module.visual(
        Box((1.06, 0.045, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=satin_graphite,
        name="main_beam",
    )
    cowl_module.visual(
        Box((0.96, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.025, -0.050)),
        material=anodized_aluminum,
        name="front_cap",
    )
    cowl_module.visual(
        Box((0.92, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.032, -0.054)),
        material=anodized_aluminum,
        name="rear_interface_strip",
    )
    cowl_module.visual(
        Box((0.66, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=satin_steel,
        name="upper_trim",
    )
    cowl_module.visual(
        Cylinder(radius=0.010, length=0.92),
        origin=Origin(xyz=(0.0, -0.020, -0.062), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="rear_tube",
    )
    cowl_module.visual(
        Box((0.18, 0.046, 0.008)),
        origin=Origin(xyz=(-0.120, -0.060, -0.059)),
        material=satin_graphite,
        name="motor_shelf",
    )
    cowl_module.visual(
        Box((0.024, 0.030, 0.030)),
        origin=Origin(xyz=(-0.195, -0.040, -0.060)),
        material=matte_black,
        name="motor_web",
    )
    cowl_module.visual(
        Box((0.20, 0.014, 0.022)),
        origin=Origin(xyz=(0.160, -0.041, -0.055)),
        material=matte_black,
        name="right_rear_brace",
    )
    for side_name, side_x in (("left", -0.31), ("right", 0.31)):
        cowl_module.visual(
            Box((0.078, 0.050, 0.020)),
            origin=Origin(xyz=(side_x, 0.0, -0.043)),
            material=matte_black,
            name=f"{side_name}_pedestal_base",
        )
        cowl_module.visual(
            Box((0.050, 0.022, 0.004)),
            origin=Origin(xyz=(side_x, 0.0, -0.031)),
            material=anodized_aluminum,
            name=f"{side_name}_escutcheon",
        )
    cowl_module.inertial = Inertial.from_geometry(
        Box((1.08, 0.16, 0.12)),
        mass=7.5,
        origin=Origin(xyz=(0.0, -0.010, -0.030)),
    )

    motor_pack = model.part("motor_pack")
    motor_pack.visual(
        Box((0.17, 0.046, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=matte_black,
        name="mount_plate",
    )
    motor_pack.visual(
        Box((0.090, 0.046, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=satin_graphite,
        name="gearbox_housing",
    )
    motor_pack.visual(
        Box((0.100, 0.054, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=anodized_aluminum,
        name="gearbox_cover",
    )
    motor_pack.visual(
        Box((0.034, 0.066, 0.016)),
        origin=Origin(xyz=(0.040, -0.038, 0.019)),
        material=satin_graphite,
        name="can_bridge",
    )
    motor_pack.visual(
        Cylinder(radius=0.029, length=0.100),
        origin=Origin(xyz=(0.078, -0.072, 0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="motor_can",
    )
    motor_pack.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.131, -0.072, 0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_aluminum,
        name="end_cap",
    )
    motor_pack.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=satin_steel,
        name="output_boss",
    )
    motor_pack.visual(
        Box((0.026, 0.018, 0.014)),
        origin=Origin(xyz=(0.122, -0.078, 0.026)),
        material=matte_black,
        name="connector",
    )
    motor_pack.inertial = Inertial.from_geometry(
        Box((0.19, 0.08, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(0.045, 0.0, 0.020)),
    )

    drive_crank = model.part("drive_crank")
    drive_crank.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=satin_steel,
        name="hub",
    )
    drive_crank.visual(
        _aero_member_mesh(
            "drive_crank_arm.obj",
            [(0.0, 0.0, 0.002), (-0.015, 0.0, 0.002), (-0.032, 0.010, 0.002)],
            width=0.012,
            thickness=0.004,
            samples_per_segment=8,
        ),
        material=satin_graphite,
        name="crank_arm",
    )
    drive_crank.visual(
        Box((0.016, 0.009, 0.004)),
        origin=Origin(xyz=(0.015, -0.008, 0.002)),
        material=anodized_aluminum,
        name="counterweight",
    )
    drive_crank.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(-0.032, 0.010, 0.004)),
        material=satin_steel,
        name="primary_stud",
    )
    drive_crank.inertial = Inertial.from_geometry(
        Box((0.06, 0.03, 0.012)),
        mass=0.16,
        origin=Origin(xyz=(-0.010, 0.002, 0.003)),
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=satin_steel,
        name="proximal_bushing",
    )
    primary_link.visual(
        _aero_member_mesh(
            "primary_link_bar.obj",
            [(0.0, 0.0, 0.002), (-0.040, 0.010, 0.002), (-0.102, 0.026, 0.003)],
            width=0.012,
            thickness=0.004,
            samples_per_segment=10,
        ),
        material=anodized_aluminum,
        name="stamped_bar",
    )
    primary_link.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(-0.102, 0.026, 0.005)),
        material=satin_steel,
        name="distal_eye",
    )
    primary_link.inertial = Inertial.from_geometry(
        Box((0.11, 0.04, 0.012)),
        mass=0.12,
        origin=Origin(xyz=(-0.050, 0.013, 0.004)),
    )

    left_wiper_module = model.part("left_wiper_module")
    left_wiper_module.visual(
        Cylinder(radius=0.007, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=satin_steel,
        name="spindle_post",
    )
    left_wiper_module.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=satin_steel,
        name="lower_hub",
    )
    left_wiper_module.visual(
        Cylinder(radius=0.011, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=satin_steel,
        name="mount_collar",
    )
    left_wiper_module.visual(
        Box((0.076, 0.012, 0.008)),
        origin=Origin(xyz=(0.024, -0.026, -0.010)),
        material=satin_graphite,
        name="bellcrank",
    )
    left_wiper_module.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.059, -0.032, -0.010)),
        material=satin_steel,
        name="relay_socket",
    )
    left_wiper_module.visual(
        Box((0.064, 0.008, 0.006)),
        origin=Origin(xyz=(0.030, -0.016, -0.010)),
        material=satin_graphite,
        name="drag_link_bar",
    )
    left_wiper_module.visual(
        Box((0.032, 0.010, 0.006)),
        origin=Origin(xyz=(0.010, -0.012, -0.010)),
        material=satin_graphite,
        name="bellcrank_root",
    )
    left_wiper_module.visual(
        Box((0.014, 0.014, 0.006)),
        origin=Origin(xyz=(0.056, -0.024, -0.010)),
        material=satin_graphite,
        name="socket_stem",
    )
    left_wiper_module.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=satin_steel,
        name="upper_spindle",
    )
    left_wiper_module.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=anodized_aluminum,
        name="spindle_cap",
    )
    left_wiper_module.visual(
        _aero_member_mesh(
            "left_wiper_arm.obj",
            [(0.0, 0.0, 0.048), (-0.050, 0.020, 0.055), (-0.130, 0.055, 0.061), (-0.250, 0.090, 0.066)],
            width=0.018,
            thickness=0.006,
            samples_per_segment=14,
        ),
        material=satin_graphite,
        name="sweep_arm",
    )
    left_wiper_module.visual(
        _tube_member_mesh(
            "left_wiper_spine.obj",
            [(-0.015, 0.006, 0.050), (-0.090, 0.036, 0.057), (-0.180, 0.068, 0.062)],
            radius=0.0028,
            corner_radius=0.014,
        ),
        material=anodized_aluminum,
        name="spring_spine",
    )
    left_wiper_module.visual(
        Box((0.028, 0.018, 0.008)),
        origin=Origin(xyz=(-0.250, 0.090, 0.063)),
        material=matte_black,
        name="blade_pad",
    )
    left_wiper_module.inertial = Inertial.from_geometry(
        Box((0.30, 0.14, 0.10)),
        mass=0.58,
        origin=Origin(xyz=(-0.120, 0.040, 0.020)),
    )

    right_wiper_module = model.part("right_wiper_module")
    right_wiper_module.visual(
        Cylinder(radius=0.007, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=satin_steel,
        name="spindle_post",
    )
    right_wiper_module.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=satin_steel,
        name="lower_hub",
    )
    right_wiper_module.visual(
        Cylinder(radius=0.011, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=satin_steel,
        name="mount_collar",
    )
    right_wiper_module.visual(
        Box((0.074, 0.012, 0.008)),
        origin=Origin(xyz=(-0.024, -0.026, -0.010)),
        material=satin_graphite,
        name="bellcrank",
    )
    right_wiper_module.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(-0.059, -0.032, -0.010)),
        material=satin_steel,
        name="relay_socket",
    )
    right_wiper_module.visual(
        Box((0.064, 0.008, 0.006)),
        origin=Origin(xyz=(-0.030, -0.016, -0.010)),
        material=satin_graphite,
        name="drag_link_bar",
    )
    right_wiper_module.visual(
        Box((0.032, 0.010, 0.006)),
        origin=Origin(xyz=(-0.010, -0.012, -0.010)),
        material=satin_graphite,
        name="bellcrank_root",
    )
    right_wiper_module.visual(
        Box((0.014, 0.014, 0.006)),
        origin=Origin(xyz=(-0.056, -0.024, -0.010)),
        material=satin_graphite,
        name="socket_stem",
    )
    right_wiper_module.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=satin_steel,
        name="upper_spindle",
    )
    right_wiper_module.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=anodized_aluminum,
        name="spindle_cap",
    )
    right_wiper_module.visual(
        _aero_member_mesh(
            "right_wiper_arm.obj",
            [(0.0, 0.0, 0.046), (0.040, 0.018, 0.052), (0.110, 0.045, 0.057), (0.220, 0.072, 0.061)],
            width=0.017,
            thickness=0.006,
            samples_per_segment=14,
        ),
        material=satin_graphite,
        name="sweep_arm",
    )
    right_wiper_module.visual(
        _tube_member_mesh(
            "right_wiper_spine.obj",
            [(0.012, 0.006, 0.048), (0.072, 0.030, 0.054), (0.152, 0.054, 0.058)],
            radius=0.0026,
            corner_radius=0.014,
        ),
        material=anodized_aluminum,
        name="spring_spine",
    )
    right_wiper_module.visual(
        Box((0.026, 0.018, 0.008)),
        origin=Origin(xyz=(0.220, 0.072, 0.058)),
        material=matte_black,
        name="blade_pad",
    )
    right_wiper_module.inertial = Inertial.from_geometry(
        Box((0.26, 0.12, 0.10)),
        mass=0.50,
        origin=Origin(xyz=(0.100, 0.032, 0.018)),
    )

    cross_link = model.part("cross_link")
    cross_link.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_steel,
        name="proximal_bushing",
    )
    cross_link.visual(
        Cylinder(radius=0.0038, length=0.504),
        origin=Origin(xyz=(0.252, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_aluminum,
        name="tie_rod",
    )
    cross_link.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.502, 0.0, 0.004)),
        material=satin_steel,
        name="distal_eye",
    )
    cross_link.inertial = Inertial.from_geometry(
        Box((0.53, 0.02, 0.012)),
        mass=0.18,
        origin=Origin(xyz=(0.252, 0.0, 0.004)),
    )

    left_blade_carrier = model.part("left_blade_carrier")
    left_blade_carrier.visual(
        Box((0.026, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=matte_black,
        name="adapter_bridge",
    )
    left_blade_carrier.visual(
        _aero_member_mesh(
            "left_blade_shell.obj",
            [(-0.38, -0.004, -0.010), (-0.20, -0.002, -0.009), (0.0, 0.0, -0.009), (0.12, 0.002, -0.009)],
            width=0.020,
            thickness=0.006,
            samples_per_segment=12,
        ),
        material=satin_graphite,
        name="blade_shell",
    )
    left_blade_carrier.visual(
        Box((0.30, 0.010, 0.004)),
        origin=Origin(xyz=(-0.12, 0.002, -0.006)),
        material=anodized_aluminum,
        name="spoiler",
    )
    left_blade_carrier.visual(
        Box((0.50, 0.003, 0.010)),
        origin=Origin(xyz=(-0.13, 0.0, -0.014)),
        material=rubber,
        name="squeegee",
    )
    left_blade_carrier.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(-0.38, -0.004, -0.010)),
        material=matte_black,
        name="left_end_cap",
    )
    left_blade_carrier.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(0.12, 0.002, -0.009)),
        material=matte_black,
        name="right_end_cap",
    )
    left_blade_carrier.inertial = Inertial.from_geometry(
        Box((0.52, 0.024, 0.022)),
        mass=0.28,
        origin=Origin(xyz=(-0.13, 0.0, -0.010)),
    )

    right_blade_carrier = model.part("right_blade_carrier")
    right_blade_carrier.visual(
        Box((0.024, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=matte_black,
        name="adapter_bridge",
    )
    right_blade_carrier.visual(
        _aero_member_mesh(
            "right_blade_shell.obj",
            [(0.0, 0.0, -0.009), (0.12, 0.002, -0.009), (0.28, 0.004, -0.010), (0.40, 0.005, -0.010)],
            width=0.018,
            thickness=0.006,
            samples_per_segment=12,
        ),
        material=satin_graphite,
        name="blade_shell",
    )
    right_blade_carrier.visual(
        Box((0.26, 0.010, 0.004)),
        origin=Origin(xyz=(0.18, 0.002, -0.006)),
        material=anodized_aluminum,
        name="spoiler",
    )
    right_blade_carrier.visual(
        Box((0.40, 0.003, 0.010)),
        origin=Origin(xyz=(0.18, 0.0, -0.014)),
        material=rubber,
        name="squeegee",
    )
    right_blade_carrier.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=matte_black,
        name="left_end_cap",
    )
    right_blade_carrier.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(0.40, 0.005, -0.010)),
        material=matte_black,
        name="right_end_cap",
    )
    right_blade_carrier.inertial = Inertial.from_geometry(
        Box((0.42, 0.024, 0.022)),
        mass=0.24,
        origin=Origin(xyz=(0.12, 0.0, -0.010)),
    )

    model.articulation(
        "cowl_to_motor_pack",
        ArticulationType.FIXED,
        parent=cowl_module,
        child=motor_pack,
        origin=Origin(xyz=(-0.120, -0.075, -0.055)),
    )
    model.articulation(
        "motor_drive",
        ArticulationType.CONTINUOUS,
        parent=motor_pack,
        child=drive_crank,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0),
    )
    model.articulation(
        "drive_to_primary_link",
        ArticulationType.FIXED,
        parent=drive_crank,
        child=primary_link,
        origin=Origin(xyz=(-0.032, 0.010, 0.006)),
    )
    model.articulation(
        "left_wiper_sweep",
        ArticulationType.REVOLUTE,
        parent=cowl_module,
        child=left_wiper_module,
        origin=Origin(xyz=(-0.31, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.12, upper=0.42),
    )
    model.articulation(
        "left_to_cross_link",
        ArticulationType.FIXED,
        parent=left_wiper_module,
        child=cross_link,
        origin=Origin(xyz=(0.059, -0.032, -0.014)),
    )
    model.articulation(
        "right_wiper_sweep",
        ArticulationType.REVOLUTE,
        parent=cowl_module,
        child=right_wiper_module,
        origin=Origin(xyz=(0.31, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.34, upper=0.10),
    )
    model.articulation(
        "left_blade_pitch",
        ArticulationType.REVOLUTE,
        parent=left_wiper_module,
        child=left_blade_carrier,
        origin=Origin(xyz=(-0.250, 0.090, 0.059)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=-0.05, upper=0.05),
    )
    model.articulation(
        "right_blade_pitch",
        ArticulationType.REVOLUTE,
        parent=right_wiper_module,
        child=right_blade_carrier,
        origin=Origin(xyz=(0.220, 0.072, 0.054)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=-0.05, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    cowl_module = object_model.get_part("cowl_module")
    motor_pack = object_model.get_part("motor_pack")
    drive_crank = object_model.get_part("drive_crank")
    primary_link = object_model.get_part("primary_link")
    left_wiper_module = object_model.get_part("left_wiper_module")
    right_wiper_module = object_model.get_part("right_wiper_module")
    cross_link = object_model.get_part("cross_link")
    left_blade_carrier = object_model.get_part("left_blade_carrier")
    right_blade_carrier = object_model.get_part("right_blade_carrier")

    motor_drive = object_model.get_articulation("motor_drive")
    left_wiper_sweep = object_model.get_articulation("left_wiper_sweep")
    right_wiper_sweep = object_model.get_articulation("right_wiper_sweep")
    left_blade_pitch = object_model.get_articulation("left_blade_pitch")
    right_blade_pitch = object_model.get_articulation("right_blade_pitch")

    output_boss = motor_pack.get_visual("output_boss")
    crank_hub = drive_crank.get_visual("hub")
    primary_stud = drive_crank.get_visual("primary_stud")
    primary_bushing = primary_link.get_visual("proximal_bushing")
    primary_eye = primary_link.get_visual("distal_eye")
    left_socket = left_wiper_module.get_visual("relay_socket")
    right_socket = right_wiper_module.get_visual("relay_socket")
    cross_bushing = cross_link.get_visual("proximal_bushing")
    cross_eye = cross_link.get_visual("distal_eye")
    left_blade_adapter = left_blade_carrier.get_visual("adapter_bridge")
    right_blade_adapter = right_blade_carrier.get_visual("adapter_bridge")
    left_blade_pad = left_wiper_module.get_visual("blade_pad")
    right_blade_pad = right_wiper_module.get_visual("blade_pad")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "motor_drive_axis_vertical",
        tuple(motor_drive.axis) == (0.0, 0.0, 1.0),
        f"Unexpected motor drive axis: {motor_drive.axis}",
    )
    ctx.check(
        "sweep_axes_vertical",
        tuple(left_wiper_sweep.axis) == (0.0, 0.0, 1.0)
        and tuple(right_wiper_sweep.axis) == (0.0, 0.0, 1.0),
        "Spindle axes should be vertical.",
    )
    ctx.check(
        "blade_pitch_axes_longitudinal",
        tuple(left_blade_pitch.axis) == (1.0, 0.0, 0.0)
        and tuple(right_blade_pitch.axis) == (1.0, 0.0, 0.0),
        "Blade carriers should pitch around their local longitudinal axis.",
    )

    ctx.expect_contact(cowl_module, motor_pack)
    ctx.expect_contact(motor_pack, drive_crank, elem_a=output_boss, elem_b=crank_hub)
    ctx.expect_contact(drive_crank, primary_link, elem_a=primary_stud, elem_b=primary_bushing)
    ctx.expect_contact(cowl_module, left_wiper_module)
    ctx.expect_contact(cowl_module, right_wiper_module)
    ctx.expect_contact(left_wiper_module, cross_link, elem_a=left_socket, elem_b=cross_bushing)
    ctx.expect_contact(left_wiper_module, left_blade_carrier, elem_a=left_blade_pad, elem_b=left_blade_adapter)
    ctx.expect_contact(right_wiper_module, right_blade_carrier, elem_a=right_blade_pad, elem_b=right_blade_adapter)

    ctx.expect_overlap(primary_link, left_wiper_module, axes="xy", elem_a=primary_eye, elem_b=left_socket, min_overlap=0.008)
    ctx.expect_overlap(cross_link, right_wiper_module, axes="xy", elem_a=cross_eye, elem_b=right_socket, min_overlap=0.008)

    cowl_aabb = ctx.part_world_aabb(cowl_module)
    left_blade_aabb = ctx.part_world_aabb(left_blade_carrier)
    right_blade_aabb = ctx.part_world_aabb(right_blade_carrier)
    if cowl_aabb is not None and left_blade_aabb is not None and right_blade_aabb is not None:
        assembly_width = cowl_aabb[1][0] - cowl_aabb[0][0]
        left_blade_length = left_blade_aabb[1][0] - left_blade_aabb[0][0]
        right_blade_length = right_blade_aabb[1][0] - right_blade_aabb[0][0]
        ctx.check(
            "realistic_assembly_width",
            1.00 <= assembly_width <= 1.10,
            f"Assembly width {assembly_width:.3f} m should read as a realistic premium two-spindle module.",
        )
        ctx.check(
            "driver_blade_longer_than_passenger_blade",
            left_blade_length > right_blade_length + 0.05,
            f"Expected left blade to be noticeably longer: left={left_blade_length:.3f}, right={right_blade_length:.3f}",
        )

    def _check_bounded_joint(joint, label: str, parent_part, child_part) -> None:
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            return
        for suffix, pose_value in (("lower", limits.lower), ("upper", limits.upper)):
            with ctx.pose({joint: pose_value}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_{suffix}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_{suffix}_supported")
                ctx.expect_contact(parent_part, child_part, name=f"{label}_{suffix}_contact")

    _check_bounded_joint(left_wiper_sweep, "left_wiper_sweep", cowl_module, left_wiper_module)
    _check_bounded_joint(right_wiper_sweep, "right_wiper_sweep", cowl_module, right_wiper_module)
    _check_bounded_joint(left_blade_pitch, "left_blade_pitch", left_wiper_module, left_blade_carrier)
    _check_bounded_joint(right_blade_pitch, "right_blade_pitch", right_wiper_module, right_blade_carrier)

    with ctx.pose({motor_drive: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="motor_drive_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="motor_drive_quarter_turn_supported")
        ctx.expect_contact(motor_pack, drive_crank, elem_a=output_boss, elem_b=crank_hub, name="motor_drive_quarter_turn_contact")

    with ctx.pose(
        {
            left_wiper_sweep: 0.22,
            right_wiper_sweep: -0.18,
            left_blade_pitch: 0.03,
            right_blade_pitch: -0.03,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="service_sweep_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="service_sweep_pose_supported")
        ctx.expect_contact(left_wiper_module, left_blade_carrier, elem_a=left_blade_pad, elem_b=left_blade_adapter, name="left_blade_service_pose_contact")
        ctx.expect_contact(right_wiper_module, right_blade_carrier, elem_a=right_blade_pad, elem_b=right_blade_adapter, name="right_blade_service_pose_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
