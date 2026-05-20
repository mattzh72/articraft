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

ASSETS = AssetContext.from_script(__file__)


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _beam_mesh(
    filename: str,
    *,
    length: float,
    width: float,
    height: float,
    corner_radius: float,
    center: tuple[float, float, float],
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(
            width,
            height,
            radius=min(corner_radius, width * 0.5 - 1e-4, height * 0.5 - 1e-4),
            corner_segments=8,
        ),
        length,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_y(math.pi * 0.5)
    geom.translate(*center)
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _ring_collar_mesh(
    filename: str,
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
    center: tuple[float, float, float],
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=44),
        [_circle_profile(inner_radius, segments=36)],
        length,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi * 0.5)
    geom.translate(*center)
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _add_y_bolts(
    part,
    *,
    prefix: str,
    surface_y: float,
    sign: float,
    x_positions: tuple[float, ...],
    z_positions: tuple[float, ...],
    radius: float,
    length: float,
    embed: float,
    material,
) -> None:
    center_y = surface_y + sign * (0.5 * length - embed)
    for ix, x_pos in enumerate(x_positions):
        for iz, z_pos in enumerate(z_positions):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x_pos, center_y, z_pos), rpy=(math.pi * 0.5, 0.0, 0.0)),
                material=material,
                name=f"{prefix}_{ix}_{iz}",
            )


def _add_x_bolts(
    part,
    *,
    prefix: str,
    surface_x: float,
    sign: float,
    y_positions: tuple[float, ...],
    z_positions: tuple[float, ...],
    radius: float,
    length: float,
    embed: float,
    material,
) -> None:
    center_x = surface_x + sign * (0.5 * length - embed)
    for iy, y_pos in enumerate(y_positions):
        for iz, z_pos in enumerate(z_positions):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(center_x, y_pos, z_pos), rpy=(0.0, math.pi * 0.5, 0.0)),
                material=material,
                name=f"{prefix}_{iy}_{iz}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_robotic_arm", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.74, 0.44, 0.16, 1.0))
    graphite_polymer = model.material("graphite_polymer", rgba=(0.18, 0.19, 0.21, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    fastener_black = model.material("fastener_black", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    shoulder_z = 0.41
    shoulder_gap = 0.070
    shoulder_cheek_thickness = 0.024
    shoulder_cheek_y = 0.5 * shoulder_gap + 0.5 * shoulder_cheek_thickness
    shoulder_outer_face_y = shoulder_cheek_y + 0.5 * shoulder_cheek_thickness

    elbow_x = 0.42
    elbow_gap = 0.074
    elbow_cheek_thickness = 0.020
    elbow_cheek_y = 0.5 * elbow_gap + 0.5 * elbow_cheek_thickness
    elbow_outer_face_y = elbow_cheek_y + 0.5 * elbow_cheek_thickness

    wrist_x = 0.34
    wrist_gap = 0.050
    wrist_cheek_thickness = 0.018
    wrist_cheek_y = 0.5 * wrist_gap + 0.5 * wrist_cheek_thickness
    wrist_outer_face_y = wrist_cheek_y + 0.5 * wrist_cheek_thickness

    base = model.part("base")
    base.visual(
        Box((0.50, 0.42, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=painted_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.32, 0.28, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=painted_steel,
        name="ballast_block",
    )
    base.visual(
        Box((0.20, 0.18, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=painted_steel,
        name="pedestal",
    )
    base.visual(
        Box((0.08, 0.020, 0.10)),
        origin=Origin(xyz=(0.010, shoulder_cheek_y, 0.350)),
        material=painted_steel,
        name="shoulder_web_pos",
    )
    base.visual(
        Box((0.08, 0.020, 0.10)),
        origin=Origin(xyz=(0.010, -shoulder_cheek_y, 0.350)),
        material=painted_steel,
        name="shoulder_web_neg",
    )
    base.visual(
        Box((0.10, shoulder_cheek_thickness, 0.18)),
        origin=Origin(xyz=(0.018, shoulder_cheek_y, shoulder_z)),
        material=machined_steel,
        name="shoulder_cheek_pos",
    )
    base.visual(
        Box((0.10, shoulder_cheek_thickness, 0.18)),
        origin=Origin(xyz=(0.018, -shoulder_cheek_y, shoulder_z)),
        material=machined_steel,
        name="shoulder_cheek_neg",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, shoulder_z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=machined_steel,
        name="shoulder_pin",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.040),
        origin=Origin(xyz=(-0.012, 0.079, shoulder_z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=graphite_polymer,
        name="shoulder_drive_pod",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=Origin(xyz=(-0.012, -0.073, shoulder_z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=graphite_polymer,
        name="service_cover",
    )
    for ix, x_pos in enumerate((-0.18, 0.18)):
        for iy, y_pos in enumerate((-0.14, 0.14)):
            base.visual(
                Cylinder(radius=0.032, length=0.018),
                origin=Origin(xyz=(x_pos, y_pos, 0.009)),
                material=rubber,
                name=f"foot_{ix}_{iy}",
            )
    _add_y_bolts(
        base,
        prefix="shoulder_bolt_pos",
        surface_y=shoulder_outer_face_y,
        sign=1.0,
        x_positions=(-0.005, 0.040),
        z_positions=(shoulder_z - 0.045, shoulder_z + 0.045),
        radius=0.006,
        length=0.010,
        embed=0.0025,
        material=fastener_black,
    )
    _add_y_bolts(
        base,
        prefix="shoulder_bolt_neg",
        surface_y=-shoulder_outer_face_y,
        sign=-1.0,
        x_positions=(-0.005, 0.040),
        z_positions=(shoulder_z - 0.045, shoulder_z + 0.045),
        radius=0.006,
        length=0.010,
        embed=0.0025,
        material=fastener_black,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.50, 0.42, 0.30)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        _ring_collar_mesh(
            "upper_arm_shoulder_collar.obj",
            length=shoulder_gap - 0.002,
            outer_radius=0.055,
            inner_radius=0.017,
            center=(0.0, 0.0, 0.0),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_steel,
        name="shoulder_collar",
    )
    upper_arm.visual(
        _beam_mesh(
            "upper_arm_beam.obj",
            length=0.29,
            width=0.095,
            height=0.050,
            corner_radius=0.016,
            center=(0.215, 0.0, 0.030),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_steel,
        name="upper_arm_beam",
    )
    upper_arm.visual(
        Box((0.11, 0.050, 0.080)),
        origin=Origin(xyz=(0.090, 0.0, 0.020)),
        material=painted_steel,
        name="shoulder_reinforcement",
    )
    upper_arm.visual(
        Box((0.16, 0.085, 0.055)),
        origin=Origin(xyz=(0.180, 0.0, 0.078)),
        material=graphite_polymer,
        name="shoulder_actuator_housing",
    )
    upper_arm.visual(
        Cylinder(radius=0.022, length=0.130),
        origin=Origin(xyz=(0.155, 0.0, 0.074), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=machined_steel,
        name="shoulder_actuator_can",
    )
    upper_arm.visual(
        Box((0.18, 0.040, 0.030)),
        origin=Origin(xyz=(0.185, 0.0, -0.010)),
        material=painted_steel,
        name="upper_arm_rib",
    )
    upper_arm.visual(
        Box((0.09, elbow_cheek_thickness, 0.12)),
        origin=Origin(xyz=(elbow_x, elbow_cheek_y, 0.0)),
        material=machined_steel,
        name="elbow_cheek_pos",
    )
    upper_arm.visual(
        Box((0.09, elbow_cheek_thickness, 0.12)),
        origin=Origin(xyz=(elbow_x, -elbow_cheek_y, 0.0)),
        material=machined_steel,
        name="elbow_cheek_neg",
    )
    upper_arm.visual(
        Box((0.054, 0.008, 0.075)),
        origin=Origin(xyz=(0.387, elbow_outer_face_y - 0.007, 0.030)),
        material=painted_steel,
        name="elbow_web_pos",
    )
    upper_arm.visual(
        Box((0.054, 0.008, 0.075)),
        origin=Origin(xyz=(0.387, -elbow_outer_face_y + 0.007, 0.030)),
        material=painted_steel,
        name="elbow_web_neg",
    )
    upper_arm.visual(
        Cylinder(radius=0.014, length=0.124),
        origin=Origin(xyz=(elbow_x, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=machined_steel,
        name="elbow_pin",
    )
    _add_y_bolts(
        upper_arm,
        prefix="elbow_bolt_pos",
        surface_y=elbow_outer_face_y,
        sign=1.0,
        x_positions=(elbow_x - 0.020, elbow_x + 0.020),
        z_positions=(-0.030, 0.030),
        radius=0.005,
        length=0.012,
        embed=0.006,
        material=fastener_black,
    )
    _add_y_bolts(
        upper_arm,
        prefix="elbow_bolt_neg",
        surface_y=-elbow_outer_face_y,
        sign=-1.0,
        x_positions=(elbow_x - 0.020, elbow_x + 0.020),
        z_positions=(-0.030, 0.030),
        radius=0.005,
        length=0.012,
        embed=0.006,
        material=fastener_black,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.50, 0.16, 0.18)),
        mass=9.0,
        origin=Origin(xyz=(0.23, 0.0, 0.02)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        _ring_collar_mesh(
            "forearm_elbow_collar.obj",
            length=elbow_gap - 0.002,
            outer_radius=0.046,
            inner_radius=0.015,
            center=(0.0, 0.0, 0.0),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_steel,
        name="elbow_collar",
    )
    forearm.visual(
        _beam_mesh(
            "forearm_beam.obj",
            length=0.24,
            width=0.085,
            height=0.080,
            corner_radius=0.013,
            center=(0.185, 0.0, 0.022),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_steel,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.090, 0.046, 0.070)),
        origin=Origin(xyz=(0.075, 0.0, 0.016)),
        material=painted_steel,
        name="elbow_reinforcement",
    )
    forearm.visual(
        Box((0.13, 0.070, 0.045)),
        origin=Origin(xyz=(0.145, 0.0, 0.080)),
        material=graphite_polymer,
        name="elbow_actuator_housing",
    )
    forearm.visual(
        Cylinder(radius=0.018, length=0.130),
        origin=Origin(xyz=(0.125, 0.0, 0.066), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=machined_steel,
        name="elbow_actuator_can",
    )
    forearm.visual(
        Box((0.16, 0.038, 0.028)),
        origin=Origin(xyz=(0.160, 0.0, -0.034)),
        material=painted_steel,
        name="forearm_rib",
    )
    forearm.visual(
        Box((0.07, wrist_cheek_thickness, 0.09)),
        origin=Origin(xyz=(wrist_x, wrist_cheek_y, 0.0)),
        material=machined_steel,
        name="wrist_cheek_pos",
    )
    forearm.visual(
        Box((0.07, wrist_cheek_thickness, 0.09)),
        origin=Origin(xyz=(wrist_x, -wrist_cheek_y, 0.0)),
        material=machined_steel,
        name="wrist_cheek_neg",
    )
    forearm.visual(
        Cylinder(radius=0.011, length=0.086),
        origin=Origin(xyz=(wrist_x, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=machined_steel,
        name="wrist_pin",
    )
    _add_y_bolts(
        forearm,
        prefix="wrist_bolt_pos",
        surface_y=wrist_outer_face_y,
        sign=1.0,
        x_positions=(wrist_x - 0.015, wrist_x + 0.015),
        z_positions=(-0.022, 0.022),
        radius=0.0045,
        length=0.008,
        embed=0.0018,
        material=fastener_black,
    )
    _add_y_bolts(
        forearm,
        prefix="wrist_bolt_neg",
        surface_y=-wrist_outer_face_y,
        sign=-1.0,
        x_positions=(wrist_x - 0.015, wrist_x + 0.015),
        z_positions=(-0.022, 0.022),
        radius=0.0045,
        length=0.008,
        embed=0.0018,
        material=fastener_black,
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.40, 0.13, 0.15)),
        mass=5.4,
        origin=Origin(xyz=(0.17, 0.0, 0.01)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        _ring_collar_mesh(
            "wrist_collar.obj",
            length=wrist_gap - 0.002,
            outer_radius=0.034,
            inner_radius=0.012,
            center=(0.0, 0.0, 0.0),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_steel,
        name="wrist_collar",
    )
    wrist.visual(
        Box((0.050, 0.040, 0.055)),
        origin=Origin(xyz=(0.040, 0.0, 0.015)),
        material=painted_steel,
        name="wrist_reinforcement",
    )
    wrist.visual(
        Box((0.085, 0.036, 0.042)),
        origin=Origin(xyz=(0.100, 0.0, 0.015)),
        material=painted_steel,
        name="wrist_bridge",
    )
    wrist.visual(
        Box((0.090, 0.044, 0.050)),
        origin=Origin(xyz=(0.180, 0.0, 0.000)),
        material=painted_steel,
        name="wrist_block",
    )
    wrist.visual(
        Box((0.070, 0.042, 0.024)),
        origin=Origin(xyz=(0.170, 0.0, 0.042)),
        material=graphite_polymer,
        name="wrist_cover",
    )
    wrist.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=machined_steel,
        name="tool_flange",
    )
    wrist.visual(
        Box((0.016, 0.068, 0.068)),
        origin=Origin(xyz=(0.218, 0.0, 0.0)),
        material=machined_steel,
        name="tool_face",
    )
    wrist.visual(
        Box((0.045, 0.014, 0.060)),
        origin=Origin(xyz=(0.252, 0.022, 0.0)),
        material=painted_steel,
        name="tool_lug_pos",
    )
    wrist.visual(
        Box((0.045, 0.014, 0.060)),
        origin=Origin(xyz=(0.252, -0.022, 0.0)),
        material=painted_steel,
        name="tool_lug_neg",
    )
    _add_x_bolts(
        wrist,
        prefix="flange_bolt",
        surface_x=0.226,
        sign=1.0,
        y_positions=(-0.020, 0.020),
        z_positions=(-0.020, 0.020),
        radius=0.004,
        length=0.008,
        embed=0.002,
        material=fastener_black,
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.20, 0.12, 0.12)),
        mass=2.2,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent="base",
        child="upper_arm",
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=1.6, lower=0.0, upper=0.90),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="forearm",
        origin=Origin(xyz=(elbow_x, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.9, lower=0.0, upper=0.95),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent="forearm",
        child="wrist",
        origin=Origin(xyz=(wrist_x, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.4, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_gap(
        "base",
        "upper_arm",
        axis="y",
        positive_elem="shoulder_cheek_pos",
        negative_elem="shoulder_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="shoulder collar seats against positive base cheek",
    )
    ctx.expect_aabb_gap(
        "upper_arm",
        "base",
        axis="y",
        positive_elem="shoulder_collar",
        negative_elem="shoulder_cheek_neg",
        max_gap=0.002,
        max_penetration=0.0,
        name="shoulder collar seats against negative base cheek",
    )
    ctx.expect_aabb_gap(
        "upper_arm",
        "forearm",
        axis="y",
        positive_elem="elbow_cheek_pos",
        negative_elem="elbow_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="elbow collar seats against positive clevis cheek",
    )
    ctx.expect_aabb_gap(
        "forearm",
        "upper_arm",
        axis="y",
        positive_elem="elbow_collar",
        negative_elem="elbow_cheek_neg",
        max_gap=0.002,
        max_penetration=0.0,
        name="elbow collar seats against negative clevis cheek",
    )
    ctx.expect_aabb_gap(
        "forearm",
        "wrist",
        axis="y",
        positive_elem="wrist_cheek_pos",
        negative_elem="wrist_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="wrist collar seats against positive clevis cheek",
    )
    ctx.expect_aabb_gap(
        "wrist",
        "forearm",
        axis="y",
        positive_elem="wrist_collar",
        negative_elem="wrist_cheek_neg",
        max_gap=0.002,
        max_penetration=0.0,
        name="wrist collar seats against negative clevis cheek",
    )
    ctx.expect_joint_motion_axis(
        "shoulder_pitch",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        "elbow_pitch",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.06,
    )
    ctx.expect_joint_motion_axis(
        "wrist_pitch",
        "wrist",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_aabb_gap(
        "wrist",
        "base",
        axis="x",
        positive_elem="tool_face",
        negative_elem="pedestal",
        min_gap=0.40,
        name="tooling projects forward from the stable base",
    )
    with ctx.pose(shoulder_pitch=0.82):
        ctx.expect_aabb_gap(
            "upper_arm",
            "base",
            axis="z",
            positive_elem="upper_arm_beam",
            negative_elem="pedestal",
            min_gap=0.006,
            name="raised upper arm clears the pedestal block",
        )
    with ctx.pose(shoulder_pitch=0.55, elbow_pitch=0.40, wrist_pitch=0.20):
        ctx.expect_aabb_gap(
            "wrist",
            "base",
            axis="z",
            positive_elem="tool_face",
            negative_elem="base_plate",
            min_gap=0.06,
            name="service pose keeps the wrist tooling above the base plate",
        )
        ctx.expect_aabb_gap(
            "wrist",
            "base",
            axis="x",
            positive_elem="tool_face",
            negative_elem="pedestal",
            min_gap=0.14,
            name="service pose keeps the tool face forward of the pedestal",
        )
        ctx.expect_aabb_gap(
            "forearm",
            "base",
            axis="z",
            positive_elem="forearm_beam",
            negative_elem="pedestal",
            min_gap=0.05,
            name="service pose keeps the forearm beam above the pedestal",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
