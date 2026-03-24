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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _annular_mesh(name: str, length: float, outer_radius: float, inner_radius: float):
    geom = ExtrudeWithHolesGeometry(
        outer_profile=_circle_profile(outer_radius),
        hole_profiles=[_circle_profile(inner_radius)],
        height=length,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_dir / f"{name}.obj")


def _spinner_mesh(name: str):
    geom = LatheGeometry(
        [
            (0.0, -0.10),
            (0.018, -0.09),
            (0.045, -0.06),
            (0.058, -0.02),
            (0.046, 0.01),
            (0.032, 0.022),
            (0.0, 0.03),
        ],
        segments=64,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_dir / f"{name}.obj")


def _x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fighter_jet_engine", assets=ASSETS)

    titanium = model.material("titanium", rgba=(0.58, 0.60, 0.63, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.30, 0.33, 1.0))
    heat_stained = model.material("heat_stained", rgba=(0.50, 0.46, 0.52, 1.0))
    soot = model.material("soot", rgba=(0.15, 0.16, 0.17, 1.0))

    core = model.part("core_case")
    core.visual(
        _annular_mesh("intake_shell", length=0.12, outer_radius=0.34, inner_radius=0.225),
        origin=Origin(xyz=(-0.35, 0.0, 0.0)),
        material=titanium,
        name="intake_shell",
    )
    core.visual(
        _annular_mesh("compressor_shell", length=0.32, outer_radius=0.30, inner_radius=0.215),
        origin=Origin(xyz=(-0.13, 0.0, 0.0)),
        material=titanium,
        name="compressor_shell",
    )
    core.visual(
        _annular_mesh("combustor_shell", length=0.20, outer_radius=0.335, inner_radius=0.195),
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
        material=titanium,
        name="combustor_shell",
    )
    core.visual(
        _annular_mesh("afterburner_shell", length=0.22, outer_radius=0.27, inner_radius=0.165),
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
        material=heat_stained,
        name="afterburner_shell",
    )
    core.visual(
        _annular_mesh("nozzle_mount", length=0.04, outer_radius=0.225, inner_radius=0.165),
        origin=Origin(xyz=(0.47, 0.0, 0.0)),
        material=heat_stained,
        name="nozzle_mount",
    )
    core.visual(
        _annular_mesh("front_bearing_ring", length=0.04, outer_radius=0.065, inner_radius=0.031),
        origin=Origin(xyz=(-0.335, 0.0, 0.0)),
        material=gunmetal,
        name="front_bearing",
    )
    for index in range(4):
        angle = index * (math.pi / 2.0)
        core.visual(
            Box((0.012, 0.16, 0.014)),
            origin=Origin(
                xyz=(-0.35, 0.145 * math.cos(angle), 0.145 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=gunmetal,
            name=f"stator_strut_{index}",
        )
    core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.90),
        mass=190.0,
        origin=_x_axis_origin((0.04, 0.0, 0.0)),
    )

    rotor = model.part("fan_rotor")
    rotor.visual(
        _spinner_mesh("fan_spinner"),
        origin=Origin(xyz=(-0.095, 0.0, 0.0)),
        material=gunmetal,
        name="spinner",
    )
    rotor.visual(
        Cylinder(radius=0.028, length=0.055),
        origin=_x_axis_origin((-0.0425, 0.0, 0.0)),
        material=gunmetal,
        name="spinner_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.035, length=0.03),
        origin=_x_axis_origin((0.0, 0.0, 0.0)),
        material=gunmetal,
        name="hub_collar",
    )
    rotor.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=_x_axis_origin((0.02, 0.0, 0.0)),
        material=gunmetal,
        name="hub_core",
    )
    rotor.visual(
        Cylinder(radius=0.18, length=0.02),
        origin=_x_axis_origin((0.06, 0.0, 0.0)),
        material=gunmetal,
        name="fan_disc",
    )
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        rotor.visual(
            Box((0.07, 0.16, 0.014)),
            origin=Origin(
                xyz=(0.06, 0.12 * math.cos(angle), 0.12 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=soot,
            name=f"blade_{index}",
        )
    rotor.visual(
        Cylinder(radius=0.14, length=0.018),
        origin=_x_axis_origin((0.11, 0.0, 0.0)),
        material=soot,
        name="booster_disc",
    )
    rotor.visual(
        Cylinder(radius=0.03, length=0.10),
        origin=_x_axis_origin((0.07, 0.0, 0.0)),
        material=gunmetal,
        name="aft_shaft",
    )
    for index in range(10):
        angle = ((2.0 * math.pi * index) / 10.0) + (math.pi / 10.0)
        rotor.visual(
            Box((0.05, 0.10, 0.01)),
            origin=Origin(
                xyz=(0.11, 0.075 * math.cos(angle), 0.075 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=soot,
            name=f"booster_blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.24),
        mass=38.0,
        origin=_x_axis_origin((0.01, 0.0, 0.0)),
    )

    nozzle = model.part("nozzle_petals")
    nozzle.visual(
        _annular_mesh("petal_base_ring", length=0.02, outer_radius=0.225, inner_radius=0.165),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=heat_stained,
        name="petal_base_ring",
    )
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        nozzle.visual(
            Box((0.12, 0.075, 0.01)),
            origin=Origin(
                xyz=(0.07, 0.202 * math.cos(angle), 0.202 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=heat_stained,
            name=f"petal_{index}",
        )
    nozzle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.14),
        mass=28.0,
        origin=_x_axis_origin((0.07, 0.0, 0.0)),
    )

    model.articulation(
        "core_to_fan_rotor",
        ArticulationType.CONTINUOUS,
        parent=core,
        child=rotor,
        origin=Origin(xyz=(-0.30, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=600.0),
    )
    model.articulation(
        "core_to_nozzle_petals",
        ArticulationType.FIXED,
        parent=core,
        child=nozzle,
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    core = object_model.get_part("core_case")
    rotor = object_model.get_part("fan_rotor")
    nozzle = object_model.get_part("nozzle_petals")
    rotor_joint = object_model.get_articulation("core_to_fan_rotor")
    front_bearing = core.get_visual("front_bearing")
    nozzle_mount = core.get_visual("nozzle_mount")
    hub_collar = rotor.get_visual("hub_collar")
    petal_base_ring = nozzle.get_visual("petal_base_ring")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Hollow coaxial engine modules place joint origins on the centerline inside open flow paths,
    # so the fixed metric articulation-origin distance warning is not a useful sensor here.
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_origin_distance(rotor, core, axes="yz", max_dist=0.01)
    ctx.expect_within(rotor, core, axes="yz")
    ctx.expect_overlap(rotor, core, axes="yz", min_overlap=0.22)
    ctx.expect_gap(
        rotor,
        core,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hub_collar,
        negative_elem=front_bearing,
        name="fan_bearing_seat",
    )
    ctx.expect_origin_distance(nozzle, core, axes="yz", max_dist=0.01)
    ctx.expect_overlap(nozzle, core, axes="yz", min_overlap=0.18)
    ctx.expect_gap(
        nozzle,
        core,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=petal_base_ring,
        negative_elem=nozzle_mount,
        name="nozzle_petals_seated_on_mount",
    )
    ctx.expect_gap(
        nozzle,
        rotor,
        axis="x",
        min_gap=0.30,
        name="nozzle_is_well_aft_of_the_fan",
    )
    with ctx.pose({rotor_joint: math.pi / 2.0}):
        ctx.expect_origin_distance(rotor, core, axes="yz", max_dist=0.01)
        ctx.expect_overlap(rotor, core, axes="yz", min_overlap=0.22)
        ctx.expect_gap(
            rotor,
            core,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=hub_collar,
            negative_elem=front_bearing,
            name="fan_bearing_seat_at_spin_pose",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
