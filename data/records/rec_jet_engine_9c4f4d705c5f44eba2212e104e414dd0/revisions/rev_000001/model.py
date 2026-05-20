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
    ConeGeometry,
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


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _engine_main_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.205, -0.34),
            (0.218, -0.28),
            (0.226, -0.04),
            (0.210, 0.10),
            (0.175, 0.18),
        ],
        [
            (0.165, -0.34),
            (0.182, -0.28),
            (0.188, -0.05),
            (0.173, 0.10),
            (0.143, 0.18),
        ],
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    ).rotate_y(math.pi / 2.0)


def _engine_aft_nozzle_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.176, 0.10),
            (0.130, 0.16),
            (0.124, 0.24),
            (0.116, 0.36),
        ],
        [
            (0.148, 0.10),
            (0.102, 0.16),
            (0.096, 0.24),
            (0.090, 0.36),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _exhaust_sleeve_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.162, 0.00),
            (0.156, 0.035),
            (0.156, 0.12),
            (0.152, 0.28),
        ],
        [
            (0.136, 0.00),
            (0.136, 0.035),
            (0.138, 0.12),
            (0.134, 0.28),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _intake_rotor_mesh() -> MeshGeometry:
    shaft = CylinderGeometry(radius=0.016, height=0.24, radial_segments=28).rotate_y(
        math.pi / 2.0
    )
    shaft.translate(0.08, 0.0, 0.0)

    hub = CylinderGeometry(radius=0.056, height=0.10, radial_segments=40).rotate_y(
        math.pi / 2.0
    )

    root_disc = CylinderGeometry(radius=0.074, height=0.016, radial_segments=40).rotate_y(
        math.pi / 2.0
    )
    root_disc.translate(0.010, 0.0, 0.0)

    spinner = ConeGeometry(radius=0.055, height=0.10, radial_segments=40).rotate_y(
        math.pi / 2.0
    )
    spinner.translate(-0.055, 0.0, 0.0)

    blade_base = BoxGeometry((0.075, 0.012, 0.060))
    blade_base.translate(0.018, 0.0, 0.058)
    blade_base.rotate_y(0.32)

    blades = MeshGeometry()
    for blade_index in range(8):
        blades.merge(blade_base.copy().rotate((1.0, 0.0, 0.0), blade_index * math.tau / 8.0))

    return _merge_geometries(shaft, hub, root_disc, spinner, blades)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="auxiliary_jet_engine_test_dolly")

    dolly_yellow = model.material("dolly_yellow", rgba=(0.86, 0.72, 0.18, 1.0))
    engine_gray = model.material("engine_gray", rgba=(0.70, 0.73, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    bright_aluminum = model.material("bright_aluminum", rgba=(0.78, 0.80, 0.84, 1.0))
    hot_steel = model.material("hot_steel", rgba=(0.43, 0.38, 0.34, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    test_dolly = model.part("test_dolly")
    test_dolly.visual(
        Box((1.02, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.19, 0.21)),
        material=dolly_yellow,
        name="left_frame_rail",
    )
    test_dolly.visual(
        Box((1.02, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.19, 0.21)),
        material=dolly_yellow,
        name="right_frame_rail",
    )
    test_dolly.visual(
        Box((0.10, 0.44, 0.05)),
        origin=Origin(xyz=(-0.40, 0.0, 0.205)),
        material=dolly_yellow,
        name="front_crossmember",
    )
    test_dolly.visual(
        Box((0.16, 0.44, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=dolly_yellow,
        name="center_crossmember",
    )
    test_dolly.visual(
        Box((0.10, 0.44, 0.05)),
        origin=Origin(xyz=(0.40, 0.0, 0.205)),
        material=dolly_yellow,
        name="rear_crossmember",
    )
    test_dolly.visual(
        Box((0.54, 0.34, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=dark_steel,
        name="deck_plate",
    )
    test_dolly.visual(
        Box((0.18, 0.16, 0.18)),
        origin=Origin(xyz=(-0.12, 0.0, 0.265)),
        material=dolly_yellow,
        name="front_pedestal",
    )
    test_dolly.visual(
        Box((0.18, 0.12, 0.02)),
        origin=Origin(xyz=(-0.12, 0.0, 0.345)),
        material=dark_steel,
        name="front_mount_pad",
    )
    test_dolly.visual(
        Box((0.18, 0.16, 0.18)),
        origin=Origin(xyz=(0.12, 0.0, 0.265)),
        material=dolly_yellow,
        name="rear_pedestal",
    )
    test_dolly.visual(
        Box((0.18, 0.12, 0.02)),
        origin=Origin(xyz=(0.12, 0.0, 0.345)),
        material=dark_steel,
        name="rear_mount_pad",
    )
    test_dolly.visual(
        Box((0.08, 0.32, 0.04)),
        origin=Origin(xyz=(-0.50, 0.0, 0.24)),
        material=dolly_yellow,
        name="tow_bar",
    )

    for wheel_index, (wheel_x, wheel_y) in enumerate(
        ((-0.44, 0.19), (-0.44, -0.19), (0.44, 0.19), (0.44, -0.19))
    ):
        side_sign = 1.0 if wheel_y > 0.0 else -1.0
        test_dolly.visual(
            Box((0.06, 0.09, 0.014)),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.159)),
            material=dark_steel,
            name=f"caster_yoke_{wheel_index}",
        )
        test_dolly.visual(
            Box((0.04, 0.04, 0.03)),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.180)),
            material=dark_steel,
            name=f"caster_stem_{wheel_index}",
        )
        test_dolly.visual(
            Box((0.014, 0.018, 0.09)),
            origin=Origin(xyz=(wheel_x, wheel_y + side_sign * 0.034, 0.095)),
            material=dark_steel,
            name=f"caster_fork_outer_{wheel_index}",
        )
        test_dolly.visual(
            Box((0.014, 0.018, 0.09)),
            origin=Origin(xyz=(wheel_x, wheel_y - side_sign * 0.034, 0.095)),
            material=dark_steel,
            name=f"caster_fork_inner_{wheel_index}",
        )
        test_dolly.visual(
            Cylinder(radius=0.080, length=0.050),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"wheel_{wheel_index}",
        )
        test_dolly.visual(
            Cylinder(radius=0.020, length=0.064),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"wheel_hub_{wheel_index}",
        )

    test_dolly.inertial = Inertial.from_geometry(
        Box((1.18, 0.62, 0.42)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
    )

    engine_body = model.part("engine_body")
    engine_body.visual(
        _save_mesh("engine_main_shell", _engine_main_shell_mesh()),
        material=engine_gray,
        name="main_shell",
    )
    engine_body.visual(
        _save_mesh("engine_aft_nozzle", _engine_aft_nozzle_mesh()),
        material=dark_steel,
        name="aft_nozzle_shell",
    )
    engine_body.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft_bearing",
    )
    engine_body.visual(
        Box((0.018, 0.040, 0.176)),
        origin=Origin(xyz=(-0.014, 0.0, 0.110)),
        material=dark_steel,
        name="upper_bearing_strut",
    )
    engine_body.visual(
        Box((0.018, 0.040, 0.176)),
        origin=Origin(xyz=(-0.014, 0.0, -0.110)),
        material=dark_steel,
        name="lower_bearing_strut",
    )
    engine_body.visual(
        Box((0.11, 0.08, 0.03)),
        origin=Origin(xyz=(-0.12, 0.0, -0.230)),
        material=dark_steel,
        name="front_mount_shoe",
    )
    engine_body.visual(
        Box((0.05, 0.08, 0.06)),
        origin=Origin(xyz=(-0.12, 0.0, -0.185)),
        material=dark_steel,
        name="front_mount_strut",
    )
    engine_body.visual(
        Box((0.11, 0.08, 0.03)),
        origin=Origin(xyz=(0.12, 0.0, -0.230)),
        material=dark_steel,
        name="rear_mount_shoe",
    )
    engine_body.visual(
        Box((0.05, 0.08, 0.06)),
        origin=Origin(xyz=(0.12, 0.0, -0.185)),
        material=dark_steel,
        name="rear_mount_strut",
    )
    engine_body.inertial = Inertial.from_geometry(
        Box((0.76, 0.46, 0.52)),
        mass=44.0,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
    )

    intake_rotor = model.part("intake_rotor")
    intake_rotor.visual(
        _save_mesh("intake_rotor_assembly", _intake_rotor_mesh()),
        material=bright_aluminum,
        name="rotor_assembly",
    )
    intake_rotor.visual(
        Cylinder(radius=0.018, length=0.20),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rotor_shaft",
    )
    intake_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=0.24),
        mass=3.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    exhaust_sleeve = model.part("exhaust_sleeve")
    exhaust_sleeve.visual(
        _save_mesh("exhaust_sleeve_shell", _exhaust_sleeve_mesh()),
        material=hot_steel,
        name="sleeve_shell",
    )
    exhaust_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.30),
        mass=4.4,
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "dolly_to_engine",
        ArticulationType.FIXED,
        parent=test_dolly,
        child=engine_body,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )
    model.articulation(
        "engine_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=engine_body,
        child=intake_rotor,
        origin=Origin(xyz=(-0.235, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=60.0),
    )
    model.articulation(
        "engine_to_exhaust_sleeve",
        ArticulationType.PRISMATIC,
        parent=engine_body,
        child=exhaust_sleeve,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    dolly = object_model.get_part("test_dolly")
    engine = object_model.get_part("engine_body")
    rotor = object_model.get_part("intake_rotor")
    sleeve = object_model.get_part("exhaust_sleeve")
    rotor_joint = object_model.get_articulation("engine_to_rotor")
    sleeve_joint = object_model.get_articulation("engine_to_exhaust_sleeve")

    ctx.check(
        "rotor articulation spins around engine axis",
        rotor_joint.axis == (1.0, 0.0, 0.0)
        and rotor_joint.motion_limits is not None
        and rotor_joint.motion_limits.lower is None
        and rotor_joint.motion_limits.upper is None,
        details=f"axis={rotor_joint.axis}, limits={rotor_joint.motion_limits}",
    )
    ctx.check(
        "sleeve articulation slides rearward along engine axis",
        sleeve_joint.axis == (1.0, 0.0, 0.0)
        and sleeve_joint.motion_limits is not None
        and sleeve_joint.motion_limits.lower == 0.0
        and sleeve_joint.motion_limits.upper is not None
        and sleeve_joint.motion_limits.upper >= 0.10,
        details=f"axis={sleeve_joint.axis}, limits={sleeve_joint.motion_limits}",
    )

    with ctx.pose({sleeve_joint: 0.0, rotor_joint: 0.5}):
        ctx.expect_contact(
            engine,
            dolly,
            elem_a="front_mount_shoe",
            elem_b="front_mount_pad",
            name="front shoe rests on front dolly pad",
        )
        ctx.expect_contact(
            engine,
            dolly,
            elem_a="rear_mount_shoe",
            elem_b="rear_mount_pad",
            name="rear shoe rests on rear dolly pad",
        )
        ctx.expect_overlap(
            rotor,
            engine,
            axes="yz",
            elem_a="rotor_assembly",
            elem_b="main_shell",
            min_overlap=0.12,
            name="intake rotor stays centered within the intake mouth",
        )
        ctx.expect_overlap(
            sleeve,
            engine,
            axes="x",
            elem_a="sleeve_shell",
            elem_b="aft_nozzle_shell",
            min_overlap=0.17,
            name="collapsed exhaust sleeve covers the aft nozzle",
        )
        ctx.expect_contact(
            rotor,
            engine,
            elem_a="rotor_shaft",
            elem_b="shaft_bearing",
            name="rotor shaft seats against the engine bearing",
        )

    sleeve_rest = ctx.part_world_position(sleeve)
    with ctx.pose({sleeve_joint: 0.10, rotor_joint: 2.4}):
        ctx.expect_overlap(
            sleeve,
            engine,
            axes="x",
            elem_a="sleeve_shell",
            elem_b="aft_nozzle_shell",
            min_overlap=0.075,
            name="extended exhaust sleeve keeps retained insertion on the nozzle",
        )
        ctx.expect_overlap(
            sleeve,
            engine,
            axes="yz",
            elem_a="sleeve_shell",
            elem_b="aft_nozzle_shell",
            min_overlap=0.20,
            name="extended sleeve remains coaxial with the aft nozzle",
        )
        sleeve_extended = ctx.part_world_position(sleeve)

    ctx.check(
        "exhaust sleeve extends backward on the dolly",
        sleeve_rest is not None
        and sleeve_extended is not None
        and sleeve_extended[0] > sleeve_rest[0] + 0.08,
        details=f"rest={sleeve_rest}, extended={sleeve_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
